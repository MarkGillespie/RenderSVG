#include "render_svg.h"

// Project 3D point to 2D screen coordinates
Vector2 projectToScreen(const Vector3& worldPos, const glm::mat4& viewMat,
                        const glm::mat4& projMat, int width, int height) {
    glm::vec4 pos(worldPos.x, worldPos.y, worldPos.z, 1.0f);

    // Transform to clip space
    glm::vec4 clipPos = projMat * viewMat * pos;

    // Perspective divide to NDC [-1,1]
    glm::vec3 ndc = glm::vec3(clipPos) / clipPos.w;

    // Convert to screen coordinates
    float x = (ndc.x + 1.0f) * 0.5f * width;
    float y = (1.0f - ndc.y) * 0.5f * height; // Flip Y for SVG

    return Vector2{x, y};
}

// Build FCPW scene from polyscope meshes for visibility testing
void buildFCPWScene(fcpw::Scene<3>& scene,
                    const std::vector<polyscope::SurfaceMesh*>& meshes) {
    std::vector<fcpw::Vector3> allVertices;
    std::vector<fcpw::Vector3i> allTriangles;

    for (auto* mesh : meshes) {
        int vOffset = allVertices.size();

        for (const glm::vec3& v : mesh->vertexPositions.data)
            allVertices.push_back(toFCPW(v));

        auto& faces = mesh->triangleVertexInds.data;
        for (size_t iF = 0; iF < mesh->nFacesTriangulation(); iF++) {
            allTriangles.push_back(fcpw::Vector3i(faces[3 * iF + 0] + vOffset,
                                                  faces[3 * iF + 1] + vOffset,
                                                  faces[3 * iF + 2] + vOffset));
        }
    }

    if (!allTriangles.empty()) {
        scene.setObjectCount(1);
        scene.setObjectVertices(allVertices, 0);
        scene.setObjectTriangles(allTriangles, 0);
        scene.build(fcpw::AggregateType::Bvh_SurfaceArea, true);
    }
}

// Check visibility using ray casting
bool checkVisibility(const Vector3& point, const Vector3& cameraPos,
                     const fcpw::Scene<3>& scene, float epsilon) {
    Vector3 dir   = point - cameraPos;
    float maxDist = norm(dir);

    fcpw::Ray<3> ray(toFCPW(cameraPos), toFCPW(dir.unit()), maxDist - epsilon);

    fcpw::Interaction<3> interaction;
    return !scene.intersect(ray, interaction, true); // check occlusion only
}

// Convert RGB to hex color string
std::string rgbToHex(const std::array<float, 3>& color) {
    std::stringstream ss;
    ss << "#";
    for (int i = 0; i < 3; ++i) {
        int val = static_cast<int>(color[i] * 255.0f);
        ss << std::hex << std::setfill('0') << std::setw(2) << val;
    }
    return ss.str();
}

// Main rendering function
void renderPolyscopeToSVG(const std::string& filename,
                          const std::vector<polyscope::SurfaceMesh*>& meshes,
                          const std::vector<polyscope::CurveNetwork*>& curves,
                          const std::vector<polyscope::PointCloud*>& clouds,
                          const SVGRenderOptions& options) {

    // Get camera parameters
    auto cameraParams = polyscope::view::getCameraParametersForCurrentView();
    glm::mat4 viewMat = cameraParams.extrinsics.getViewMat();
    glm::mat4 projMat = polyscope::view::getCameraPerspectiveMatrix();
    Vector3 cameraPos = fromGLM(cameraParams.extrinsics.getPosition());

    // Build FCPW scene for visibility testing
    fcpw::Scene<3> fcpwScene;
    buildFCPWScene(fcpwScene, meshes);

    // Collect all projected geometry
    std::vector<ProjectedFace> projectedFaces;
    std::vector<ProjectedSegment> projectedSegments;
    std::vector<ProjectedPoint> projectedPoints;

    // Process meshes
    bool hasOccludedFaces = false, hasVisibleFaces = false;
    for (size_t meshIdx = 0; meshIdx < meshes.size(); ++meshIdx) {
        auto* mesh  = meshes[meshIdx];
        auto& p     = mesh->vertexPositions.data;
        auto& faces = mesh->triangleVertexInds.data;

        // Get mesh color (assuming uniform color)
        glm::vec3 meshColor = mesh->getSurfaceColor();

        for (size_t iF = 0; iF + 1 < mesh->faceIndsStart.size(); iF++) {
            ProjectedFace pf;
            pf.meshIndex = meshIdx;
            pf.color     = {meshColor.x, meshColor.y, meshColor.z};

            // read off positions for face vertices from polyscope mesh
            std::vector<Vector3> v;
            size_t iStart  = mesh->faceIndsStart[iF];
            size_t iEnd    = mesh->faceIndsStart[iF + 1];
            Vector3 center = Vector3::zero();
            for (size_t iV = iStart; iV < iEnd; iV++) {
                v.push_back(fromGLM(p[mesh->faceIndsEntries[iV]]));
                center += v.back();
            }
            center /= v.size();
            size_t degree = v.size();
            if (degree < 3) {
                std::cout << "err: faces must have degree at least 3" << vendl;
                std::cout << "Face " << iF << vendl;
                std::cout << "   ind start: " << iStart << " of "
                          << mesh->faceIndsEntries.size() << vendl;
                std::cout << "     ind end: " << iEnd << " of "
                          << mesh->faceIndsEntries.size() << vendl;
                exit(1);
            }

            Vector3 normal = cross(v[1] - v[0], v[2] - v[0]);
            normal         = normal / norm(normal);
            pf.normal      = normal;

            // Project vertices
            for (size_t i = 0; i < v.size(); ++i) {
                pf.vertices.push_back(projectToScreen(
                    v[i], viewMat, projMat, options.width, options.height));
            }

            // Compute depth (distance from camera)
            pf.depth = norm(center - cameraPos);

            // Check visibility
            pf.isVisible = checkVisibility(center, cameraPos, fcpwScene);
            if (pf.isVisible) {
                hasVisibleFaces = true;
            } else {
                hasOccludedFaces = true;
            }

            projectedFaces.push_back(pf);
        }
    }

    // Process curve networks
    bool hasOccludedSegments = false, hasVisibleSegments = false;
    for (size_t curveIdx = 0; curveIdx < curves.size(); ++curveIdx) {
        auto* curve          = curves[curveIdx];
        auto& p              = curve->nodePositions.data;
        auto& eTail          = curve->edgeTailInds.data;
        auto& eTip           = curve->edgeTipInds.data;
        glm::vec3 curveColor = curve->getColor();

        for (size_t iE = 0; iE < curve->nEdges(); iE++) {
            ProjectedSegment ps;
            ps.curveIndex = curveIdx;
            ps.color      = {curveColor.x, curveColor.y, curveColor.z};

            Vector3 v0     = fromGLM(p[eTail[iE]]);
            Vector3 v1     = fromGLM(p[eTip[iE]]);
            Vector3 center = (v0 + v1) / 2.0f;

            ps.p0        = projectToScreen(v0, viewMat, projMat, options.width,
                                           options.height);
            ps.p1        = projectToScreen(v1, viewMat, projMat, options.width,
                                           options.height);
            ps.depth     = norm(center - cameraPos);
            ps.isVisible = checkVisibility(center, cameraPos, fcpwScene);
            if (ps.isVisible) {
                hasVisibleSegments = true;
            } else {
                hasOccludedSegments = true;
            }

            projectedSegments.push_back(ps);
        }
    }

    // Process point clouds
    bool hasOccludedPoints = false, hasVisiblePoints = false;
    for (size_t cloudIdx = 0; cloudIdx < clouds.size(); ++cloudIdx) {
        auto* cloud          = clouds[cloudIdx];
        auto& positions      = cloud->points.data;
        glm::vec3 pointColor = cloud->getPointColor();

        for (glm::vec3& pos : positions) {
            ProjectedPoint pp;
            pp.cloudIndex = cloudIdx;
            pp.color      = {pointColor.x, pointColor.y, pointColor.z};

            Vector3 worldPos = fromGLM(pos);
            pp.pos = projectToScreen(worldPos, viewMat, projMat, options.width,
                                     options.height);
            pp.depth     = norm(worldPos - cameraPos);
            pp.isVisible = checkVisibility(worldPos, cameraPos, fcpwScene);
            if (pp.isVisible) {
                hasVisiblePoints = true;
            } else {
                hasOccludedPoints = true;
            }

            projectedPoints.push_back(pp);
        }
    }

    // Sort by depth (back to front for proper rendering)
    std::sort(projectedFaces.begin(), projectedFaces.end(),
              [](const ProjectedFace& a, const ProjectedFace& b) {
                  return a.depth > b.depth;
              });

    std::sort(projectedSegments.begin(), projectedSegments.end(),
              [](const ProjectedSegment& a, const ProjectedSegment& b) {
                  return a.depth > b.depth;
              });

    std::sort(projectedPoints.begin(), projectedPoints.end(),
              [](const ProjectedPoint& a, const ProjectedPoint& b) {
                  return a.depth > b.depth;
              });

    // Generate SVG
    std::ofstream svg(filename);
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
    svg << "width=\"" << options.width << "\" ";
    svg << "height=\"" << options.height << "\" ";
    svg << "viewBox=\"0 0 " << options.width << " " << options.height
        << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\""
        << options.backgroundColor << "\"/>\n";

    if (options.includeOccluded && hasOccludedFaces) { // Render occluded faces
        svg << "<g id=\"occluded-faces\" opacity=\"0.3\">\n";
        for (const auto& face : projectedFaces) {
            if (!face.isVisible) {
                // Calculate shading
                float shade =
                    options.ambientLight +
                    options.diffuseLight *
                        std::max(0.0f,
                                 static_cast<float>(
                                     dot(face.normal, options.lightDirection)));

                std::array<float, 3> shadedColor = {face.color[0] * shade,
                                                    face.color[1] * shade,
                                                    face.color[2] * shade};

                svg << "  <polygon points=\"";
                for (const auto& v : face.vertices) {
                    svg << v.x << "," << v.y << " ";
                }
                svg << "\" fill=\"" << rgbToHex(shadedColor) << "\" ";
                svg << "stroke=\"none\" opacity=\"" << options.opacity
                    << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    if (hasVisibleFaces) { // Render visible faces
        svg << "<g id=\"visible-faces\">\n";
        for (const auto& face : projectedFaces) {
            if (face.isVisible) {
                // Calculate shading
                float shade =
                    options.ambientLight +
                    options.diffuseLight *
                        std::max(0.0f,
                                 static_cast<float>(
                                     dot(face.normal, options.lightDirection)));

                std::array<float, 3> shadedColor = {face.color[0] * shade,
                                                    face.color[1] * shade,
                                                    face.color[2] * shade};

                svg << "  <polygon points=\"";
                for (const auto& v : face.vertices) {
                    svg << v.x << "," << v.y << " ";
                }
                svg << "\" fill=\"" << rgbToHex(shadedColor) << "\" ";
                svg << "stroke=\"#000000\" stroke-width=\"0.5\" ";
                svg << "opacity=\"" << options.opacity << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    if (options.includeOccluded &&
        hasOccludedSegments) { // Render occluded segments
        svg << "<g id=\"occluded-segments\" opacity=\"0.3\">\n";
        for (const auto& seg : projectedSegments) {
            if (!seg.isVisible) {
                svg << "  <line x1=\"" << seg.p0.x << "\" y1=\"" << seg.p0.y
                    << "\" ";
                svg << "x2=\"" << seg.p1.x << "\" y2=\"" << seg.p1.y << "\" ";
                svg << "stroke=\"" << rgbToHex(seg.color) << "\" ";
                svg << "stroke-width=\"" << options.lineWidth << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    if (hasVisibleSegments) { // Render visible segments
        svg << "<g id=\"visible-segments\">\n";
        for (const auto& seg : projectedSegments) {
            if (seg.isVisible) {
                svg << "  <line x1=\"" << seg.p0.x << "\" y1=\"" << seg.p0.y
                    << "\" ";
                svg << "x2=\"" << seg.p1.x << "\" y2=\"" << seg.p1.y << "\" ";
                svg << "stroke=\"" << rgbToHex(seg.color) << "\" ";
                svg << "stroke-width=\"" << options.lineWidth << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    if (options.includeOccluded &&
        hasOccludedPoints) { // Render occluded points
        svg << "<g id=\"occluded-points\" opacity=\"0.3\">\n";
        for (const auto& pt : projectedPoints) {
            if (!pt.isVisible) {
                svg << "  <circle cx=\"" << pt.pos.x << "\" cy=\"" << pt.pos.y
                    << "\" ";
                svg << "r=\"" << options.pointRadius << "\" ";
                svg << "fill=\"" << rgbToHex(pt.color) << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    if (hasVisiblePoints) { // Render visible points
        svg << "<g id=\"visible-points\">\n";
        for (const auto& pt : projectedPoints) {
            if (pt.isVisible) {
                svg << "  <circle cx=\"" << pt.pos.x << "\" cy=\"" << pt.pos.y
                    << "\" ";
                svg << "r=\"" << options.pointRadius << "\" ";
                svg << "fill=\"" << rgbToHex(pt.color) << "\"/>\n";
            }
        }
        svg << "</g>\n";
    }

    svg << "</svg>\n";
    svg.close();
}

void renderPolyscopeToSVG(const std::string& filename,
                          polyscope::SurfaceMesh* mesh,
                          const SVGRenderOptions& options) {
    return renderPolyscopeToSVG(filename, {mesh}, {}, {}, options);
}
