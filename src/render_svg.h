#pragma once

#include <algorithm>
#include <fcpw/fcpw.h>
#include <fstream>
#include <geometrycentral/utilities/vector2.h>
#include <geometrycentral/utilities/vector3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iomanip>
#include <polyscope/camera_view.h>
#include <polyscope/curve_network.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <sstream>
#include <unordered_map>

#include "utils.h"

using geometrycentral::Vector2;
using geometrycentral::Vector3;

struct SVGRenderOptions {
    int width              = 1920;
    int height             = 1080;
    Vector3 lightDirection = Vector3{0.57735, 0.57735, 0.57735}; // normalized
    float ambientLight     = 0.3f;
    float diffuseLight     = 0.7f;
    float pointRadius      = 2.0f;
    float lineWidth        = 1.5f;
    float opacity          = 1.0f;
    bool includeOccluded   = true;
    std::string backgroundColor = "#ffffff";
};

// Helper struct for storing projected geometry with depth
struct ProjectedFace {
    std::vector<Vector2> vertices;
    float depth;
    Vector3 normal;
    std::array<float, 3> color;
    int meshIndex;
    bool isVisible;
};

struct ProjectedSegment {
    Vector2 p0, p1;
    float depth;
    std::array<float, 3> color;
    int curveIndex;
    bool isVisible;
};

struct ProjectedPoint {
    Vector2 pos;
    float depth;
    std::array<float, 3> color;
    int cloudIndex;
    bool isVisible;
};

// Convert Vector3 to glm::vec3
inline glm::vec3 toGLM(const Vector3& v) { return glm::vec3(v.x, v.y, v.z); }
inline fcpw::Vector3 toFCPW(const Vector3& v) {
    return {float(v.x), float(v.y), float(v.z)};
}
inline fcpw::Vector3 toFCPW(const glm::vec3& v) {
    return {float(v.x), float(v.y), float(v.z)};
}

// Convert glm::vec3 to Vector3
inline Vector3 fromGLM(const glm::vec3& v) { return Vector3{v.x, v.y, v.z}; }

// Project 3D point to 2D screen coordinates
Vector2 projectToScreen(const Vector3& worldPos, const glm::mat4& viewMat,
                        const glm::mat4& projMat, int width, int height);

// Build FCPW scene from polyscope meshes for visibility testing
void buildFCPWScene(fcpw::Scene<3>& scene,
                    const std::vector<polyscope::SurfaceMesh*>& meshes);

// Check visibility using ray casting
bool checkVisibility(const Vector3& point, const Vector3& cameraPos,
                     const fcpw::Scene<3>& scene, float epsilon = 1e-4f);

// Convert RGB to hex color string
std::string rgbToHex(const std::array<float, 3>& color);

// Main rendering function
void renderPolyscopeToSVG(const std::string& filename,
                          const std::vector<polyscope::SurfaceMesh*>& meshes,
                          const std::vector<polyscope::CurveNetwork*>& curves,
                          const std::vector<polyscope::PointCloud*>& clouds,
                          const SVGRenderOptions& options = SVGRenderOptions());

void renderPolyscopeToSVG(const std::string& filename,
                          polyscope::SurfaceMesh* mesh,
                          const SVGRenderOptions& options = SVGRenderOptions());
