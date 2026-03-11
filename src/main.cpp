#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/simple_idt.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include "render_svg.h"
#include "utils.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geom;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

std::vector<polyscope::SurfaceMesh*> psMeshes;
std::vector<polyscope::CurveNetwork*> psCurves;
std::vector<polyscope::PointCloud*> psPoints;

size_t svgIndex = 0;


// ====== polar-dual.pdf camera
// {"farClipRatio":20.0,"fov":45.0,"nearClipRatio":0.005,"projectionMode":"Perspective","viewMat":[0.874567747116089,9.31322574615479e-10,-0.484901487827301,0.0851092860102654,-0.119920261204243,0.968936204910278,-0.216288238763809,0.000308383489027619,0.469838172197342,0.247308552265167,0.847401022911072,-1.46712577342987,0.0,0.0,0.0,1.0],"windowHeight":720,"windowWidth":1280}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {
    if (ImGui::Button("save svg")) {
        // renderPolyscopeToSVG("out_" + std::to_string(svgIndex++) + ".svg",
        // {},
        //                      psCurves, psPoints);
        for (size_t iMesh = 0; iMesh < psMeshes.size(); iMesh++) {
            renderPolyscopeToSVG(
                "out_mesh_" + std::to_string(iMesh) + "_version_" +
                    std::to_string(svgIndex) + ".svg",
                std::vector<polyscope::SurfaceMesh*>{psMeshes[iMesh]}, psCurves,
                psPoints);
        }
        svgIndex++;
    }
}

int main(int argc, char** argv) {

    // Configure the argument parser
    args::ArgumentParser parser("Geometry program");
    args::Positional<std::string> inputFilename(parser, "mesh",
                                                "Mesh to be processed.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&) {
        std::cout << parser;
        return 0;
    } catch (const args::ParseError& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    std::string filename = getDataPath("../meshes/bunny_small.obj");
    // Make sure a mesh name was given
    if (inputFilename) {
        filename = args::get(inputFilename);
    }

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // Load mesh
    std::tie(mesh, geom) = readManifoldSurfaceMesh(filename);

    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh(
        "primal polyhedron", geom->vertexPositions, mesh->getFaceVertexList());
    psMeshes.push_back(psMesh);

    psPoints.push_back(polyscope::registerPointCloud(
        "origin", std::vector<Vector3>{Vector3{0, 0, 0}}));

    std::vector<Vector3> faceMins, faceCenters, faceMinNormals,
        faceCenterNormals, polarDualVertices;
    for (Face f : mesh->faces()) {
        Vector3 pi = geom->vertexPositions[f.halfedge().tailVertex()],
                pj = geom->vertexPositions[f.halfedge().tipVertex()],
                pk = geom->vertexPositions[f.halfedge().next().tipVertex()];
        Vector3 n  = unit(cross(pj - pi, pk - pi));
        faceMins.push_back(dot(n, pi) * n);

        faceCenters.push_back(Vector3::zero());
        for (Vertex i : f.adjacentVertices())
            faceCenters.back() += geom->vertexPositions[i];
        faceCenters.back() /= f.degree();

        faceMinNormals.push_back(faceMins.back() + .15 * n);
        faceCenterNormals.push_back(faceCenters.back() + .15 * n);

        polarDualVertices.push_back(.15 * n / dot(n, pi));
    }

    psPoints.push_back(polyscope::registerPointCloud("faceMins", faceMins));
    psPoints.push_back(
        polyscope::registerPointCloud("faceCenters", faceCenters));
    psPoints.push_back(
        polyscope::registerPointCloud("faceMinNormals", faceMinNormals));
    psPoints.push_back(
        polyscope::registerPointCloud("faceCenterNormals", faceCenterNormals));

    std::vector<std::vector<size_t>> polarDualFaces;
    for (Vertex i : mesh->vertices()) {
        polarDualFaces.push_back({});
        for (Face f : i.adjacentFaces())
            polarDualFaces.back().push_back(f.getIndex());
        std::reverse(polarDualFaces.back().begin(),
                     polarDualFaces.back().end());
    }
    psMeshes.push_back(polyscope::registerSurfaceMesh(
        "dual polyhedron", polarDualVertices, polarDualFaces));

    renderPolyscopeToSVG("out_" + std::to_string(svgIndex++) + ".svg", psMesh);

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}
