/**
 * @file ShewchukBox.cc
 * @brief Example demonstrating ShewchukRefiner3D on a simple box
 *
 * This example shows the complete workflow for generating a quality
 * tetrahedral mesh of a box using Shewchuk's Delaunay refinement algorithm:
 * 1. Create box geometry using OpenCASCADE
 * 2. Sample boundary points and create initial Delaunay triangulation
 * 3. Set up constraint subsegments (edges) and subfacets (faces)
 * 4. Apply ShewchukRefiner3D to improve mesh quality
 * 5. Export the refined mesh to VTK format
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/MeshOperations3D.h"
#include "Meshing/Core/3D/Shewchuk3DQualityController.h"
#include "Meshing/Core/3D/ShewchukRefiner3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "spdlog/spdlog.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

#include <map>
#include <set>
#include <vector>

using namespace Meshing;

/**
 * @brief Sample points along an edge from the geometry
 */
std::vector<std::pair<double, Point3D>> sampleEdge(
    const Geometry3D::IEdge3D& edge,
    size_t numSamples)
{
    std::vector<std::pair<double, Point3D>> samples;
    auto [tMin, tMax] = edge.getParameterBounds();

    for (size_t i = 0; i <= numSamples; ++i)
    {
        double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(numSamples);
        samples.emplace_back(t, edge.getPoint(t));
    }
    return samples;
}

/**
 * @brief Sample points on a surface in a grid pattern
 */
std::vector<Point3D> sampleSurface(
    const Geometry3D::ISurface3D& surface,
    size_t uSamples,
    size_t vSamples)
{
    std::vector<Point3D> samples;
    auto bounds = surface.getParameterBounds();
    double uMin = bounds.getUMin();
    double uMax = bounds.getUMax();
    double vMin = bounds.getVMin();
    double vMax = bounds.getVMax();

    // Only sample interior points (edges are sampled separately)
    for (size_t i = 1; i < uSamples; ++i)
    {
        double u = uMin + (uMax - uMin) * static_cast<double>(i) / static_cast<double>(uSamples);
        for (size_t j = 1; j < vSamples; ++j)
        {
            double v = vMin + (vMax - vMin) * static_cast<double>(j) / static_cast<double>(vSamples);
            samples.push_back(surface.getPoint(u, v));
        }
    }
    return samples;
}

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("=== ShewchukRefiner3D Box Example ===");

    // =========================================
    // Step 1: Create box geometry with OpenCASCADE
    // =========================================
    spdlog::info("Creating 10x10x10 box geometry...");

    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(box);

    Meshing::MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    const auto* geometry = context.getGeometry();
    const auto* topology = context.getTopology();

    spdlog::info("Topology: {} corners, {} edges, {} surfaces",
                 topology->getAllCornerIds().size(),
                 topology->getAllEdgeIds().size(),
                 topology->getAllSurfaceIds().size());

    // =========================================
    // Step 2: Sample boundary points
    // =========================================
    spdlog::info("Sampling boundary points...");

    std::vector<Point3D> allPoints;
    std::map<std::string, size_t> cornerNodeIds; // corner ID -> node ID

    // Sample corner points
    for (const auto& cornerId : topology->getAllCornerIds())
    {
        const auto* corner = geometry->getCorner(cornerId);
        allPoints.push_back(corner->getPoint());
    }

    const size_t numCorners = allPoints.size();

    // Sample edge points (interior only, excluding endpoints)
    const size_t edgeSamples = 3; // Number of interior samples per edge

    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto* edge = geometry->getEdge(edgeId);
        auto samples = sampleEdge(*edge, edgeSamples + 1);

        // Skip first and last (corners)
        for (size_t i = 1; i < samples.size() - 1; ++i)
        {
            allPoints.push_back(samples[i].second);
        }
    }

    // Sample surface points (interior only)
    const size_t surfaceSamples = 2;
    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto* surface = geometry->getSurface(surfaceId);
        auto samples = sampleSurface(*surface, surfaceSamples + 1, surfaceSamples + 1);
        for (const auto& pt : samples)
        {
            allPoints.push_back(pt);
        }
    }

    spdlog::info("Total boundary points: {}", allPoints.size());

    // =========================================
    // Step 3: Initialize Delaunay triangulation
    // =========================================
    spdlog::info("Initializing Delaunay triangulation...");

    auto nodeIds = context.getOperations().initializeDelaunay(allPoints);

    spdlog::info("Initial mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 4: Set up constraints (subsegments and subfacets)
    // =========================================
    spdlog::info("Setting up boundary constraints...");

    std::vector<ConstrainedSubsegment3D>& subsegments = context.getConstrainedSubsegments();
    std::vector<ConstrainedSubfacet3D>& subfacets = context.getConstrainedSubfacets();

    // Map corner IDs to node IDs
    size_t cornerIdx = 0;
    for (const auto& cornerId : topology->getAllCornerIds())
    {
        cornerNodeIds[cornerId] = nodeIds[cornerIdx++];
    }

    // Create subsegments for each edge
    std::map<std::string, std::vector<size_t>> edgeNodeIds;
    size_t edgePointIdx = numCorners;
    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto& topoEdge = topology->getEdge(edgeId);
        size_t startNode = cornerNodeIds[topoEdge.getStartCornerId()];
        size_t endNode = cornerNodeIds[topoEdge.getEndCornerId()];

        // Build chain of node IDs for this edge
        std::vector<size_t> edgeChain;
        edgeChain.push_back(startNode);
        for (size_t i = 0; i < edgeSamples; ++i)
        {
            edgeChain.push_back(nodeIds[edgePointIdx++]);
        }
        edgeChain.push_back(endNode);

        // Create subsegments
        for (size_t i = 0; i + 1 < edgeChain.size(); ++i)
        {
            ConstrainedSubsegment3D seg;
            seg.nodeId1 = edgeChain[i];
            seg.nodeId2 = edgeChain[i + 1];
            seg.geometryId = edgeId;
            subsegments.push_back(seg);
        }

        edgeNodeIds[edgeId] = edgeChain;
    }

    // Create subfacets for each surface (simple triangulation of boundary)
    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto& topoSurface = topology->getSurface(surfaceId);
        const auto& boundaryEdgeIds = topoSurface.getBoundaryEdgeIds();

        // Collect all boundary nodes for this face, avoiding duplicates
        std::vector<size_t> faceNodes;
        std::set<size_t> seenNodes;
        for (const auto& edgeId : boundaryEdgeIds)
        {
            const auto& chain = edgeNodeIds[edgeId];
            for (size_t nodeId : chain)
            {
                if (seenNodes.find(nodeId) == seenNodes.end())
                {
                    faceNodes.push_back(nodeId);
                    seenNodes.insert(nodeId);
                }
            }
        }

        // Simple fan triangulation from first vertex
        if (faceNodes.size() >= 3)
        {
            for (size_t i = 1; i + 1 < faceNodes.size(); ++i)
            {
                ConstrainedSubfacet3D facet;
                facet.nodeId1 = faceNodes[0];
                facet.nodeId2 = faceNodes[i];
                facet.nodeId3 = faceNodes[i + 1];
                facet.geometryId = surfaceId;
                subfacets.push_back(facet);
            }
        }
    }

    spdlog::info("Constraints: {} subsegments, {} subfacets",
                 subsegments.size(), subfacets.size());

    // =========================================
    // Step 5: Apply ShewchukRefiner3D
    // =========================================
    spdlog::info("Starting Shewchuk refinement...");

    // Quality parameters: B = 2.5 (> 2 for guaranteed termination), max 5000 elements
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 5000);

    ShewchukRefiner3D refiner(context, qualityController, subsegments, subfacets);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 6: Export to VTK
    // =========================================
    spdlog::info("Exporting mesh to shewchuk_box.vtu...");

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "shewchuk_box.vtu");

    spdlog::info("=== Example Complete ===");
    spdlog::info("View in ParaView: paraview shewchuk_box.vtu");

    return 0;
}
