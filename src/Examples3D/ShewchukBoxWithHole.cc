/**
 * @file ShewchukBoxWithHole.cc
 * @brief Example demonstrating ShewchukRefiner3D on a box with a cylindrical hole
 *
 * This example demonstrates mesh generation for a more complex domain:
 * a box with a cylindrical hole drilled through it. The workflow shows:
 * 1. Creating box and cylinder geometry using OpenCASCADE
 * 2. Boolean subtraction to create the hole
 * 3. Sampling boundary points including curved surfaces
 * 4. Setting up constraints for both planar and curved faces
 * 5. Applying ShewchukRefiner3D for quality mesh generation
 * 6. Exporting the refined mesh to VTK format
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/MeshOperations3D.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/Shewchuk3DQualityController.h"
#include "Meshing/Core/3D/ShewchukRefiner3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "spdlog/spdlog.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

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

    // Sample interior points only (edges are sampled separately)
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

    spdlog::info("=== ShewchukRefiner3D Box with Cylindrical Hole Example ===");

    // =========================================
    // Step 1: Create box with cylindrical hole
    // =========================================
    spdlog::info("Creating box with cylindrical hole geometry...");

    // Create a 10x10x10 box
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // Create a cylinder through the center (axis along Z, radius 2)
    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();

    // Boolean subtraction: box - cylinder = box with hole
    TopoDS_Shape boxWithHole = BRepAlgoAPI_Cut(box, cylinder).Shape();

    spdlog::info("Box: 10x10x10, Hole: cylinder radius 2, centered at (5,5)");

    // Convert to topology and geometry
    Readers::TopoDS_ShapeConverter converter(boxWithHole);

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
    std::map<std::string, size_t> cornerNodeIds;

    // Sample corner points
    for (const auto& cornerId : topology->getAllCornerIds())
    {
        const auto* corner = geometry->getCorner(cornerId);
        allPoints.push_back(corner->getPoint());
    }

    const size_t numCorners = allPoints.size();

    // Sample edge points - use more samples for curved edges
    const size_t straightEdgeSamples = 3;
    const size_t curvedEdgeSamples = 8; // More samples for circular edges

    std::map<std::string, std::vector<size_t>> edgeNodeIds;

    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto* edge = geometry->getEdge(edgeId);
        auto [tMin, tMax] = edge->getParameterBounds();

        // Detect if edge is curved by checking if midpoint deviates from straight line
        Point3D start = edge->getPoint(tMin);
        Point3D end = edge->getPoint(tMax);
        Point3D mid = edge->getPoint((tMin + tMax) / 2.0);
        Point3D expectedMid = (start + end) * 0.5;
        double deviation = (mid - expectedMid).norm();

        size_t numSamples = (deviation > 0.01) ? curvedEdgeSamples : straightEdgeSamples;

        auto samples = sampleEdge(*edge, numSamples + 1);

        // Skip first and last (corners)
        for (size_t i = 1; i < samples.size() - 1; ++i)
        {
            allPoints.push_back(samples[i].second);
        }
    }

    // Sample surface points - more samples for curved surfaces
    const size_t planarSurfaceSamples = 2;
    const size_t curvedSurfaceSamples = 4;

    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto* surface = geometry->getSurface(surfaceId);
        auto surfBounds = surface->getParameterBounds();
        double uMin = surfBounds.getUMin();
        double uMax = surfBounds.getUMax();
        double vMin = surfBounds.getVMin();
        double vMax = surfBounds.getVMax();

        // Detect if surface is curved by checking corner vs center
        Point3D corner = surface->getPoint(uMin, vMin);
        Point3D center = surface->getPoint((uMin + uMax) / 2.0, (vMin + vMax) / 2.0);
        Point3D expectedCenter = (surface->getPoint(uMin, vMin) + surface->getPoint(uMax, vMax)) * 0.5;
        double deviation = (center - expectedCenter).norm();

        size_t samples = (deviation > 0.01) ? curvedSurfaceSamples : planarSurfaceSamples;

        auto pts = sampleSurface(*surface, samples + 1, samples + 1);
        for (const auto& pt : pts)
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
    // Step 4: Set up constraints
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
    size_t edgePointIdx = numCorners;
    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto& topoEdge = topology->getEdge(edgeId);
        const auto* edge = geometry->getEdge(edgeId);

        size_t startNode = cornerNodeIds[topoEdge.getStartCornerId()];
        size_t endNode = cornerNodeIds[topoEdge.getEndCornerId()];

        // Determine number of interior samples used for this edge
        auto [tMin, tMax] = edge->getParameterBounds();
        Point3D start = edge->getPoint(tMin);
        Point3D end = edge->getPoint(tMax);
        Point3D mid = edge->getPoint((tMin + tMax) / 2.0);
        Point3D expectedMid = (start + end) * 0.5;
        double deviation = (mid - expectedMid).norm();
        size_t numSamples = (deviation > 0.01) ? curvedEdgeSamples : straightEdgeSamples;

        // Build chain of node IDs for this edge
        std::vector<size_t> edgeChain;
        edgeChain.push_back(startNode);
        for (size_t i = 0; i < numSamples; ++i)
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

    // Create subfacets for each surface
    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto& topoSurface = topology->getSurface(surfaceId);
        const auto& boundaryEdgeIds = topoSurface.getBoundaryEdgeIds();

        // Collect all boundary nodes for this face
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

    // Quality parameters:
    // B = 2.5 (> 2 for guaranteed termination)
    // Element limit = 10000 (more elements due to complex geometry)
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 10000);

    ShewchukRefiner3D refiner(context, qualityController, subsegments, subfacets);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 6: Export to VTK
    // =========================================
    spdlog::info("Exporting mesh to shewchuk_box_with_hole.vtu...");

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "shewchuk_box_with_hole.vtu");

    spdlog::info("=== Example Complete ===");
    spdlog::info("View in ParaView: paraview shewchuk_box_with_hole.vtu");
    spdlog::info("");
    spdlog::info("Notes:");
    spdlog::info("  - The cylindrical hole creates curved boundary surfaces");
    spdlog::info("  - More samples are used for curved edges and surfaces");
    spdlog::info("  - The refiner respects both planar and curved constraints");

    return 0;
}
