#include "ConstrainedDelaunay2D.h"
#include "Delaunay2D.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/DiscretizationSettings2D.h"
#include "Geometry/2D/Base/GeometryOperations2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdgeLoop.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DFace.h"
#include "HoleTriangleRemover.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshVerifier.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Utils/MeshLogger.h"
#include "Common/Exceptions/MeshException.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>

namespace Meshing
{

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context, const std::vector<Point2D>& additionalPoints) :
    context_(&context),
    meshData2D_(&context.getMeshData()),
    meshMutator_(&context.getMutator()),
    meshOperations_(&context.getOperations())
{
    // Configure discretization settings (1 segment per edge = no subdivision)
    Geometry2D::DiscretizationSettings2D discretizationSettings(1, 2 * 3.1415 / 8);

    // Create geometry operations for this geometry
    Geometry2D::GeometryOperations2D geometryOps(context_->getGeometry());

    // Extract points from geometry with discretization
    auto pointsOnEdges = geometryOps.extractPointsWithEdgeDiscretization(context_->getTopology(),
                                                                         discretizationSettings);

    // Add additional points
    std::vector<Point2D> allPoints = std::move(pointsOnEdges.points);
    std::vector<std::vector<double>> allTParameters = std::move(pointsOnEdges.tParameters);
    std::vector<std::vector<std::string>> allGeometryIds = std::move(pointsOnEdges.geometryIds);
    allPoints.insert(allPoints.end(), additionalPoints.begin(), additionalPoints.end());
    // Additional points don't have edge parameters or geometry IDs (empty vectors)
    allTParameters.insert(allTParameters.end(), additionalPoints.size(), std::vector<double>{});
    allGeometryIds.insert(allGeometryIds.end(), additionalPoints.size(), std::vector<std::string>{});

    // Create Delaunay triangulation
    Delaunay2D delaunay(allPoints, meshData2D_, allTParameters, allGeometryIds);
    delaunay.triangulate();

    // Extract constrained edges
    constrainedEdges_ = meshOperations_->extractConstrainedEdges(
        context_->getTopology(),
        pointsOnEdges.cornerIdToPointIndexMap,
        delaunay.getPointIndexToNodeIdMap(),
        pointsOnEdges.edgeIdToPointIndicesMap);

    exportAndVerifyMesh();

    // Enforce all constrained edges
    bool allConstrainedEdgesPresent = false;
    while (!allConstrainedEdgesPresent)
    {
        allConstrainedEdgesPresent = true;
        for (const auto& segment : constrainedEdges_)
        {
            allConstrainedEdgesPresent = allConstrainedEdgesPresent &&
                                         meshOperations_->enforceEdge(segment.nodeId1, segment.nodeId2);
        }
    }
    exportAndVerifyMesh();
    return; // TODO: Fix bug in hole detection before proceeding

    // Remove triangles inside holes
    if (context_->getTopology().hasHoles())
    {
        removeHoleTriangles();
        exportAndVerifyMesh();
    }
}

ConstrainedDelaunay2D::~ConstrainedDelaunay2D() {

};

void ConstrainedDelaunay2D::triangulate() // TODO: Move code from constructor to here
{
}

std::vector<ConstrainedSegment2D> ConstrainedDelaunay2D::getConstrainedEdges() const
{
    return constrainedEdges_;
}

void ConstrainedDelaunay2D::exportAndVerifyMesh()
{
    Export::VtkExporter exporter;
    MeshData3D meshData3D(*meshData2D_);
    exporter.exportMesh(meshData3D, "constrained_delaunay_" + std::to_string(exportCounter_++) + ".vtu");

    bool allConstrinedEdgesPresent = false;
    // MeshLogger::logMeshData2D(*meshData2D_);
    MeshVerifier verifier(*meshData2D_);

    auto result = verifier.verify();
    if (!result.isValid)
    {
        for (const auto& error : result.errors)
        {
            spdlog::error(" - {}", error);
        }
        CMESH_THROW_VERIFICATION_FAILED("Mesh verification failed", result.errors);
    }
}

void ConstrainedDelaunay2D::removeHoleTriangles()
{
    auto face = buildDomainFace();
    HoleTriangleRemover remover(*meshData2D_, *meshMutator_, *face);
    remover.removeInvalidTriangles();
    spdlog::info("Removed {} triangles from holes", remover.getRemovedCount());
}

std::unique_ptr<Geometry2D::IFace2D> ConstrainedDelaunay2D::buildDomainFace() const
{
    const Geometry2D::GeometryCollection2D& geometry = context_->getGeometry();
    const Topology2D::Topology2D& topology = context_->getTopology();

    // Build outer edge loop
    auto outerLoop = std::make_unique<Geometry2D::OpenCascade2DEdgeLoop>(
        topology.getOuterEdgeLoop(), geometry);

    // Build face with outer loop
    auto face = std::make_unique<Geometry2D::OpenCascade2DFace>(std::move(outerLoop));

    // Add hole loops
    for (const auto& holeEdgeIds : topology.getHoleEdgeLoops())
    {
        auto holeLoop = std::make_unique<Geometry2D::OpenCascade2DEdgeLoop>(
            holeEdgeIds, geometry);
        face->addHole(std::move(holeLoop));
    }

    return face;
}

} // namespace Meshing
