#include "ConstrainedDelaunay2D.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include "Delaunay2D.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/DiscretizationSettings2D.h"
#include "Geometry/2D/Base/GeometryOperations2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshVerifier.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "spdlog/spdlog.h"
#include "Utils/MeshLogger.h"

namespace Meshing
{

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context, const std::vector<Point2D>& additionalPoints) :
    context_(&context),
    meshData2D_(&context.getMeshData()),
    meshMutator_(&context.getMutator()),
    meshOperations_(&context.getOperations())
{
    // Configure discretization settings (1 segment per edge = no subdivision)
    Geometry2D::DiscretizationSettings2D discretizationSettings(1);

    // Create geometry operations for this geometry
    Geometry2D::GeometryOperations2D geometryOps(context_->getGeometry());

    // Extract points from geometry with discretization
    auto extractionResult = geometryOps.extractPointsWithEdgeDiscretization(
        context_->getTopology(),
        discretizationSettings);

    // Add additional points
    std::vector<Point2D> allPoints = std::move(extractionResult.points);
    allPoints.insert(allPoints.end(), additionalPoints.begin(), additionalPoints.end());

    // Create Delaunay triangulation
    Delaunay2D delaunay(allPoints, meshData2D_);
    delaunay.triangulate();

    // Extract constrained edges
    constrainedEdges_ = meshOperations_->extractConstrainedEdges(
        context_->getTopology(),
        extractionResult.cornerIdToPointIndexMap,
        delaunay.getPointIndexToNodeIdMap());

    exportAndVerifyMesh();

    // Enforce all constrained edges
    bool allConstrainedEdgesPresent = false;
    while (!allConstrainedEdgesPresent)
    {
        allConstrainedEdgesPresent = true;
        for (const auto& edge : constrainedEdges_)
        {
            allConstrainedEdgesPresent = allConstrainedEdgesPresent &&
                                         meshOperations_->enforceEdge(edge.first, edge.second);
            exportAndVerifyMesh();
        }
    }
}

ConstrainedDelaunay2D::~ConstrainedDelaunay2D() {

};

void ConstrainedDelaunay2D::triangulate()
{
}

void ConstrainedDelaunay2D::exportAndVerifyMesh()
{
    Export::VtkExporter exporter;
    MeshData3D meshData3D(*meshData2D_);
    exporter.exportMesh(meshData3D, "constrained_delaunay_" + std::to_string(exportCounter_++) + ".vtu");

    bool allConstrinedEdgesPresent = false;
    MeshLogger::logMeshData2D(*meshData2D_);
    MeshVerifier verifier(*meshData2D_);

    auto result = verifier.verify();
    if (!result.isValid)
    {
        for (const auto& error : result.errors)
        {
            spdlog::error(" - {}", error);
        }
        throw std::runtime_error("Mesh verification failed");
    }
}

} // namespace Meshing
