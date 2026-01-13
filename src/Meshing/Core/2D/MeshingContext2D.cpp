#include "MeshingContext2D.h"
#include <set>
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/IFace2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DFaceBuilder.h"
#include "MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

MeshingContext2D MeshingContext2D::fromSurface(
    const Geometry3D::ISurface3D& surface,
    const Topology3D::Surface3D& topoSurface,
    const Topology3D::Topology3D& fullTopology,
    const Geometry3D::GeometryCollection3D& fullGeometry)
{
    // Create 2D geometry collection
    auto geometry2D = std::make_unique<Geometry2D::GeometryCollection2D>();

    // Get boundary corners and project them to 2D parametric space
    const auto& cornerIds = topoSurface.getCornerIds();
    for (const auto& cornerId : cornerIds)
    {
        auto* corner3D = fullGeometry.getCorner(cornerId);
        if (corner3D != nullptr)
        {
            // Project 3D corner to 2D parametric coordinates
            Point2D coord2D = surface.projectPoint(corner3D->getPoint());
            auto corner2D = std::make_unique<Geometry2D::Corner2D>(cornerId, coord2D);
            geometry2D->addCorner(std::move(corner2D));
        }
    }

    // Get boundary edges and project them to 2D parametric space
    const auto& edgeIds = topoSurface.getBoundaryEdgeIds();
    for (const auto& edgeId : edgeIds)
    {
        const auto& topoEdge = fullTopology.getEdge(edgeId);
        auto* edge3D = fullGeometry.getEdge(edgeId);

        if (edge3D != nullptr)
        {
            // Get start and end corners in 2D
            auto* startCorner2D = geometry2D->getCorner(topoEdge.getStartCornerId());
            auto* endCorner2D = geometry2D->getCorner(topoEdge.getEndCornerId());

            if (startCorner2D != nullptr && endCorner2D != nullptr)
            {
                // Create linear edge in 2D parametric space
                auto edge2D = std::make_unique<Geometry2D::LinearEdge2D>(
                    edgeId,
                    startCorner2D->getPoint(),
                    endCorner2D->getPoint());
                geometry2D->addEdge(std::move(edge2D));
            }
        }
    }

    // Create 2D topology
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    // Build corner connectivity
    for (const auto& cornerId : cornerIds)
    {
        std::set<std::string> connectedEdges;

        // Find edges that connect to this corner
        for (const auto& edgeId : edgeIds)
        {
            const auto& topoEdge = fullTopology.getEdge(edgeId);
            if (topoEdge.getStartCornerId() == cornerId || topoEdge.getEndCornerId() == cornerId)
            {
                connectedEdges.insert(edgeId);
            }
        }

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, connectedEdges));
    }

    // Build edges
    for (const auto& edgeId : edgeIds)
    {
        const auto& topoEdge = fullTopology.getEdge(edgeId);
        topoEdges.emplace(edgeId, Topology2D::Edge2D(
                                      edgeId,
                                      topoEdge.getStartCornerId(),
                                      topoEdge.getEndCornerId()));
    }

    // The boundary edge loop is the ordered list of edge IDs
    std::vector<std::string> boundaryLoop = edgeIds;

    auto topology2D = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, boundaryLoop);

    return MeshingContext2D(std::move(geometry2D), std::move(topology2D));
}

MeshingContext2D::MeshingContext2D(
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry,
    std::unique_ptr<Topology2D::Topology2D> topology) :
    geometry_(std::move(geometry)),
    topology_(std::move(topology))
{
    ensureInitialized();
}

MeshingContext2D::~MeshingContext2D() = default;

MeshingContext2D::MeshingContext2D(MeshingContext2D&&) noexcept = default;
MeshingContext2D& MeshingContext2D::operator=(MeshingContext2D&&) noexcept = default;

MeshData2D& MeshingContext2D::getMeshData()
{
    ensureInitialized();
    return *meshData_;
}

MeshOperations2D& MeshingContext2D::getOperations()
{
    ensureInitialized();
    return *meshOperations_;
}

MeshMutator2D& MeshingContext2D::getMutator()
{
    ensureInitialized();
    return *meshMutator_;
}

void MeshingContext2D::clearMesh()
{
    meshData_ = std::make_unique<MeshData2D>();
    meshMutator_ = std::make_unique<MeshMutator2D>(*meshData_);
}

std::unique_ptr<Geometry2D::IFace2D> MeshingContext2D::buildDomainFace() const
{
    return Geometry2D::OpenCascade2DFaceBuilder::buildFromTopology(
        *topology_, *geometry_);
}

void MeshingContext2D::ensureInitialized()
{
    if (!meshData_)
    {
        meshData_ = std::make_unique<MeshData2D>();
    }
    if (!meshMutator_)
    {
        meshMutator_ = std::make_unique<MeshMutator2D>(*meshData_);
    }
    if (!meshOperations_)
    {
        meshOperations_ = std::make_unique<MeshOperations2D>(*meshData_);
    }
}

} // namespace Meshing
