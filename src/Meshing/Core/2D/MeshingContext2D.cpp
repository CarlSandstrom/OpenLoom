#include "MeshingContext2D.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include <set>

namespace Meshing
{

MeshingContext2D MeshingContext2D::fromSurface(const Geometry3D::ISurface3D& surface,
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
    const auto surfBounds = surface.getParameterBounds();
    const double uPeriod = surfBounds.getUMax() - surfBounds.getUMin();

    // Pass 1: seam twin edges — registers virtual "_seam" corners at U + uPeriod.
    // Must run before pass 2 so that closed-circle edges can resolve their shifted endpoints.
    auto addVirtualCorner = [&](const std::string& virtualId)
    {
        if (geometry2D->getCorner(virtualId) != nullptr)
            return;
        // Strip "_seam" to get the original corner ID
        std::string origCornerId = virtualId.substr(0, virtualId.size() - 5);
        auto* origCorner = fullGeometry.getCorner(origCornerId);
        if (!origCorner)
            return;
        Point2D uv = surface.projectPoint(origCorner->getPoint());
        geometry2D->addCorner(std::make_unique<Geometry2D::Corner2D>(
            virtualId, Point2D(uv.x() + uPeriod, uv.y())));
    };

    for (const auto& edgeId : edgeIds)
    {
        if (!fullTopology.getSeamCollection().isSeamTwin(edgeId))
            continue;

        const auto& topoEdge = fullTopology.getEdge(edgeId);
        const std::string& seamStartId = topoEdge.getStartCornerId();
        const std::string& seamEndId = topoEdge.getEndCornerId();

        addVirtualCorner(seamStartId);
        addVirtualCorner(seamEndId);

        auto* sc2D = geometry2D->getCorner(seamStartId);
        auto* ec2D = geometry2D->getCorner(seamEndId);
        if (sc2D && ec2D)
        {
            // The seam twin topology traverses from seamStart→seamEnd (reverse of the original
            // seam), but 3D parameter normalization assigns t=0 to the original seam start and
            // t=1 to the original seam end. To match those normalized t-values, the LinearEdge2D
            // must go in the original seam direction (ec2D → sc2D), not the twin direction.
            geometry2D->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
                edgeId, ec2D->getPoint(), sc2D->getPoint()));
        }
    }

    // Pass 2: non-seam-twin edges.
    // Closed-circle edges (start == end corner in 3D, e.g. top/bottom caps of a cylinder)
    // close at the seam-shifted virtual corner in UV space rather than the original corner.
    // Without this the LinearEdge2D would be zero-length, making getPoint(t) always return
    // the seam corner and producing invalid split midpoints during refinement.
    for (const auto& edgeId : edgeIds)
    {
        if (fullTopology.getSeamCollection().isSeamTwin(edgeId))
            continue;

        const auto& topoEdge = fullTopology.getEdge(edgeId);

        auto* startCorner2D = geometry2D->getCorner(topoEdge.getStartCornerId());
        auto* endCorner2D = geometry2D->getCorner(topoEdge.getEndCornerId());

        if (topoEdge.getStartCornerId() == topoEdge.getEndCornerId())
        {
            const std::string seamEndId = topoEdge.getEndCornerId() + "_seam";
            auto* seamEndCorner2D = geometry2D->getCorner(seamEndId);
            if (seamEndCorner2D != nullptr)
                endCorner2D = seamEndCorner2D;
        }

        if (startCorner2D != nullptr && endCorner2D != nullptr)
        {
            geometry2D->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
                edgeId, startCorner2D->getPoint(), endCorner2D->getPoint()));
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

        // For seam twin edges, register virtual corners in topoCorners so
        // Topology2D is consistent (corner IDs in edges must exist in corners map).
        if (fullTopology.getSeamCollection().isSeamTwin(edgeId))
        {
            for (const auto& cid : {topoEdge.getStartCornerId(), topoEdge.getEndCornerId()})
            {
                if (!topoCorners.count(cid))
                    topoCorners.emplace(cid, Topology2D::Corner2D(cid, {edgeId}));
            }
        }
    }

    // The boundary edge loop is the ordered list of edge IDs
    std::vector<std::string> boundaryLoop = edgeIds;

    auto topology2D = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryLoop);

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

const MeshData2D& MeshingContext2D::getMeshData() const
{
    return *meshData_;
}

MeshOperations2D& MeshingContext2D::getOperations()
{
    ensureInitialized();
    return *meshOperations_;
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
