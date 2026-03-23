#include "MeshingContext2D.h"
#include "PeriodicMeshData2D.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/Base/SurfaceProjectedEdge2D.h"
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
    const auto& seams = fullTopology.getSeamCollection();

    // Pass 1: seam twin edges.
    // Creates edge-specific Corner2D objects at the precomputed UV positions stored in
    // SeamCollection (derived from BRep_Tool::CurveOnSurface in the converter). Using
    // edge-specific corner IDs ("<twinId>_start" / "<twinId>_end") avoids UV ambiguity
    // for doubly-periodic surfaces (e.g. torus) where multiple twins share the same 3D vertex.
    //
    // Builds direction-separated shifted-corner maps for pass 2:
    //   uShiftedCorner: corner → U-shifted ICorner2D* (from U-seam twins)
    //   vShiftedCorner: corner → V-shifted ICorner2D* (from V-seam twins)
    // Using two maps avoids overwrite conflicts on doubly-periodic surfaces where a single
    // 3D vertex appears in both U-seam and V-seam twins (e.g. the torus shared vertex).
    // Also records which original seam edges are U-seams vs V-seams so pass 2 can pick the
    // correct shifted endpoint for closed-loop seam originals.
    std::unordered_map<std::string, const Geometry2D::ICorner2D*> uShiftedCorner;
    std::unordered_map<std::string, const Geometry2D::ICorner2D*> vShiftedCorner;
    std::unordered_map<std::string, Topology3D::SeamCollection::SeamDirection> originalToSeamDir;

    for (const auto& edgeId : edgeIds)
    {
        if (!seams.isSeamTwin(edgeId))
            continue;

        const auto twinStartArr = seams.getTwinStartUV(edgeId);
        const auto twinEndArr = seams.getTwinEndUV(edgeId);

        const Point2D twinStartUV(twinStartArr[0], twinStartArr[1]);
        const Point2D twinEndUV(twinEndArr[0], twinEndArr[1]);

        const std::string startCornerId = edgeId + "_start";
        const std::string endCornerId = edgeId + "_end";

        if (!geometry2D->getCorner(startCornerId))
            geometry2D->addCorner(
                std::make_unique<Geometry2D::Corner2D>(startCornerId, twinStartUV));
        if (!geometry2D->getCorner(endCornerId))
            geometry2D->addCorner(
                std::make_unique<Geometry2D::Corner2D>(endCornerId, twinEndUV));

        auto* sc2D = geometry2D->getCorner(startCornerId); // twin start = orig end + shift
        auto* ec2D = geometry2D->getCorner(endCornerId);   // twin end   = orig start + shift
        if (sc2D && ec2D)
        {
            // The seam twin topology traverses from seamStart→seamEnd (reverse of the original
            // seam), but 3D parameter normalization assigns t=0 to the original seam start and
            // t=1 to the original seam end. To match those normalized t-values, the LinearEdge2D
            // must go in the original seam direction (ec2D → sc2D), not the twin direction.
            geometry2D->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
                edgeId, ec2D->getPoint(), sc2D->getPoint()));
        }

        // Build the shifted-corner lookups for pass 2, separated by seam direction so that
        // doubly-periodic surfaces (torus) with a single shared vertex can store both the
        // U-shifted and V-shifted copies without the second overwriting the first.
        const Topology3D::SeamCollection::SeamDirection seamDir = seams.getSeamDirection(edgeId);
        const std::string& origEdgeId = seams.getOriginalEdgeId(edgeId);
        const auto& origTopoEdge = fullTopology.getEdge(origEdgeId);
        originalToSeamDir[origEdgeId] = seamDir;

        auto& targetMap = (seamDir == Topology3D::SeamCollection::SeamDirection::U)
                              ? uShiftedCorner
                              : vShiftedCorner;
        targetMap[origTopoEdge.getEndCornerId()] = sc2D;
        targetMap[origTopoEdge.getStartCornerId()] = ec2D;
    }

    // Pass 2: non-seam-twin edges.
    // Closed-circle edges (start == end corner in 3D, e.g. top/bottom caps of a cylinder)
    // close at the seam-shifted virtual corner in UV space rather than the original corner.
    // Without this the LinearEdge2D would be zero-length, making getPoint(t) always return
    // the seam corner and producing invalid split midpoints during refinement.
    //
    // Selection rule for doubly-periodic (torus) seam originals:
    //   U-seam original runs along the V direction → its far end is V-shifted → use vShiftedCorner.
    //   V-seam original runs along the U direction → its far end is U-shifted → use uShiftedCorner.
    // For non-seam closed-circle edges (e.g. cylinder cap), use whichever shifted copy exists.
    for (const auto& edgeId : edgeIds)
    {
        if (seams.isSeamTwin(edgeId))
            continue;

        const auto& topoEdge = fullTopology.getEdge(edgeId);

        auto* startCorner2D = geometry2D->getCorner(topoEdge.getStartCornerId());
        auto* endCorner2D = geometry2D->getCorner(topoEdge.getEndCornerId());

        if (topoEdge.getStartCornerId() == topoEdge.getEndCornerId())
        {
            const Geometry2D::ICorner2D* shiftedCorner = nullptr;
            const auto seamDirIt = originalToSeamDir.find(edgeId);

            if (seamDirIt != originalToSeamDir.end())
            {
                // Seam original: pick the shifted corner from the perpendicular direction.
                const bool isUSeamOriginal =
                    (seamDirIt->second == Topology3D::SeamCollection::SeamDirection::U);
                const auto& srcMap = isUSeamOriginal ? vShiftedCorner : uShiftedCorner;
                auto it = srcMap.find(topoEdge.getEndCornerId());
                if (it != srcMap.end())
                    shiftedCorner = it->second;
            }
            else
            {
                // Non-seam closed-circle (e.g. cylinder rim): use whichever shifted copy exists.
                auto it = uShiftedCorner.find(topoEdge.getEndCornerId());
                if (it != uShiftedCorner.end())
                    shiftedCorner = it->second;
                else
                {
                    auto itV = vShiftedCorner.find(topoEdge.getEndCornerId());
                    if (itV != vShiftedCorner.end())
                        shiftedCorner = itV->second;
                }
            }

            if (shiftedCorner != nullptr)
            {
                endCorner2D = shiftedCorner;
            }
            else
            {
                // Closed circle on a non-periodic face: LinearEdge2D would be zero-length,
                // making getPoint(t) always return the corner UV. Use a surface-projected
                // edge that evaluates the 3D arc and projects to UV.
                const auto* edge3D = fullGeometry.getEdge(edgeId);
                if (edge3D != nullptr && startCorner2D != nullptr)
                {
                    geometry2D->addEdge(std::make_unique<Geometry2D::SurfaceProjectedEdge2D>(
                        edgeId, surface, *edge3D));
                    continue;
                }
            }
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

void MeshingContext2D::setPeriodicConfig(const PeriodicDomainConfig& config)
{
    periodicConfig_ = config;
    // Reset lazy-initialized objects so they are rebuilt with the new config.
    meshOperations_.reset();
    periodicData_.reset();
}

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
    if (!periodicData_ && periodicConfig_)
    {
        periodicData_ = std::make_unique<PeriodicMeshData2D>(*meshData_, *periodicConfig_);
    }
    if (!meshMutator_)
    {
        meshMutator_ = std::make_unique<MeshMutator2D>(*meshData_);
    }
    if (!meshOperations_)
    {
        meshOperations_ = std::make_unique<MeshOperations2D>(*meshData_, periodicData_.get());
    }
}

} // namespace Meshing
