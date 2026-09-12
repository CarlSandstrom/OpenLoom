#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/DualEdgeRestrictionOracle.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Core/3D/RCDT/SurfaceCandidates.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Meshing
{
class CurveSegmentManager;
class MeshConnectivity;
class MeshData3D;
class TetrahedralElement;
} // namespace Meshing

namespace Geometry3D
{
class GeometryCollection3D;
class ISurface3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

class RestrictedTriangulation
{
public:
    /// Full initial scan. Builds internal topology lookup tables, then classifies
    /// every tetrahedral face as restricted or not. minimumEdgeLength is used
    /// to size each surface's tessellation oracle so its cells are fine enough
    /// to correctly classify faces down to that scale — built once upfront,
    /// never rebuilt during refinement. settings is stored for the lifetime of
    /// this object (assumed constant across a refinement run) and used to
    /// evaluate each face's quality as it's (re)classified, keeping the
    /// bad-triangle set (see getBadTriangles()) incrementally maintained
    /// instead of rescanned from scratch on every query.
    void buildFrom(const MeshData3D& meshData,
                   const MeshConnectivity& connectivity,
                   const Geometry3D::GeometryCollection3D& geometry,
                   const Topology3D::Topology3D& topology,
                   double minimumEdgeLength,
                   const SurfaceMesh3DQualitySettings& settings);

    /// Incremental update after a Bowyer-Watson insertion.
    /// Removes cavity-interior faces that no longer exist, then re-classifies
    /// all faces of the new tetrahedra adjacent to newNodeId.
    void updateAfterInsertion(const std::vector<FaceKey>& cavityInteriorFaceKeys,
                              size_t newNodeId,
                              const MeshData3D& meshData,
                              const MeshConnectivity& connectivity,
                              const Geometry3D::GeometryCollection3D& geometry);

    /// Removes any restricted face whose node set contains both nodeId1 and nodeId2.
    /// Call this after splitting the curve segment between those two nodes, so that
    /// faces spanning the now-subdivided edge are not left as stale entries.
    void invalidateFacesWithEdge(size_t nodeId1, size_t nodeId2);

    /// Restricted faces that violate quality criteria, maintained incrementally
    /// (see buildFrom()/updateAfterInsertion()/invalidateFacesWithEdge()) rather
    /// than rescanned here: each face's quality is evaluated once, when it's
    /// (re)classified, not on every call to this method.
    std::vector<BadRestrictedTriangle> getBadTriangles() const;

    const RestrictedFaceMap& getRestrictedFaces() const;

    /// Post-hoc cleanup over the restricted-face set, meant to be called ONCE
    /// after refinement has fully converged -- see
    /// RestrictedFaceAudit::removeDefectiveFaces(), which this hands the face
    /// set and the topology lookup to.
    DefectiveFaceRemovalSummary removeDefectiveFaces(const MeshData3D& meshData);

    /// The insertion point for the given bad triangle: where its dual Voronoi
    /// edge (the segment between its two adjacent tets' circumcenters) crosses
    /// the surface. Computed on demand rather than in getBadTriangles() to
    /// avoid paying 30 bisection iterations × 3 OCC calls for every bad face
    /// when only one will actually be inserted this step.
    /// Returns nullopt if the endpoints cannot be computed or the edge does not
    /// cross the surface.
    std::optional<Point3D> computeInsertionPoint(const FaceKey& face,
                                                 const MeshData3D& meshData,
                                                 const MeshConnectivity& connectivity,
                                                 const Geometry3D::ISurface3D& surface) const;

    /// Every edge whose incident restricted faces do not match what the CAD
    /// topology calls for there -- see RestrictedFaceAudit::findNonManifoldEdges()
    /// for the invariant itself and why it is not the flat "exactly 2 faces"
    /// test it replaces. An empty vector means the set satisfies it everywhere,
    /// which is what AmbientTetrahedronClassifier's flood fill requires.
    std::vector<NonManifoldRestrictedEdge> findNonManifoldEdges(const MeshData3D& meshData) const;

private:
    /// (Re)evaluates face's quality against settings_ and updates badFaces_
    /// accordingly -- inserted/refreshed if it fails, erased if it now
    /// passes. Called once per (re)classified face, immediately after the
    /// oracle confirms it's restricted to surfaceId, so badFaces_ stays
    /// current without ever needing a full rescan (see getBadTriangles()).
    void updateBadFaceEntry(const FaceKey& face,
                            const std::string& surfaceId,
                            const MeshData3D& meshData,
                            const Geometry3D::GeometryCollection3D& geometry);

    RestrictedFaceMap restrictedFaces_;
    BadRestrictedFaceMap badFaces_;
    SurfaceMesh3DQualitySettings settings_;

    /// Built from topology in buildFrom(). Shared: the oracle resolves a
    /// face's candidate surfaces through it, RestrictedFaceAudit reads the
    /// curve-to-surface lookup off it. Declared before oracle_, which holds a
    /// pointer to it.
    SurfaceCandidates surfaceCandidates_;

    /// What decides restriction. Held by value and used only through
    /// classify() and insertionPointFor(), so OPE-186's replacement is this
    /// declaration plus the construction in buildFrom().
    DualEdgeRestrictionOracle oracle_;
};

} // namespace Meshing
