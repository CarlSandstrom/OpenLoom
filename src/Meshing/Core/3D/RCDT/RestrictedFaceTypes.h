#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace Meshing
{

/// The restricted-face set: every face RCDT accepted as boundary, against the
/// CAD surface it was classified to. This map IS the surface mesh -- see
/// RCDTMeshExtractor, which copies it out unchanged.
using RestrictedFaceMap = std::unordered_map<FaceKey, std::string, FaceKeyHash>;

struct BadRestrictedTriangle
{
    FaceKey face;
    std::string surfaceId;
    Point3D circumcircleCenter;
    double shortestEdge;
};

/// The subset of RestrictedFaceMap currently failing the quality criteria,
/// maintained incrementally as faces are (re)classified rather than rescanned.
using BadRestrictedFaceMap = std::unordered_map<FaceKey, BadRestrictedTriangle, FaceKeyHash>;

/// CAD curve id -> the surfaces that curve bounds, read from the topology once
/// in RestrictedTriangulation::buildFrom(). A seam curve appears twice against
/// its own surface, which is load-bearing -- see
/// RestrictedFaceAudit::findNonManifoldEdges().
using EdgeToAdjacentSurfacesMap = std::unordered_map<std::string, std::vector<std::string>>;

/// What a restriction oracle concluded about one face.
///
/// The three states exist because "no surface" and "no answer" are different
/// outcomes with opposite consequences, and a single optional<surfaceId>
/// spelled them the same way: the first is correct, the second is a hole in
/// the surface mesh. Telling them apart is what lets a classification failure
/// be counted rather than silently becoming a missing face.
enum class FaceRestriction
{
    /// No surface is shared by all three nodes. The face is interior to the
    /// tetrahedralization and nothing is missing.
    NotRestricted,

    /// On the model boundary, and the surface it belongs to is known.
    Restricted,

    /// Candidate surfaces existed, but none could be confirmed. This is where
    /// the residual holes come from -- a face that plausibly belongs on the
    /// boundary and was left out of the set anyway.
    ///
    /// Note this is an UPPER BOUND on the real defects, not a count of them:
    /// plenty of genuinely interior faces have three nodes sharing a surface
    /// (see RestrictedFaceAudit's same-curve chord faces for one whole family
    /// of them), so a face landing here is suspicious, not condemned.
    Unconfirmed
};

struct FaceClassification
{
    FaceRestriction restriction = FaceRestriction::NotRestricted;

    /// Set only when restriction is Restricted; empty otherwise.
    std::string surfaceId;
};

/// How an edge's restricted-face coverage departs from what the CAD
/// topology calls for -- see RestrictedFaceAudit::findNonManifoldEdges() for
/// the invariant itself.
enum class RestrictedEdgeDefect
{
    /// Fewer incident faces than expected: a hole in the surface.
    MissingFace,
    /// More incident faces than expected: the same piece of surface covered
    /// twice, the over-acceptance flap of OPE-184.
    ExcessFace,
    /// The expected NUMBER of faces, but restricted to the wrong surfaces --
    /// e.g. two faces on an edge that lies in a surface's interior but that
    /// disagree about which surface, or a curve whose two incident faces
    /// both claim the same one of its two adjacent surfaces.
    SurfaceMismatch
};

/// An edge whose incident restricted faces do not match what the CAD
/// topology calls for. surfaceId is where a repair point should be projected:
/// the surface the edge is short of a face on when one is missing, otherwise
/// the surface carrying the excess.
struct NonManifoldRestrictedEdge
{
    EdgeKey edge;
    std::string surfaceId;
    RestrictedEdgeDefect defect = RestrictedEdgeDefect::MissingFace;
};

/// What RestrictedFaceAudit::removeDefectiveFaces() removed, and the
/// non-manifold edges it could not resolve, counted per RestrictedEdgeDefect.
struct DefectiveFaceRemovalSummary
{
    size_t chordFacesRemoved = 0;
    size_t excessFacesRemoved = 0;
    size_t remainingMissingFaceEdges = 0;
    size_t remainingExcessFaceEdges = 0;
    size_t remainingSurfaceMismatchEdges = 0;
};

} // namespace Meshing
