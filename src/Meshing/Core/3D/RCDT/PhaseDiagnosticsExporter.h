#pragma once

#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"

#include <string>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
class MeshConnectivity;
class MeshData3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Writes the centroid-phase field and the restriction disagreement to
 * VTU, for inspection in ParaView.
 *
 * Exists to make one question visible: the phase test
 * (DualEdgeRestrictionOracle::isPhaseBoundaryFace) decides restriction from
 * whether the two tetrahedra sharing a face lie on different sides of the
 * model, and it is the only shape-independent signal in the classifier -- a
 * centroid is a convex combination of its tetrahedron's own vertices, so
 * unlike a circumcenter it cannot land far from the element it represents.
 * But it declines whenever either centroid is Ambiguous, which is OPE-187's
 * recorded root cause. Whether that band is thin isolated pockets or a
 * connected sheet decides whether a region-labelling design can work at all,
 * and nobody has looked at it.
 *
 * Writes two files:
 *
 *  - `<prefix>_phase_tets.vtu` -- the ambient tetrahedralization, each
 *    element carrying its centroid's phase and that centroid's distance to
 *    the nearest surface, each node carrying its own distance to the nearest
 *    surface.
 *  - `<prefix>_phase_faces.vtu` -- every face that EITHER the live classifier
 *    or the centroid rule alone considers restricted, carrying which of the
 *    two accepted it and the phases of both adjacent tetrahedra.
 *
 * Deliberately writes its own VTU rather than extending VtkExporter: the
 * production exporter's output is what `scripts/refactor-check.sh` diffs
 * against `tests/golden/`, so adding fields there would move all 13 goldens
 * and make every future diagnostic field a behaviour change.
 *
 * Reads nothing private and classifies nothing itself -- phases come from the
 * shared classifyPointPhase(), so this cannot drift from what the mesher
 * actually does. It is therefore unaffected by OPE-186 replacing the oracle,
 * and the disagreement it reports stays meaningful across that change.
 */
class PhaseDiagnosticsExporter
{
public:
    /// restrictedFaces is the live classifier's answer, to be diffed against
    /// the centroid rule. filePrefix names the pair of files written beside
    /// the other debug exports.
    static void write(const MeshData3D& meshData,
                      const MeshConnectivity& connectivity,
                      const Geometry3D::GeometryCollection3D& geometry,
                      const Topology3D::Topology3D& topology,
                      const RestrictedFaceMap& restrictedFaces,
                      const std::string& filePrefix);
};

} // namespace Meshing
