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
 * It has two failure modes, and this export exists to show where each one
 * bites. It declines whenever either centroid is Ambiguous, which is
 * OPE-187's recorded root cause. And where a tetrahedron is large relative to
 * the local feature it answers confidently and WRONGLY: a centroid lies
 * inside its own tetrahedron, but a tetrahedron spanning a thin section can
 * have that centroid outside the solid, so both sides of a genuine boundary
 * face read as Exterior. Measured on SaddleSurfaceMesh: 622 declines against
 * 19 confident errors, the latter on tetrahedra 12x the median volume and
 * clustered on the creases where OPE-187's punctures sit.
 *
 * The second mode matters more than its count suggests, because the answer
 * proposed for the first -- propagate a label from confident neighbours --
 * cannot address it: there the neighbours are confident and wrong.
 *
 * Writes two grids, each as `.vtu` for ParaView and as `.nodes.tsv` /
 * `.cells.tsv` tables with one column per field:
 *
 *  - `<prefix>_phase_tets` -- the ambient tetrahedralization, each
 *    element carrying its centroid's phase and that centroid's distance to
 *    the nearest surface, each node carrying its own distance to the nearest
 *    surface.
 *  - `<prefix>_phase_faces` -- every face that EITHER the live classifier
 *    or the centroid rule alone considers restricted, carrying which of the
 *    two accepted it and the phases of both adjacent tetrahedra.
 *
 * Computes the fields here and hands them to VtkExporter::writeGrid and
 * TsvExporter::writeGrid as two caller-assembled grids; the fields depend on RCDT's phase classification,
 * so they cannot live in the Export module itself.
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
