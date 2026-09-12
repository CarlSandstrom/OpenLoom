#pragma once

#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Interfaces/IQualityController3D.h"

#include <cstddef>

namespace Meshing
{

class MeshData3D;

/**
 * @brief Tetrahedron quality criteria for RCDTRefiner's volume-quality
 * refinement priority (priority 3, RCDTMesher::meshVolume() only).
 * RCDTQualityController is the restricted-triangle counterpart.
 *
 * Quality is the circumradius-to-shortest-edge ratio, bounded by
 * SurfaceMesh3DQualitySettings::tetCircumradiusToShortestEdgeRatio. Delaunay
 * refinement's termination guarantee holds only above 2.0, so the constructor
 * warns for bounds at or below it. Slivers -- an acceptable ratio with poor
 * dihedral angles -- are not detected; see TetrahedronQualityRefiner.
 *
 * Only isTetrahedronTooSmall() has a production caller: TetrahedronQualityRefiner
 * uses it as the floor below which a bad tetrahedron is given up on rather than
 * refined. That refiner finds its skinny tetrahedra through
 * MeshQueries3D::findSkinnyTetrahedra, not through isTetrahedronAcceptable(), so
 * the remaining four methods exist to satisfy IQualityController3D and are
 * exercised only by the unit tests. That is why tetElementLimit, which
 * SurfaceMesh3DQualitySettings documents as capping volume refinement, is in
 * fact never enforced.
 */
class RCDTTetQualityController : public IQualityController3D
{
public:
    RCDTTetQualityController(const MeshData3D& meshData, const SurfaceMesh3DQualitySettings& settings);

    bool isMeshAcceptable(const MeshData3D& data, const MeshConnectivity& connectivity) const override;
    bool isTetrahedronAcceptable(const TetrahedralElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;
    bool isTetrahedronTooSmall(const TetrahedralElement& element) const override;

    // Size floor below which isTetrahedronTooSmall() reports a tetrahedron as
    // not worth refining.
    //
    // Provenance, as far as git records it: both values arrived together with
    // isTetrahedronTooSmall in commit 5176a68 (January 2026) on the legacy
    // Shewchuk volume mesher's controller, and were copied here verbatim by
    // OPE-162. No derivation, model or measurement was ever recorded for them,
    // and that class is gone (OPE-161), so there is nothing further to look up.
    //
    // On their own terms: both are absolute rather than relative to model size,
    // so they behave as intended only near the unit scale this project's models
    // use. They are complementary, not redundant -- the volume floor catches
    // flat tetrahedra at any edge length (a regular tetrahedron reaches it at
    // edge ~2e-6), while the edge floor catches needles, whose volume stays
    // around 1e-11 with one edge at 1e-10. MIN_REFINABLE_EDGE is also five
    // orders above ElementQuality3D's internal 1e-15 degeneracy epsilon, so a
    // tetrahedron rejected here is not yet degenerate to that computation.
    static constexpr double MIN_REFINABLE_VOLUME = 1e-18;
    static constexpr double MIN_REFINABLE_EDGE = 1e-10;

private:
    const MeshData3D* meshData_;
    SurfaceMesh3DQualitySettings settings_;
};

} // namespace Meshing
