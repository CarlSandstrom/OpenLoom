# Cap protecting-ball radii by local feature size

Bound every protecting ball in `ProtectingBallPlacer` by the local feature size of the geometry, so a curve lying close to a surface it does not bound is protected by balls that stay clear of that surface. Fixes the non-flat facets on `ThinCapBoxSurface` and `CylinderWithThinBottom.step`; the first step toward sizing the balls from `h(x)` (the idea noted as OPE-212).

## 1. Context

1. A curve closer to a non-incident surface than its ball radius makes the ball reach through that surface. Facets touching the ball are exempt from the shape and distance criteria (`ProtectionExemption`), so refinement accepts tilted facets joining the two surfaces.
2. Measured on `ThinCapBoxSurface` (10x10x10 box, 6x6x9 pocket, 1 mm cap): 88 of 1326 triangles join the pocket ceiling (z=9) to the top face (z=10). On the cup STEP: 411 of 6654.
3. The placer sizes balls only from the existing discretization spacing, and only shrinks balls that hit other *balls*. Nothing bounds a ball by non-incident surfaces.

## 2. Goal

1. No protecting ball's radius exceeds the local-feature-size cap at its centre, so a ball does not reach a non-incident surface (cap = feature size / 2.75).
2. Curves near thin features are subdivided locally by the existing repopulate step; elsewhere the discretization is unchanged.
3. `ThinCapBoxSurface` meshes with zero triangles joining z=9 and z=10.

## 3. Non-goals

1. Curvature-based sizing, or sizing the balls from the full `h(x)` (rest of OPE-212).
2. Same-surface folds. `LocalFeatureSize3D` documents that it does not detect them.
3. Speeding up `LocalFeatureSize3D` (quadratic in sample count).
4. Changing the facet criteria, `ProtectionExemption`, the repair algorithm, or the boundary discretization.
5. Slivers (OPE-211).

## 4. Current state

1. `src/Meshing/Core/3D/RCDT/ProtectingBallPlacer.cpp`, `ProtectingBallPlacer::place(discretization, topology, geometry)`, single caller `RCDTMesher.cpp` (`buildAmbientTriangulation`). Flow: `placeCorners` -> `insertBalls` per curve (-> `spacedBalls`) -> `repairBalls` (`shrinkIntersectingBalls` + `repopulate`, at most 29 rounds) -> `rebuild`.
2. Size function today: `sizeAt(curve, arc)` reads the discretization spacing and is used only for the midpoint test in `insertBalls`. Corner radius is `min(max(cornerSize, tangentCornerFloor), nearestCorner / 3)` in `placeCorners`. Interior radii are interpolated between end radii in `spacedBalls`.
3. `src/Meshing/Core/3D/General/`: `LocalFeatureSize3D::compute(samples, topology)`, `SizingFieldBuilder3D::build(geometry, topology, SizingFieldSettings3D)` (= `min(curvature size, featureSize / elementsAcrossFeatureGap)`, floor `minimumSizeFraction * diagonal`), `SizingField3D::evaluate(point)` (Lipschitz envelope). Tested, no RCDT consumer.
4. Tests: `tests/Meshing/Core/test_ProtectingBallPlacer.cpp` (helper `protect`, `expectCgalProtectionConditions`), `test_RCDTMesher.cpp`, `test_SizingFieldBuilder3D.cpp`.
5. Prior failure to respect: OPE-181 stage 2 replaced the old protection wholesale (1939 s / 697 non-manifold vs 511 s / 17). Feed one input at a time. Ball radius stays near 0.5 of local spacing; this change only lowers radii.

## 5. Design

1. `SizingFieldSettings3D` gains `bool includeCurvature = true`. When false, `SizingFieldBuilder3D::build` ignores the curvature term, so the field is feature size / `elementsAcrossFeatureGap` only. Default behaviour is unchanged.
2. `ProtectingBallPlacer::place` builds `SizingFieldBuilder3D::build(geometry, topology, settings)` with default settings, `includeCurvature = false`, `elementsAcrossFeatureGap = 2.75` (this is the divisor; 2.0 left 278 triangles across the cap on the cup STEP because the sampled feature size overstates the gap, see OPE-205). `place`'s signature does not change. `cap(point) = field.evaluate(point)`.
3. The cap is applied in three places, all inside the placer:
   1. corner radius: `min(existing radius, cap(corner point))`;
   2. `sizeAt`: `min(spacing, cap(point at arc))`;
   3. every interior ball from `spacedBalls`: radius clamped to `cap(ball point)`. The interpolation between end radii would otherwise skip a thin region mid-curve. Gaps this opens are filled by `repopulate` in `repairBalls`.
4. Invariants: every final ball radius is at most `cap(centre)`; the three CGAL conditions (`expectCgalProtectionConditions`) still hold; where the cap exceeds the current radius nothing changes.
5. No cap where the feature size is infinite or no sample exists: the builder already emits one diagonal-sized source, so the cap is large and inert. No new throw path.

## 6. Acceptance criteria

1. `cmake --build build -j$(nproc)` is warning-free in touched files.
2. `ctest --test-dir build/tests -j$(nproc) -LE slow --output-on-failure`: all pass.
3. New test `ProtectingBallPlacerTest.BallsStayClearOfNonIncidentSurfaces` (thin-cap box, same pocket as `ThinCapBoxSurface.cc`): every weighted point whose centre lies at z=9 has radius below 1.0, and `expectCgalProtectionConditions` holds. Fails before step 3 of section 9, passes after.
4. New test `RCDTMesherTest.ThinCapBoxHasNoFacetsAcrossTheCap`: `RCDTMesher::meshSurface` on that box yields 0 triangles with vertices on both z=9 and z=10 (baseline 88).
5. `./build/src/Examples/3D/Surface/ThinCapBoxSurface` exits 0. Same check on `CylinderWithThinBottom.step` via `MeshStepFile3DSurface`: 0 triangles join z=49 and z=50 (baseline 411), as a manual run.
6. `TIER=full ./scripts/refactor-check.sh`: either no golden moves, or the run output names each moved model and you approve each one. `--accept` only after that approval (goldens may move; decision is yours).
7. `./scripts/check-docs.sh` passes.
8. `time ./build/src/Examples/3D/Surface/SaddleSurfaceMesh`: measured 51 s against the 22.4 s baseline (203 against 94 curve points). The 1.10x bound is waived and accepted as a known cost until OPE-205.
9. `ctest --test-dir build/tests -j$(nproc) --output-on-failure`: all pass.

## 7. Assumptions

1. Cap source is feature size only, divisor 2.75, default samples (32 per curve, 8 per surface direction), `gradientLimit` default 0.3 (your answers; the gradient limit is mine).
2. Applying the cap to interpolated interior radii (5.3.3) is part of "into the size function". Not discussed; needed because `sizeAt` is read only at midpoints.
3. The cap wins over `tangentCornerFloor` (OPE-218) at a corner, as the nearest-corner/3 cap already does. Risk: the floor exists so repair converges. If the two conflict and repair stops converging, stop and report (open question 1 below, decided).
4. The `includeCurvature` flag is the way to get a feature-size-only field. Alternatives (a second builder method) are equivalent.
5. `ThinCapBoxSurface` (already added, uncommitted) is the reproduction and the test geometry.
6. 1 mm is the cap thickness the test uses; no thinner geometry is covered.
7. `LocalFeatureSize3D` finds the curve-versus-top-plane pair constraining. Read from its doc ("share nothing: the rule always fires"), not yet run.
8. The 10% bound on `SaddleSurfaceMesh` is yours; the baseline is measured in step 1, not recalled.
9. Per `CLAUDE.md`, no commits and no branch unless you ask.
10. OPE-212 could not be found in Plane, so this work gets a new ticket (OPE-204). The "first step of OPE-212" wording in the summary refers to the idea in the notes, not to a findable ticket.
11. Timing on `PartDesignExample-Body.step` is not required; the earlier 79 min is put down to debug logging being on (your decision, unmeasured).

## 8. Open questions

1. Reminder, for when it arises: if the cap and `tangentCornerFloor` conflict and repair stops converging, the implementer stops and reports the model and the round count; the resolution (raise the cap there, or another approach) is decided then.

## 9. Implementation steps

1. Baselines, no code: `TIER=full ./scripts/refactor-check.sh` clean, `time` on `SaddleSurfaceMesh`, 88 and 411 spanning-triangle counts recorded. Check: numbers written down.
2. Add `includeCurvature` to `SizingFieldSettings3D` and the builder, plus a test in `test_SizingFieldBuilder3D.cpp`. Check: fast tests green, `refactor-check.sh` unchanged.
3. Add the tests of 6.3 and 6.4 (expected red), then cap corners, `sizeAt` and interior radii in `ProtectingBallPlacer`. Check: 6.2 to 6.4 green; report golden movement per 6.6 before any `--accept`.
4. Docs: add the cap to the Protecting ball and R1.2 sections of `doc/RCDT_Current.md` and the matching entry in `doc/RCDT_Techniques.md`. Check: `./scripts/check-docs.sh`.
5. Add the `Export::TsvExporter::writeSurfaceMesh` call to `ThinCapBoxSurface.cc`, add `3D/Surface/ThinCapBoxSurface` to `FAST_EXAMPLES` in `scripts/refactor-check.sh`, and bless its golden after the fix (`--accept` for the new entry only). Check: `TIER=full ./scripts/refactor-check.sh`.
6. Final: 6.8 timing, then 6.9 and the full tier.
