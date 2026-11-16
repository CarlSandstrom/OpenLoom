# Cap protecting-ball radii by searched distance to non-incident surfaces (OPE-205)

Replace the sampled local-feature-size cap in `ProtectingBallPlacer` (OPE-204) with the distance from each ball centre to the nearest surface that shares no corner with the ball, found by a coarse-then-refine search. The divisor returns to 2.0 and the quadratic field build goes away.

## 1. Context

1. OPE-204 caps ball radii with `LocalFeatureSize3D`, which measures sample to sample. The sampling overstates gaps, so the divisor had to be 2.75; at 2.0 the cup (`CylinderWithThinBottom.step`) keeps 278 triangles joining z=49 and z=50.
2. The field build is quadratic in the sample count (0.14 s, 0.63 s, 6.7 s per grid doubling) and does not scale to real STEP parts.
3. OPE-204 took `SaddleSurfaceMesh` from 94 to 203 curve points (22.4 s to 51 s) and moved four goldens that have no thin feature.

## 2. Goal

1. Every ball radius is at most `max(d / 2.0, floor)`, where `d` is the distance from its centre to the nearest constraining surface (5.1) and `floor` is 1e-3 of the model's bounding-box diagonal.
2. `ThinCapBoxSurface` and the cup mesh with 0 triangles across the thin wall at divisor 2.0.
3. The placer no longer uses `SizingFieldBuilder3D`; `includeCurvature` is gone.

## 3. Non-goals

1. Sharp wedges: a surface that shares a corner with the curve is never checked, however close it runs.
2. Same-surface folds, curvature sizing, slivers.
3. Deleting or changing `SizingFieldBuilder3D`, `SizingField3D`, `LocalFeatureSize3D` beyond removing `includeCurvature`.
4. New methods on `ISurface3D` or `IEdge3D`. No `BRepExtrema`.
5. An automated test for the cup; it stays a manual run.
6. The facet criteria, `ProtectionExemption`, the repair algorithm, the boundary discretization.

## 4. Current state

1. `src/Meshing/Core/3D/RCDT/ProtectingBallPlacer.cpp`: `place()` builds `SizingFieldBuilder3D::build(...)` with `includeCurvature = false`, `elementsAcrossFeatureGap = 2.75`, and reads it as `featureCap->evaluate(point)` in exactly three places: `sizeAt`, `spacedBalls`, `placeCorners`. `Curve` holds `const SizingField3D* featureCap`.
2. `includeCurvature` (`SizingFieldBuilder3D.h/.cpp`) has two users: the placer and one test in `tests/Meshing/Core/test_SizingFieldBuilder3D.cpp`.
3. Topology: `Topology3D::getAllSurfaceIds()`, `Surface3D::getCornerIds()`, `Surface3D::getBoundaryEdgeIds()`, `Edge3D::getStartCornerId()/getEndCornerId()`, `SeamCollection::isSeamTwin()`.
4. Geometry: `ISurface3D::getPoint`, `getParameterBounds` (the face's own UV rectangle in `OpenCascadeSurface`), `isUVWithinTrimmedBoundary` (cached classifier); `IEdge3D::getPoint`, `getParameterBounds`, `isDegenerate`. There is no closest-point query on the trimmed face: `projectPoint` and `getGap` use the untrimmed surface.
5. Tests that pin the behaviour already exist: `ProtectingBallPlacerTest.BallsStayClearOfNonIncidentSurfaces`, `RCDTMesherThinCapBoxTest.HasNoFacetsAcrossTheCap`, and `expectCgalProtectionConditions` in `test_ProtectingBallPlacer.cpp`.
6. Goldens: `ThinCapBoxSurface` is in `FAST_EXAMPLES`. `git show 74839fc^:tests/golden/<model>/<file>` is the pre-cap golden of the four models OPE-204 moved.
7. Docs naming the 2.75 cap: `ProtectingBallPlacer.h` class comment, `doc/RCDT_Current.md` line 69, `doc/RCDT_Techniques.md` "Feature-size cap".

## 5. Design

1. **Constraining surfaces.** A surface constrains a ball iff its `getCornerIds()` contains none of the ball's corners: the curve's start and end corner for a curve ball, the corner itself for a corner ball. This also excludes the surfaces the curve bounds. Reason: a surface through the ball's corner is at distance zero there, and a plain distance would shrink balls into every corner of every model.
2. **New class** `Meshing::NonIncidentSurfaceDistance` in `src/Meshing/Core/3D/General/` (`.h`, `.cpp`, added to that folder's `CMakeLists.txt`):
   ```cpp
   NonIncidentSurfaceDistance(const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology);
   /// Distance from point to the nearest surface containing none of cornerIds;
   /// infinity when no surface qualifies.
   double distance(const Point3D& point, const std::vector<std::string>& cornerIds) const;
   ```
3. **Cached once in the constructor:** per edge (not degenerate, not a seam twin) 9 points at 8 equal parameter intervals; per surface an 8x8 grid of cell-centre UV points, keeping only those with `isUVWithinTrimmedBoundary`. Each sample stores its cell radius: the largest 3D distance to an adjacent sample.
4. **`distance()`**, `best` starting at infinity:
   1. Edges first. Over the boundary edges of the constraining surfaces (each edge once), take every cached sample as a candidate. Visit candidates by increasing coarse distance; skip one when `coarse distance - cell radius >= best`. Refine a visited candidate in 1D: halve the parameter interval around the current nearest point, re-evaluate, repeat. Lower `best`.
   2. Interiors. Same visiting rule over the cached in-trim surface samples. Refine in 2D: evaluate the 3x3 points of the 2x2 cell neighbourhood, move to the nearest in-trim one, halve the cell, repeat. Only in-trim points may lower `best`. UV is clamped to the parameter bounds, no periodic wrap.
   3. Refinement stops when the cell's 3D size is below 1% of `best`, or after 12 halvings.
5. **Invariant:** every point that lowers `best` lies on an edge or is an in-trim surface point, so the result is never below the true distance. A missed basin can only make the cap too large.
6. **Placer.** `place()` builds one `NonIncidentSurfaceDistance`. `Curve` drops `featureCap` and gains the object pointer, its two corner ids and the floor. The three reads become `max(distance(point, corners) / 2.0, floor)`; `placeCorners` passes the single corner id. `floor = 1e-3 * diagonal` of the bounding box of `discretization.points`. When the floor wins, `spdlog::warn` names the edge or corner. `place()`'s signature does not change.
7. **Diagnostic.** The existing `spdlog::info` summary line gains the number of final balls (corners and interior) whose radius is within 1e-9 relative of their cap.

## 6. Acceptance criteria

1. `cmake --build build -j$(nproc)`: no warnings in touched files.
2. New `tests/Meshing/Core/test_NonIncidentSurfaceDistance.cpp`, all on OCC shapes through `Readers::TopoDS_ShapeConverter`:
   1. thin-cap box (as in `ThinCapBoxSurface.cc`), point (5, 2, 9) with the corners of that ceiling edge: 1.0 within 1%;
   2. same box, point (5, 0, 10) with the corners of that top edge: `sqrt(5)` within 1% (nearest point is the ceiling's rim, found by the edge stage);
   3. plain 10x10x10 box, midpoint of an edge with that edge's corners: 10.0 within 1% (corner-sharing faces excluded);
   4. cylinder radius 5, height 10, point on the top circle with that circle's corner: 10.0 within 1% (seam, closed edge).
3. `ctest --test-dir build/tests -j$(nproc) -LE slow --output-on-failure`: all pass, including the two tests of 4.5 with the divisor at 2.0.
4. `./build/src/Examples/3D/Surface/MeshStepFile3DSurface src/Examples/3D/STEP/CylinderWithThinBottom.step`: exits 0, and 0 triangles have vertices on both z=49 and z=50 (manual count).
5. `grep -rn "includeCurvature\|SizingField" src/Meshing/Core/3D/RCDT/ProtectingBallPlacer.cpp src/Meshing/Core/3D/RCDT/ProtectingBallPlacer.h`: no output. `grep -rn includeCurvature src tests`: no output.
6. `TIER=full ./scripts/refactor-check.sh`: for every model it reports moved, either the summary line's capped-ball count is above 0, or its output is identical to `git show 74839fc^:tests/golden/...`. Report each moved model with its count; `--accept` only after approval.
7. `time ./build/src/Examples/3D/Surface/SaddleSurfaceMesh`: at most 1.10x the time measured in step 1. If exceeded, stop and report the curve-point count and which edges the cap lowered.
8. `./scripts/check-docs.sh` and `./scripts/check-layers.sh` pass.
9. `ctest --test-dir build/tests -j$(nproc) --output-on-failure`: all pass.

## 7. Assumptions

1. The class lives in `General/`, not beside the placer as the ticket suggests: it is a pure geometry query. The ticket's name `NonIncidentSurfaceDistance` is kept although the rule is "shares no corner".
2. Coarse resolution is 8x8 per surface and 8 intervals per edge. You said "8*8 or even 4*4"; 8 matches `samplesPerSurfaceDirection` today.
3. The pruning rule (5.4.1) replaces the "best two or three candidates" we discussed. Reason: cost. It treats the cell radius as a bound on how far the true nearest point can be from the sample, which is approximate on strongly curved cells.
4. 1% convergence and 12 halvings are my numbers.
5. No periodic wrap; the seam edge is a boundary edge of its surface, so the edge stage covers the seam line.
6. The floor's diagonal is taken over `discretization.points`. OPE-204 took it over the sizing field's samples; the two differ slightly.
7. "Balls the cap lowered" is counted as final balls sitting at their cap (5.7), which costs one extra distance query per ball.
8. A surface with no in-trim coarse sample is still covered by its boundary edges; no warning.
9. Surfaces or edges missing from `GeometryCollection3D` (null from `getSurface`/`getEdge`) are skipped silently, as the placer does today.
10. The unit tests use OCC shapes; no mock `ISurface3D` is added.
11. The Saddle prediction (cap bites only on the two bottom edges under the low sides, about 0.5, near what the ball-ball shrink already gives) is a hand calculation, not a run.
12. Goldens did not move between `74839fc` and `HEAD`, so `74839fc^` is a valid pre-cap reference.
13. Divisor 2.0 is `constexpr` in the placer, not a setting.
14. Per `CLAUDE.md`: no commit and no branch unless you ask; a checkpoint is offered at each green step.

## 8. Open questions

1. The pruning rule can skip a candidate whose cell is strongly curved and so miss the true nearest point. Accept, or inflate the cell radius by a safety factor? Recommendation: accept, because the result errs toward a larger cap only, and acceptance 6.3 and 6.4 would expose a miss on the target models.
2. If the cup needs a finer coarse grid than 8x8 to reach 0 at divisor 2.0, is raising the grid allowed inside this change? Recommendation: yes, up to 16x16 and reported, because the grid is linear in cost here; anything beyond that falls under your decision 1a (stop and report).
3. Should the capped-ball count stay at `info`, or drop to `debug` once the goldens are approved? Recommendation: keep `info`, because it is one line per run and it is the check 6.6 depends on for future golden moves.

## 9. Implementation steps

1. Baselines, no code. Record: `TIER=full ./scripts/refactor-check.sh` clean; `time` of `SaddleSurfaceMesh` built at `74839fc^` (a `git worktree`, same build type) and at `HEAD`; curve-point counts from the placer's log line. Check: numbers written in the report.
2. Add `NonIncidentSurfaceDistance` with the tests of 6.2. No caller yet. Check: 6.2 green, fast tests green, `refactor-check.sh` unchanged, `check-layers.sh` passes.
3. Swap the placer to the new class at divisor 2.0 with the floor, the warning and the count (5.6, 5.7). Check: 6.3 and 6.4. If either thin-wall count is not 0, stop and report the count, the model and the smallest divisor that reaches 0 (decision 1a). Then 6.6 and 6.7; report before any `--accept`.
4. Remove `includeCurvature` from `SizingFieldSettings3D`, the builder and its test. Check: 6.5, fast tests green, `refactor-check.sh` unchanged against step 3.
5. Docs: the `ProtectingBallPlacer.h` class comment, `doc/RCDT_Current.md` line 69, the "Feature-size cap" entry in `doc/RCDT_Techniques.md`; mark OPE-205 done in `doc/specs/thin-feature-ball-cap.md` only if you want it touched. Check: 6.8.
6. Final: 6.9 and `TIER=full ./scripts/refactor-check.sh`.
