# Minimum ball size in protecting-ball repair (OPE-249)

Port CGAL's `minimal_size` rule into `ProtectingBallPlacer`: no ball is placed below a minimum size, a shrink that would go below it sets the ball to exactly that size, and a ball held there is never shrunk by a non-neighbour again. This stops the repair from running away when two curves touch away from a corner, as they do on `Flanged_Housing.step`.

## 1. Context

1. `MeshStepFile3DSurface Flanged_Housing.step` never gets past `ProtectingBallPlacer::place`. On this model `edge_113` (a semicircle of radius 30 at z = −87) and `edge_14` (on the same cylinder) come within 2.8e-5 of each other and share no corner. The STEP tolerance is 0.26, so in CAD terms they touch.
2. `shrinkIntersectingBalls` shrinks any two overlapping balls that are not neighbours to `distance / 2.1`, with no lower limit. `repopulate` then fills the gaps with smaller balls, and the next round shrinks those too. On these two edges the minimum radius went 0.0087 → 0.00064 → 0.00018 and the ball count went 1039 → 3699 → 9884. In total, 7064 → 27193 balls by round 3, at 15–25 s per round, because the shrink pass compares every pair.

## 2. Goal

1. Every radius the placer produces (corners, `sizeAt`, `spacedBalls`) is at least `minimumSize = MINIMUM_CAP_FRACTION × bounding-box diagonal`, the floor the surface cap already uses. A non-neighbour shrink below it sets the radius to exactly `minimumSize`.
2. A ball held at that limit is marked and is never shrunk again. A conflict involving it alone does not start another round.
3. On `Flanged_Housing.step`, `place()` returns with repair settled before `MAXIMUM_REPAIR_ROUNDS`.

## 3. Non-goals

1. Making the shrink pass faster than O(N²) (separate ticket).
2. (Dropped during implementation: the size function is floored too; see 5.5.)
3. Changing the per-ball `capAt` warning flood (separate ticket).
4. Downstream behaviour on Flanged_Housing (refinement, extraction). It is measured and reported, and any failure there becomes a new ticket.
5. Tolerance-aware handling of touching curves. No CAD tolerance is exposed today.
6. The rule beyond CGAL that shrinks a neighbour's hiding ball to their distance (`shrinkIntersectingBalls`, neighbour branch).

## 4. Current state

1. `src/Meshing/Core/3D/RCDT/ProtectingBallPlacer.cpp`, all in its anonymous namespace:
   1. `Ball {arc, point, radius, cornerId}`
   2. `shrinkIntersectingBalls(balls, neighbours)` returns `bool shrunk`. It sets `shrunk = true` on every non-neighbour conflict, even one that changes nothing.
   3. `repairBalls(curves, ends, corners)` runs shrink, then `repopulate` per curve, at most 29 rounds.
   4. `place()` computes the floor inline as `SurfaceCap{&surfaceDistance, MINIMUM_CAP_FRACTION * boundingBoxDiagonal(...)}` and logs one `spdlog::info` summary line.
2. Corner balls live in a `std::map<std::string, Ball>`. `collectBalls` hands out pointers to them and to curve balls. `repopulate` and `withCorners` copy `Ball` by value.
3. `tests/Meshing/Core/test_ProtectingBallPlacer.cpp` has the `protect(shape, settings)` helper and `expectCgalProtectionConditions`, which asserts that non-neighbour balls are disjoint. No test reads a STEP file. `tests/CMakeLists.txt` marks tests slow through `SLOW_TEST_FILTER`.
4. CGAL reference: `~/lib/cgal/Mesh_3/include/CGAL/Mesh_3/Protect_edges_sizing_field.h`, `refine_balls` (~l.1730–1830). A new size below `minimal_size_` becomes `change_ball_size(v, minimal_weight(), true)`, and only `if(!is_special(v))`.

## 5. Design

1. `Ball` gains `bool atMinimumSize = false`. It is copied with the ball, so it survives `repopulate` and the corner map.
2. `place()` computes `auto minimumSize = MINIMUM_CAP_FRACTION * boundingBoxDiagonal(discretization.points)` once. It passes this value to `SurfaceCap.floor` and to `repairBalls(curves, ends, corners, minimumSize)`, which passes it on to `shrinkIntersectingBalls(balls, neighbours, minimumSize)`.
3. For each ball of a non-neighbour pair, in `shrinkIntersectingBalls`:
   1. Collect each ball's smallest non-neighbour target `distance / 2.1` over the pass (CGAL's `new_sizes`).
   2. At the end of the pass, a ball not yet marked whose target is below `minimumSize` gets exactly `minimumSize` and is marked `atMinimumSize`.
   3. Otherwise an unmarked ball's new radius is `min(newRadius, target)`, as before. A marked ball ignores non-neighbour targets. The neighbour-hidden rule still applies to it.
4. `shrunk` becomes true only when a radius changes. A pair whose balls are both marked changes nothing and does not start a round.
5. As in CGAL, `placeCorners` (`max(min(size, nearest / 3, cap), minimumSize)`), `sizeAt` and `spacedBalls` floor their radii at `minimumSize`. Measured on Flanged_Housing: with shrink-only flooring, a floored 0.62 ball hid a smaller ball on the other curve; and when the neighbour-hidden rule was told to skip marked balls, floored balls hid corners 54, 68 and 85 instead. Both disappear once no ball starts below the floor.
6. Invariants:
   1. Every radius is at least `minimumSize`, except one shrunk by the neighbour-hidden rule.
   2. A ball's `atMinimumSize` flag goes from false to true at most once.
   3. Non-neighbour balls may overlap only where both are marked, or where one is marked and the other has not yet been shrunk (it is shrunk next round).
   4. Behaviour is unchanged on every model where no shrink target falls below `minimumSize`.
7. The `place()` summary line gains a count of the final balls with `atMinimumSize` set.

## 6. Acceptance criteria

1. `cmake --build build -j$(nproc)` builds with no new warnings in the touched files.
2. New slow test `ProtectingBallPlacerTouchingCurvesTest.SettlesAtMinimumSize`. It loads `src/Examples/3D/STEP/Flanged_Housing.step` through `Readers::StepReader3D` with `DiscretizationSettings3D(std::nullopt, pi / 8, 2)` and runs `place()`. It passes when:
   1. `place()` returns within the test's ctest `TIMEOUT`;
   2. the CGAL conditions hold, except that two non-neighbour balls may overlap when both radii are ≤ `minimumSize × (1 + 1e-9)`.
   Run it with `ctest --test-dir build/tests -R ProtectingBallPlacerTouchingCurvesTest --output-on-failure`. Today it times out.
3. `ctest --test-dir build/tests -j$(nproc) -LE slow --output-on-failure`: all tests pass.
4. Release run: `SPDLOG_LEVEL=info time ./build-release/src/Examples/3D/Surface/MeshStepFile3DSurface src/Examples/3D/STEP/Flanged_Housing.step`. The summary line shows fewer than 29 repair rounds and a nonzero minimum-size count. Report the time `place()` took and how the rest of the run ends.
5. `TIER=full ./scripts/refactor-check.sh`: list each model that moved, with its minimum-size count. `--accept` each one only after approval.
6. `./scripts/check-docs.sh` and `./scripts/check-layers.sh` pass.
7. `ctest --test-dir build/tests -j$(nproc) --output-on-failure`: all tests pass.

## 7. Assumptions

1. "Reuse 1e-3 × diag" means the same constant and the same diagonal (over `discretization.points`) as the surface-cap floor. The value is computed once and shared, not recomputed.
2. (Revised during implementation.) First "radii are never raised". Measurement showed that this lets balls hide each other, so the CGAL rule now applies in full, size function included (5.5), at your decision.
3. The field is named `atMinimumSize` rather than CGAL's `special`, because the project's naming rules prefer a descriptive name.
4. The tighter `shrunk` bookkeeping (5.4) changes only the round count, never the output, on models where nothing is floored. Golden check 6.5 confirms this.
5. Termination on Flanged_Housing is expected, not proven. Each ball is marked at most once, but `repopulate` can add fillers that need marking in turn. The 29-round cap stays as the backstop.
6. `Flanged_Housing.step` gets committed with this change. It is the only reproduction, and no synthetic OCC shape is attempted.
7. The test reaches the STEP file through a new compile definition on `runTests` (e.g. `OPENLOOM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"`), and `runTests` links the library that holds `StepReader3D`. Neither exists today.
8. The test name is added to `SLOW_TEST_FILTER`, because it runs a full placement on a real part in the Debug build.
9. `edge_174`/`edge_175` (shrunk to ~0.05 today) will now stop at 0.62. That is intended (your answer: reuse 1e-3 × diag).
10. The neighbour-hidden shrink rule stays unfloored (non-goal 3.6). It is not involved in the runaway that was measured.
11. Docs to update: `ProtectingBallPlacer.h` class comment, `doc/RCDT_Current.md` (repair bullet, ~line 70), `doc/RCDT_Techniques.md` (Repair bullet, ~line 203).
12. Per `CLAUDE.md`: no commits, and a branch only if you ask.

## 8. Open questions

1. Decided: the test asserts the final protection, and the round count is read from the summary line by hand.
2. Decided: the ctest `TIMEOUT` is 3× the measured Debug time (150 s, so 450 s).
3. The test's neighbour-coverage check fails at corner 46 on this model. That is OPE-250 (vertex suspected off the curve end), not this change. Recommendation: leave this test failing until OPE-250 is fixed, rather than exempting the check here, because an exemption would hide OPE-250's regression signal.

## 9. Implementation steps

1. Baselines, no code changes. Run `TIER=full ./scripts/refactor-check.sh` (expect it clean) and record the fast-test count. Check: both results written down.
2. Add `atMinimumSize`, the shared `minimumSize`, the floored shrink (5.3), the strict `shrunk` bookkeeping (5.4) and the summary count (5.7). Check: 6.1, 6.3, and 6.4 on the Release build.
3. Commit-ready test: add the compile definition and link, the STEP file, the test from 6.2 and its slow-filter entry. Measure its Debug time and set `TIMEOUT` (open question 2). Check: 6.2 passes. With step 2 stashed, it times out.
4. Run the golden check (6.5) and report the models that moved. Check: approval before any `--accept`.
5. Update the docs (assumption 11). Check: 6.6.
6. Final checks: 6.7, plus a written report of 6.4 (time and how the run ends). File a ticket for any downstream failure, and one for the `capAt` warning flood.
