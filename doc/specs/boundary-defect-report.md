# Report RCDT boundary defects to callers (OPE-15)

Run `RestrictedFaceAudit` on both RCDT paths and give its findings to the caller through `getBoundaryDefects()` on both mesher interfaces and their wrappers. `meshVolume()` still refuses only a boundary with holes, but it now records the report before it throws.

## 1. Context

1. `RCDTMesher::meshSurface()` runs no boundary audit. A punctured or doubled surface mesh comes back exactly like a clean one, with no log line.
2. `RCDTMesher::meshVolume()` audits, keeps only the `MissingFace` count and throws on it. The `ExcessFace` and `SurfaceMismatch` findings are thrown away.
3. The comment on `requireClosedBoundary` (`RCDTMesher.cpp:277`) claims that "a surface mesh with a hole still reports its own defects". No such reporting exists.

## 2. Goal

1. After `meshSurface()` or `meshVolume()` (including one that threw on holes), `getBoundaryDefects()` returns every edge `RestrictedFaceAudit::findNonManifoldEdges` flags, with its kind, surface ID and endpoint coordinates. When a mesh was produced, each entry also has the edge's node IDs in that mesh's numbering.
2. A clean run reports an empty list. A non-empty report also logs one `spdlog::warn` line with the count of each kind.
3. Meshes and goldens are unchanged.

## 3. Non-goals

1. Fixing any defect, or changing restriction, refinement or the audit's invariant.
2. Throwing on `ExcessFace` or `SurfaceMismatch` in either path, or throwing at all in `meshSurface()`.
3. Overlapping or folded triangles (Plane OPE-215). The audit does not detect them.
4. Moving `tests/Meshing/Core/SurfaceMeshTopology` into the library.
5. Printing the report from the examples or the exporters.

## 4. Current state

1. `src/Meshing/Core/3D/RCDT/RCDTMesher.cpp`:
   1. `meshSurface()` runs `buildInitialTriangulation` -> `refineSurface` -> `buildSurfaceMesh`.
   2. `meshVolume()` runs `buildInitialTriangulation` -> `refineVolume` -> `requireClosedBoundary(countMissingFaceEdges(...))` -> `buildSurfaceMesh` -> `extractVolumeMesh` -> `UnreferencedNodeRemover::remove(*volumeMesh_)`.
   3. Both helpers are file-local. Both entry points reset `surfaceMesh_` and `volumeMesh_` first.
2. `buildSurfaceMesh` extracts, smooths (`synchronizeNodePositions` writes the smoothed positions back to the live `MeshData3D`), then calls `UnreferencedNodeRemover::remove(*surfaceMesh_)`.
3. `RestrictedFaceAudit::findNonManifoldEdges(restrictedFaces, buildEdgeToAdjacentSurfaces(topology), meshData)` returns `std::vector<NonManifoldRestrictedEdge>{EdgeKey edge, std::string surfaceId, RestrictedEdgeDefect defect}`. The types are in `RestrictedFaceTypes.h`, and the edges are in working-mesh `NodeId`s.
4. `src/Meshing/Core/3D/General/UnreferencedNodeRemover`: `static void remove(SurfaceMesh3D&)` and `remove(VolumeMesh3D&)`. Each builds `std::vector<NodeId> renumbering` (indexed by old ID, `NodeId::INVALID` for a dropped node) and throws it away. Tests are in `tests/Meshing/Core/test_UnreferencedNodeRemover.cpp`.
5. `ISurfaceMesher3D` / `IVolumeMesher3D` (`src/Meshing/Interfaces/3D/`) declare `meshX()` plus a `const XMesh3D* getXMesh() const`. `SurfaceMesher3D` / `VolumeMesher3D` wrap them with `mesh()` / `getMesh()`.
6. Tests: `RCDTMesherVolumeWithHoleInBoundaryTest.Throws` (`test_RCDTMesherBox.cpp`) leaves holes with `maxRefinementIterations = 0`, `DiscretizationSettings3D(3, 2)` and a box minus a cylinder. `RCDTMesherBoxTest` and `RCDTMesherBoxVolumeTest` are clean fixtures. `test_RestrictedFaceAudit.cpp` builds `MeshData3D` by hand (`SquareSetup`).
7. Docs describing the volume-only check: `doc/RCDT_Techniques.md` ("Closed-boundary check"), `doc/RCDT_Current.md:152,179`, `doc/Meshing_Walkthrough.md:72`, `doc/Meshing_Core_Overview.md:99`.

## 5. Design

1. **Data types**, in `src/Meshing/Data/3D/BoundaryDefectReport3D.h`. They live in `Data/` so the interfaces need nothing from `RCDT/`.
   ```cpp
   enum class BoundaryDefectKind3D { MissingFace, ExcessFace, SurfaceMismatch };
   struct BoundaryDefect3D {
       BoundaryDefectKind3D kind;
       std::string surfaceId;                         // as NonManifoldRestrictedEdge::surfaceId
       std::array<Point3D, 2> endpoints;
       std::optional<std::array<NodeId, 2>> nodeIds;  // output-mesh numbering; set iff a mesh was produced
   };
   struct BoundaryDefectReport3D { std::vector<BoundaryDefect3D> defects; };
   ```
   The kinds map one to one onto `RestrictedEdgeDefect`, so the meaning is unchanged.
2. **Interfaces:** `ISurfaceMesher3D` and `IVolumeMesher3D` each gain `virtual const BoundaryDefectReport3D* getBoundaryDefects() const = 0;`. It returns nullptr until a run has reached the audit. `RCDTMesher` implements it once (one override satisfies both bases). `SurfaceMesher3D::getBoundaryDefects()` and `VolumeMesher3D::getBoundaryDefects()` forward it.
3. **Renumbering:** `UnreferencedNodeRemover::remove(...)` returns the `std::vector<NodeId>` it already builds (old ID -> new ID, `INVALID` = dropped). Callers that don't need it ignore the result.
4. **Builder:** `src/Meshing/Core/3D/RCDT/BoundaryDefectReporter` (a static class, one job):
   1. `audit(restrictedFaces, topology, meshData) -> std::vector<NonManifoldRestrictedEdge>` wraps the audit call.
   2. `toReport(defects, meshData, const std::optional<std::vector<NodeId>>& renumbering) -> BoundaryDefectReport3D` reads the endpoint coordinates from the live `meshData` and maps node IDs through `renumbering` when one is given.
   3. `logSummary(report)` writes the warn line if the report is non-empty.
   4. A defect node that renumbers to `INVALID` is a programming error (every defect edge belongs to an extracted restricted face) and throws `OPENLOOM_THROW_MESH(INVALID_OPERATION, ...)`.
5. **`RCDTMesher` data flow:** add a `std::unique_ptr<BoundaryDefectReport3D> boundaryDefects_` member, reset with the meshes at the start of each call. `buildSurfaceMesh` returns the surface renumbering.
   1. `meshSurface()`: audit after `refineSurface`, then `buildSurfaceMesh`. Store `toReport(defects, meshData, surfaceRenumbering)` (coordinates are post-smoothing), then log.
   2. `meshVolume()`: audit after `refineVolume`. If any entry is `MissingFace`, store `toReport(defects, meshData, nullopt)`, log, and throw. The message is today's, with all three counts. Otherwise build both meshes, then store `toReport(defects, meshData, volumeRenumbering)` and log.
   3. `countMissingFaceEdges` goes away. `requireClosedBoundary` counts `MissingFace` in the audit result.
6. **Criterion comment:** replace the false sentence. State that only `MissingFace` is refused, because a hole lets the `AmbientTetrahedronRemover` flood fill reach the solid's tetrahedra (OPE-185). Excess and mismatched faces still separate inside from outside, so they are reported and not refused.
7. **Invariant:** the restricted-face map, refinement, smoothing and extraction are untouched. The audit is read-only, so every `.tsv` golden is unchanged.

## 6. Acceptance criteria

1. `cmake --build build -j$(nproc)` builds with no new warnings.
2. `./build/tests/runTests --gtest_filter='UnreferencedNodeRemover*'` passes, including a new case asserting the returned map (dropped -> `INVALID`, survivors dense and in order).
3. New `BoundaryDefectReporterTest` (on `SquareSetup`-style data, two triangles sharing one edge, plus a renumbering):
   1. `toReport` gives four `MissingFace` entries with the correct `endpoints`.
   2. `nodeIds` match the renumbering when one is given, and are `nullopt` when it isn't.
   3. A node that renumbers to `INVALID` throws `MeshException`.
4. New `RCDTMesherSurfaceWithHoleInBoundaryTest.ReportsMissingFaces` (same fixture as the volume throw test, `meshSurface()`):
   1. `getBoundaryDefects()` is non-null and has at least one `MissingFace`.
   2. Every entry has `nodeIds` set, in range of `getSurfaceMesh()->nodes`.
   3. Each entry's endpoints equal those nodes' coordinates.
5. `RCDTMesherVolumeWithHoleInBoundaryTest.Throws` additionally asserts, after the throw:
   1. `getBoundaryDefects()` is non-null with at least one `MissingFace` and every `nodeIds == nullopt`.
   2. `getVolumeMesh() == nullptr`.
6. `RCDTMesherBoxTest` and `RCDTMesherBoxVolumeTest` each gain a test that `getBoundaryDefects()->defects` is empty.
7. A `SurfaceMesher3D` and a `VolumeMesher3D` test (the box) asserts that `getBoundaryDefects()` is nullptr before `mesh()` and non-null after.
8. `ctest --test-dir build/tests -j$(nproc) --output-on-failure` passes everything.
9. `TIER=full ./scripts/refactor-check.sh` passes with no golden difference. `./scripts/check-docs.sh` and `./scripts/check-layers.sh` pass.
10. `grep -n "still reports its own defects" src/Meshing/Core/3D/RCDT/RCDTMesher.cpp` returns nothing.

## 7. Assumptions

1. The report lists every flagged edge, without a cap or deduplication by surface. Defect counts on real runs are small (0 on every golden).
2. The coordinates come from the live `MeshData3D` when the report is built: post-smoothing on success, pre-smoothing on a volume throw (no smoothing has run then).
3. After `meshVolume()` succeeds, `nodeIds` refer to the volume mesh's numbering, even though `RCDTMesher::getSurfaceMesh()` is also populated with a different numbering. `VolumeMesher3D` exposes only the volume mesh.
4. `getBoundaryDefects()` returns a raw pointer that is nullptr until run, mirroring `getSurfaceMesh()`/`getVolumeMesh()`. If the pipeline throws before the audit (for example in discretization), it stays nullptr.
5. The new enum mirrors `RestrictedEdgeDefect` rather than reusing it, so `Meshing/Interfaces/` does not depend on `Meshing/Core/3D/RCDT/`. The extra type is accepted for this reason.
6. `BoundaryDefectReporter` lives in `RCDT/` (it reads `RestrictedFaceAudit`) and is added to the Key Modules row for `Meshing/Core/3D/RCDT/` so `check-docs.sh` stays honest. `BoundaryDefectReport3D` is added to the `Meshing/Data/` row.
7. The audit cost is one extra pass in `meshSurface()`, linear in the number of restricted faces. It is not measured and is assumed negligible next to refinement.
8. The surface-with-hole fixture still has `MissingFace` edges after `meshSurface()`, as it does for the volume. This is unverified until the test runs.
9. The warn line format is `"RCDTMesher: boundary defects -- {} missing face, {} excess face, {} surface mismatch"`.
10. `doc/RCDT_Techniques.md`, `RCDT_Current.md`, `Meshing_Walkthrough.md` and `Meshing_Core_Overview.md` are updated in the same change as the behaviour.

## 8. Open questions

1. *Decided:* no helper on `BoundaryDefectReport3D`. Counting is `BoundaryDefectReporter::count(report, kind)`, shared by the log line and the throw message.
2. *Decided:* yes, one line in each wrapper's usage `@code` block.

## 9. Implementation steps

1. `UnreferencedNodeRemover::remove` returns its renumbering; add the map test. Check: build, fast ctest, `./scripts/refactor-check.sh` unchanged.
2. Add `BoundaryDefectReport3D.h` and `BoundaryDefectReporter` (no caller yet) with `BoundaryDefectReporterTest`. Check: acceptance criterion 3 and the fast ctest.
3. Wire `RCDTMesher`: audit both paths, store and log the report, record it before the throw, remove `countMissingFaceEdges`, fix the comment; add `getBoundaryDefects()` to both interfaces and wrappers. Check: acceptance criteria 4-7 and 10, the fast ctest, `refactor-check.sh` unchanged.
4. Update the four docs and the Key Modules table in `CLAUDE.md`. Check: `./scripts/check-docs.sh`, then `TIER=full ./scripts/refactor-check.sh` and the full ctest (criteria 8-9).
