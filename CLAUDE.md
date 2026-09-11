# OpenLoom Project Context

Constrained Delaunay triangulation library for 2D and 3D mesh generation with OpenCASCADE CAD support.

Do not commit unless I ask you to. The one exception is during a multi-step refactor: at each green step, *offer* a commit — still do not make one unsolicited. See Refactoring below.

## Build & Run

```bash
# Configure (first time only)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build everything
cmake --build build -j$(nproc)

# Run tests -- inner loop, 342 of 376 cases in ~3s
ctest --test-dir build/tests -j$(nproc) -LE slow --output-on-failure

# Run everything, including the full-model meshing tests (~85s)
ctest --test-dir build/tests -j$(nproc) --output-on-failure

# Run specific test
ctest --test-dir build/tests -R "TestClass.TestName" --output-on-failure
# or directly:
./build/tests/runTests --gtest_filter="TestClass.TestName"

# Behaviour-preservation check for refactoring: meshes representative models
# and diffs every .vtu against tests/golden/. Any difference means the change
# is not a refactor.
./scripts/refactor-check.sh              # fast tier, ~9s
TIER=full ./scripts/refactor-check.sh    # adds the slow models, ~75s
./scripts/refactor-check.sh --accept     # re-bless after an intended change
```

## Examples

2D executables in `build/src/Examples/2D/`:
- `SimpleDelaunay2D` - Basic 2D Delaunay triangulation
- `RectangleWithHole2D` - 2D mesh with hole constraints
- `RectangleWithCrack` - 2D mesh with crack constraints
- `SquareWithCircularHole` - 2D square with circular hole
- `SquareWithCircleAndTwinEdges`
- `SquareWithInternalCircles`
- `MeshStepFile2D`

3D surface executables in `build/src/Examples/3D/Surface/`:
- `CylinderSurfaceMesh`
- `SurfaceMeshEdges`

3D volume executables in `build/src/Examples/3D/Volume/`:
- `BoxWithHole` - 3D volume mesh with a cylindrical hole
- `CreateBox` - Basic 3D volume mesh

View output with ParaView: `paraview output.vtu`

## Architecture

### Key Modules
| Module | Purpose |
|--------|---------|
| `Common/` | Types, BoundingBox, Exceptions |
| `Geometry/2D/` | 2D geometric entities (ICorner2D, IEdge2D, IFace2D — Base and OpenCascade impls) |
| `Geometry/3D/` | 3D geometric entities (ICorner3D, IEdge3D, ISurface3D — Base and OpenCascade impls) |
| `Topology/` | 3D topological relationships (Corner3D, Edge3D, Surface3D, Topology3D) |
| `Topology2D/` | 2D topological relationships |
| `Meshing/Core/2D/` | 2D Delaunay: ConstrainedDelaunay2D, MeshOperations2D, ShewchukRefiner2D |
| `Meshing/Core/3D/General/` | Shared 3D infrastructure: MeshingContext3D, MeshOperations3D, geometry/quality utils |
| `Meshing/Core/3D/Surface/` | UV-space surface mesher (SurfaceMesher3D, SurfaceMeshingContext3D) — superseded by RCDT |
| `Meshing/Core/3D/Volume/` | Initial (unconstrained) Delaunay tetrahedralization: Delaunay3D. Top-level entry point: VolumeMesher3D |
| `Meshing/Core/3D/RCDT/` | Ambient-space RCDT mesher: RCDTMesher, RCDTContext, RCDTRefiner — the only volume/surface meshing algorithm implemented today, behind ISurfaceMesher3D/IVolumeMesher3D |
| `Meshing/Data/` | MeshData2D/3D, Node2D/3D, TriangleElement, TetrahedralElement |
| `Readers/` | OpenCASCADE CAD import |
| `Export/` | VtkExporter (VTU format) |

### Design Patterns
- **Strategy Pattern**: `IMesher` interface with pluggable implementations
- **Context Pattern**: `MeshingContext2D/3D` manages geometry + topology + mesh lifecycle
- **Friend Classes**: `MeshData` ↔ `MeshMutator` for controlled mutation

### Type Aliases (Eigen-based)
```cpp
using Point2D = Eigen::Vector2d;
using Point3D = Eigen::Vector3d;
```

## Code Conventions

See `.claude/rules/cpp-coding-standards.md` for comprehensive standards. Key points:

- **Headers**: `.h` with `#pragma once`
- **Sources**: `.cpp`
- **Classes**: `CamelCase` (e.g., `ConstrainedDelaunay2D`)
- **Interfaces**: Prefix with `I` (e.g., `ICorner3D`)
- **Members**: `camelCase_` with trailing underscore
- **Constants**: `ALL_CAPS`
- **Formatting**: Microsoft style, 4-space indent, Allman braces

## Debugging & Bug Fixes

When fixing a bug or adding a feature, make the right change — not the convenient one. Do not accumulate code:

- **No patching**: If a bug or feature reveals a structural problem, fix the structure. Do not work around it with local hacks or conditional logic bolted on top of a bad foundation.
- **Architectural changes when needed**: If the correct fix requires refactoring or restructuring, do that. Discuss the plan with the user first, then execute it properly.
- **One change at a time**: Apply a fix, build, and test before trying anything else. Do not layer multiple speculative fixes on top of each other.
- **Remove failed attempts**: If a fix attempt didn't work, remove it entirely before trying the next approach. Never leave dead or commented-out code behind.
- **Use the debugger to investigate**: An lldb debugger is available via MCP. Prefer it over adding `spdlog` statements — use breakpoints, backtraces, and expression evaluation to inspect state. Only add permanent `spdlog` calls when they provide ongoing diagnostic value beyond a single debugging session. (`SPDLOG_LEVEL=debug` is set by default.)
- **Use the LSP for symbol work**: clangd is configured (it finds `build/compile_commands.json` automatically). Use `findReferences`, `goToDefinition`, and `incomingCalls` to trace a symbol rather than grepping for its name — grep matches text, the LSP matches symbols, and it sees overloads, templates, and aliases that a name search misses. The first query after a cold start returns nothing while it indexes; repeat it once.
- **No temporary scaffolding**: Do not add temporary assertions, diagnostic tests, or throwaway log statements. Remove any that were added during investigation once the bug is resolved.
- **Tests must be intentional**: Only add a test if it covers a real scenario worth keeping permanently. Do not add tests just to verify a fix during debugging. (Characterization tests written during a refactor are the one exception — see Refactoring below.)
- **Mesh integrity checks**: When running examples or tests to investigate a bug, always set the environment variable `CHECK_MESH_EACH_ITERATION=1`. This enables per-iteration mesh consistency checks that help catch corruption early.

## Refactoring

Refactoring means changing structure while behaviour stays identical. It is a different activity from bug fixing and carries a different standard of proof: passing tests do not show that a mesh came out the same.

- **Prove it, don't assert it**: run `./scripts/refactor-check.sh` before and after the change. It meshes representative models and diffs every `.vtu` against `tests/golden/`. If a golden changes, the change is not a refactor — stop and report which model moved and how.
- **Re-blessing is a decision, not a step**: `--accept` overwrites the goldens and is only for a behaviour change that was discussed and intended. Never run it to make a failing check pass.
- **No smuggling**: never fix a bug, adjust a tolerance, or change a concept's meaning in the same step as a structural move. If the refactor exposes a bug, say so and leave it — a separate change fixes it afterwards.
- **Inventory before editing**: for any move, split, or rename, first list every call site with the LSP (`findReferences`, `incomingCalls`) rather than grep, and propose the seam. Agree on the seam before touching files.
- **One move per step**: extract, build, `ctest --test-dir build/tests -j$(nproc) -LE slow`, then `./scripts/refactor-check.sh`. Only then start the next move.
- **Offer a checkpoint at each green step**: build clean, fast tests green, goldens unchanged. Say so and offer the commit — a multi-step refactor left uncommitted makes "remove failed attempts entirely" impossible to carry out, because a bad step can no longer be separated from the good ones.
- **Branch before speculating**: if an extraction may not work out, start it on a scratch branch or `git stash` first, so abandoning it is one command rather than manual reconstruction.
- **Delete the old shape**: the pre-refactor code goes in the same step. Never leave the old path beside the new one, behind a flag, or commented out.
- **Characterization tests are allowed**: a test written to pin existing behaviour so it can be restructured safely is worth keeping permanently, and is the exception to "Tests must be intentional" above.
- **Full tier before finishing**: end with `TIER=full ./scripts/refactor-check.sh` and the complete `ctest --test-dir build/tests -j$(nproc)`. The fast tier does not cover `BoxWithHole` or `SaddleSurfaceMesh`.

## Error Handling

See `doc/Error_Handling.md` for complete guide.

```cpp
// Entity lookups (programming errors)
OPENLOOM_THROW_ENTITY_NOT_FOUND("Node", id)
OPENLOOM_REQUIRE_NOT_NULL(ptr, "name")

// Legitimate "not found" results
std::optional<T> result = tryFind(...);

// Simple success/failure
bool success = tryOperation(...);
```

Exception hierarchy: `Exception` → `GeometryException`, `MeshException`, `TopologyException`

## Dependencies

- **VTK** - Mesh export
- **Eigen3** - Linear algebra
- **OpenCASCADE** - CAD geometry (STEP files)
- **spdlog** - Logging (`SPDLOG_LEVEL=debug|info|warn`)
- **GoogleTest** - Testing

## Documentation

- `doc/Terminology.md` - CAD and mesh terminology glossary
- `doc/Error_Handling.md` - Error handling guide
- `doc/Meshing_Core_Overview.md` - Architecture overview
