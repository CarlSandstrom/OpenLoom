# OpenLoom Project Context

Constrained Delaunay triangulation library for 2D and 3D mesh generation with OpenCASCADE CAD support.

## Build & Run

```bash
# Configure (first time only)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build everything
cmake --build build -j$(nproc)

# Run tests
ctest --test-dir build/tests --output-on-failure
# or directly:
./build/tests/runTests

# Run specific test
./build/tests/runTests --gtest_filter="TestClass.TestName"
```

## Examples

2D executables in `build/src/Examples2D/`:
- `SimpleDelaunay2D` - Basic 2D Delaunay triangulation
- `RectangleWithHole2D` - 2D mesh with hole constraints
- `RectangleWithCrack` - 2D mesh with crack constraints
- `SquareWithCircularHole` - 2D square with circular hole
- `SquareWithCircleAndTwinEdges`
- `SquareWithInternalCircles`
- `MeshStepFile2D`

3D executables in `build/src/Examples3D/`:
- `BoxWithHole` - 3D mesh with hole
- `ConstrainedBoxExample` - 3D constrained mesh
- `CreateBox` - Basic 3D box mesh
- `CylinderSurfaceMesh`
- `ShewchukBox`
- `ShewchukBoxWithHole`
- `SurfaceMeshEdges`

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
| `Meshing/Core/3D/Surface/` | Surface mesh generation: SurfaceMeshingContext3D, FacetTriangulation |
| `Meshing/Core/3D/Volume/` | Volume mesh generation: ConstrainedDelaunay3D, ShewchukRefiner3D |
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

When fixing a bug, identify and apply the minimal correct change. Do not accumulate code:

- **One fix at a time**: Apply a fix, build, and test before trying anything else. Do not layer multiple speculative fixes on top of each other.
- **Remove failed attempts**: If a fix attempt didn't work, remove it entirely before trying the next approach. Never leave dead or commented-out code behind.
- **Use logging to investigate**: Since there is no debugger access, use `spdlog` calls freely to inspect state during debugging (`SPDLOG_LEVEL=debug` is set by default). Remove temporary log statements once the bug is resolved, unless they are genuinely useful to keep.
- **No other temporary scaffolding**: Do not add temporary assertions or diagnostic tests. Remove any that were added during investigation once the bug is resolved.
- **Tests must be intentional**: Only add a test if it covers a real scenario worth keeping permanently. Do not add tests just to verify a fix during debugging.

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
