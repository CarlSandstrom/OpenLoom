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

Executables in `build/src/Examples/`:
- `SimpleDelaunay2D` - Basic 2D Delaunay triangulation
- `RectangleWithHole2D` - 2D mesh with hole constraints
- `RectangleWithCrack` - 2D mesh with crack constraints
- `SquareWithCircularHole` - 2D square with circular hole
- `BoxWithHole` - 3D mesh with hole
- `ConstrainedBoxExample` - 3D constrained mesh
- `CreateBox` - Basic 3D box mesh

View output with ParaView: `paraview output.vtu`

## Architecture

### Key Modules
| Module | Purpose |
|--------|---------|
| `Common/` | Types, BoundingBox, Exceptions |
| `Geometry/` | 2D/3D geometric entities (ICorner, IEdge, ISurface interfaces) |
| `Topology/` / `Topology2D/` | Topological relationships |
| `Meshing/Core/` | Delaunay algorithms (ConstrainedDelaunay2D/3D, Computer2D/3D) |
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

See `.clinerules` for comprehensive standards. Key points:

- **Headers**: `.h` with `#pragma once`
- **Sources**: `.cpp`
- **Classes**: `CamelCase` (e.g., `ConstrainedDelaunay2D`)
- **Interfaces**: Prefix with `I` (e.g., `ICorner3D`)
- **Members**: `camelCase_` with trailing underscore
- **Constants**: `ALL_CAPS`
- **Formatting**: Microsoft style, 4-space indent, Allman braces

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
