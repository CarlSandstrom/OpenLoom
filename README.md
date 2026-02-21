# OpenLoom

> **This project is under heavy development and not yet ready for production use.**

A constrained Delaunay triangulation library for 2D and 3D mesh generation with OpenCASCADE CAD support, written in C++20.

## Status

| Component | Status |
|-----------|--------|
| 2D constrained Delaunay mesher | Working |
| 3D surface mesher | Under development |
| 3D volume mesher | Not yet started |

## Background

OpenLoom is designed with extensibility as a core principle. The architecture makes it straightforward to add new meshing algorithms, refinement strategies, and import/export formats without touching the existing pipeline. Key extension points are defined as interfaces (`IMesher`, `ICorner`, `IEdge`, `ISurface`), and the context pattern cleanly separates geometry, topology, and mesh data.

The current meshing and refinement implementation is based on **Shewchuk's algorithm** — a quality Delaunay refinement method that guarantees well-shaped elements by iteratively inserting Steiner points to eliminate poorly-conditioned triangles and tetrahedra. This is used in both the 2D and 3D meshers.

## Features

- 2D constrained Delaunay triangulation (CDT)
- OpenCASCADE integration for importing STEP/IGES CAD geometry
- VTK export (`.vtu`) for visualization in ParaView
- Quality-driven mesh refinement (Shewchuk-style)
- Mesh verification and debug utilities

## Dependencies

| Library | Purpose | Required |
|---------|---------|----------|
| [Eigen3](https://eigen.tuxfamily.org) | Linear algebra | Yes |
| [VTK](https://vtk.org) | Mesh export | Yes |
| [spdlog](https://github.com/gabime/spdlog) | Logging | Yes |
| [fmt](https://github.com/fmtlib/fmt) | String formatting | Yes |
| [GoogleTest](https://github.com/google/googletest) | Testing | Yes |
| [OpenCASCADE](https://dev.opencascade.org) | CAD geometry import | Optional |
| OpenMP | Parallel mesh verification | Optional |

## Building

### Install dependencies (Fedora/RHEL)

```bash
sudo dnf install clang clang-tools-extra cmake
sudo dnf install vtk-devel eigen3-devel spdlog-devel fmt-devel gtest-devel
```

### Configure and build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

#### Optional build flags

```bash
# Enable OpenMP for parallel mesh verification
cmake -S . -B build -DOPENLOOM_USE_OPENMP=ON

# Point to a custom OpenCASCADE installation
cmake -S . -B build -DOpenCASCADE_DIR=$HOME/local/opencascade/lib/cmake/opencascade
```

### Run tests

```bash
ctest --test-dir build/tests --output-on-failure
# or directly:
./build/tests/runTests

# Run a specific test
./build/tests/runTests --gtest_filter="TestClass.TestName"
```

## Examples

After building, executables are in `build/src/Examples2D/` and `build/src/Examples3D/`. Output `.vtu` files can be opened with [ParaView](https://www.paraview.org).

### 2D examples

| Executable | Description |
|-----------|-------------|
| `SimpleDelaunay2D` | Basic 2D Delaunay triangulation |
| `RectangleWithHole2D` | Rectangle mesh with a hole constraint |
| `RectangleWithCrack` | Mesh with crack/slit constraints |
| `SquareWithCircularHole` | Square domain with a circular cutout |
| `SquareWithInternalCircles` | Multiple internal circle constraints |
| `MeshStepFile2D` | Mesh a 2D profile loaded from a STEP file |

### 3D examples

| Executable | Description |
|-----------|-------------|
| `CreateBox` | Basic 3D box mesh |
| `BoxWithHole` | 3D box with a through-hole |
| `ShewchukBox` | Quality-refined 3D box mesh |
| `ShewchukBoxWithHole` | Quality refinement with hole constraints |
| `ConstrainedBoxExample` | 3D constrained mesh example |

```bash
# Run an example and view the result
./build/src/Examples2D/RectangleWithHole2D
paraview output.vtu
```

## Architecture

```
src/
├── Common/         # Types, BoundingBox, Exceptions
├── Geometry/       # CAD geometry entities (ICorner, IEdge, ISurface interfaces)
├── Topology/       # Topological relationships (3D)
├── Topology2D/     # Topological relationships (2D)
├── Meshing/
│   ├── Core/       # Delaunay algorithms (2D, 3D, Bowyer-Watson, constraint recovery)
│   └── Data/       # MeshData2D/3D, Node, TriangleElement, TetrahedralElement
├── Readers/        # OpenCASCADE STEP/IGES import
└── Export/         # VtkExporter (VTU format)
```

Key design patterns:
- **Strategy**: `IMesher` interface with pluggable algorithm implementations
- **Context**: `MeshingContext2D/3D` manages geometry, topology, and mesh lifecycle
- **Controlled mutation**: `MeshData` exposes mutation only through `MeshMutator`

## Logging

Runtime logging level is controlled via environment variable:

```bash
SPDLOG_LEVEL=debug ./build/src/Examples2D/RectangleWithHole2D
# levels: trace, debug, info, warn, error
```

## Building OpenCASCADE (optional)

If you need STEP file support and your distribution doesn't package OpenCASCADE:

```bash
cd ~/software/OCCT-7_8_1/build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/local/opencascade \
    -DBUILD_LIBRARY_TYPE=Shared \
    -DBUILD_MODULE_DataExchange=ON \
    -DBUILD_MODULE_ApplicationFramework=OFF \
    -DBUILD_MODULE_Draw=OFF \
    -DBUILD_MODULE_Visualization=OFF \
    -DUSE_TK=OFF
make -j$(nproc)
make install
```

Then configure OpenLoom with:

```bash
cmake -S . -B build -DOpenCASCADE_DIR=$HOME/local/opencascade/lib/cmake/opencascade
```

## Documentation

- [doc/Terminology.md](doc/Terminology.md) - CAD and mesh terminology glossary
- [doc/Meshing_Core_Overview.md](doc/Meshing_Core_Overview.md) - Architecture overview
- [doc/Error_Handling.md](doc/Error_Handling.md) - Error handling guide

## Contributing

Contributions are welcome. Please open an issue before starting significant work so we can discuss the approach. Bug reports and feedback are equally appreciated.

## Acknowledgements

The meshing and refinement algorithms are based on the work of J.R. Shewchuk:

> J.R. Shewchuk, "Delaunay Refinement Algorithms for Triangular Mesh Generation," *Computational Geometry: Theory and Applications*, 22(1-3):21-74, 2002.

## About

OpenLoom is developed by [Sandbricius Consulting AB](https://sandbricius.se). We offer consulting services for tailored mesh generation solutions — contact us if you need custom adaptations, integration support, or domain-specific extensions.

## License

MIT — see [LICENSE](LICENSE) for the full text. This software is provided as-is, without warranty of any kind.
