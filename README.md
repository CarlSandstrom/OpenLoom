# OpenLoom

> **This project is under heavy development and not yet ready for production use.**

A constrained Delaunay triangulation library for 2D and 3D mesh generation with OpenCASCADE CAD support, written in C++20.

## Status

| Component | Status |
|-----------|--------|
| 2D constrained Delaunay mesher | Working |
| 3D surface mesher — UV-space (legacy) | Complete (superseded by RCDT) |
| 3D surface mesher — RCDT | Under development |
| 3D volume mesher — RCDT | Under development |

## Background

OpenLoom is designed with extensibility as a core principle. The architecture makes it straightforward to add new meshing algorithms, refinement strategies, and import/export formats without touching the existing pipeline. Key extension points are defined as interfaces (`IMesher`, `ICorner`, `IEdge`, `ISurface`), and the context pattern cleanly separates geometry, topology, and mesh data.

The 2D mesher uses Ruppert's algorithm (implemented as `ShewchukRefiner2D`) for quality refinement — iteratively inserting Steiner points to eliminate poorly-conditioned triangles. The 3D surface and volume meshers use the **Restricted Constrained Delaunay Triangulation** (RCDT) algorithm by Khoury & Shewchuk (SoCG 2021). RCDT works in ambient 3D space: a single Delaunay tetrahedralization is built over all boundary vertices, and surface triangles emerge implicitly as faces whose adjacent tetrahedra lie on opposite sides of the CAD surface. A legacy UV-space surface mesher (`SurfaceMesher3D`) also exists but is superseded by the RCDT pipeline.

## Features

- 2D constrained Delaunay triangulation (CDT)
- OpenCASCADE integration for importing STEP/IGES CAD geometry
- VTK export (`.vtu`) for visualization in ParaView
- Quality-driven mesh refinement: Ruppert's algorithm for 2D, RCDT (Khoury & Shewchuk 2021) for 3D
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

### 3D surface examples

| Executable | Description |
|-----------|-------------|
| `CylinderSurfaceMesh` | RCDT surface mesh of a cylinder |
| `SurfaceMeshEdges` | Visualise edge discretization and initial surface triangulation |
| `BoxWithHoleSurface` | RCDT surface mesh of a box with a through-hole |
| `ThinFinSurfaceMesh` | RCDT surface mesh of a thin fin geometry |
| `TorusSurfaceMesh` | RCDT surface mesh of a torus |

### 3D volume examples

| Executable | Description |
|-----------|-------------|
| `CreateBox` | Basic 3D box volume mesh |
| `BoxWithHole` | 3D box with a through-hole |
| `ShewchukBox` | Legacy Shewchuk-refined 3D box mesh |
| `ShewchukBoxWithHole` | Legacy Shewchuk refinement with hole constraints |

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

The 2D mesher and its quality refinement are based on:

> J.R. Shewchuk, "Delaunay Refinement Algorithms for Triangular Mesh Generation," *Computational Geometry: Theory and Applications*, 22(1-3):21-74, 2002.

The 3D surface and volume meshers are based on:

> M. Khoury and J.R. Shewchuk, "Restricted Constrained Delaunay Triangulations," *37th International Symposium on Computational Geometry (SoCG 2021)*, LIPIcs, vol. 189, pp. 49:1–49:16, 2021.

## About

OpenLoom is developed by [Sandbricius Consulting AB](https://sandbricius.se). We offer consulting services for tailored mesh generation solutions — contact us if you need custom adaptations, integration support, or domain-specific extensions.

## License

MIT — see [LICENSE](LICENSE) for the full text. This software is provided as-is, without warranty of any kind.
