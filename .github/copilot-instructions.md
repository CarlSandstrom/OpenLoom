# cMesh Agent Guide

## Architecture & Data Flow
- `Common/` hosts math helpers (`BoundingBox{2,3}D`, `MathConverter`, `Types`) that every module includes; keep new utilities dependency-free so they can compile in headers.
- Geometry lives behind interfaces in `src/Geometry/Base` (`Surface`, `Edge`, `Corner`, `GeometryCollection`) while `Geometry/OpenCascade` supplies the concrete OpenCascade-backed shapes; treat `GeometryCollection` as the single entry point for geometry data.
- `Topology/Topology` mirrors the same IDs and provides validation helpers (`isValid`, `isManifold`) that meshing code calls before generating elements.
- `Meshing/Core/MeshingContext` ties a `GeometryCollection` + `Topology` to mutable mesh state (`Meshing/Data/*`); call `getMeshData()/getConnectivity()` through the context instead of caching the owning pointers yourself.
- Mesh generation strategies implement `Meshing/Core/IMesher`; `SimpleMesher` and `Delaunay3D` show the pattern of pulling nodes from geometry, inserting through `MeshOperations`, and optionally invoking a `IQualityController`.
- Export flows through `Export/VtkExporter`, which currently emits ASCII `.vtu` (tetra support only); integration tests (`tests/Integration/test_VtkExporter.cpp`) assert the full XML skeleton, so keep new tags consistent.

## Key Modules & Extension Points
- Readers: `Readers/OpenCascade` + `ShapeConverter` create `GeometryCollection`/`Topology` pairs from CAD; add new importers by returning those same abstractions so solvers remain engine-agnostic.
- Meshing Data: `MeshData`, `MeshConnectivity`, and `MeshOperations` own `std::unique_ptr` containers and expose mutation only through friend APIs; never modify the containers directly.
- Meshing Operations: transaction-style helpers in `Meshing/Operations/` update nodes/elements and rebuild adjacency; call `MeshingContext::rebuildConnectivity()` after bulk edits.
- Export: `Export/IExporter` is intentionally tiny—new exporters should mimic `VtkExporter::exportMesh` signature and keep file I/O streaming (std::ostream) friendly.
- Examples: `src/Examples/*.cc` link every core library plus OpenCascade/VTK (see `src/Examples/CMakeLists.txt`) and are the fastest way to exercise end-to-end workflows outside tests.

## Build & Test Workflow
- Configure once with `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug` (top-level `CMakeLists.txt` pins Clang and enables `-g3 -O0` flags); rerun when dependencies change.
- Build everything via `cmake --build build -j$(nproc)`; binaries land under `build/src/...` and `build/tests/runTests`.
- Run the full GoogleTest suite with `ctest --test-dir build/tests --output-on-failure` or launch `./build/tests/runTests` directly; tests cover geometry primitives, meshing data/core/operations, and integration exporters.
- Examples depend on OpenCascade + VTK dev packages (`README.md` lists Fedora packages); ensure `OpenCASCADE_DIR` resolves to `$HOME/lib/opencascade/lib/cmake/opencascade` or override before configuring.

## Coding Standards (Must Follow)
- Files: headers `.h`, sources `.cpp`, one class per file, `#pragma once`, no `using namespace`.
- Naming: classes `CamelCase`, variables `camelCase`, members `camelCase_`, constants `ALL_CAPS`, setters `set...`, getters `get...`.
- Functions: prefer `const` correctness everywhere, pass non-primitives by `const&`, one declaration per line.
- Classes: no public/protected members or friends beyond the existing data/operations friendship, ownership via `std::unique_ptr`, only `static_cast`/`dynamic_cast`.
- General: rely on `auto` when it improves clarity, wrap namespaces as `namespace Foo { ... }`, include headers via paths relative to `src/`.
- Only declarations in headers, implementations in `.cpp` files. Structs should go into another header file for each library where the libraries structs are kept.

## Implementation Tips
- Favor forward declarations in headers (`Coding standards`) and include the concrete headers inside `.cpp` files to keep compile units light—see `Meshing/Core/IMesher.h`.
- When authoring new tests, add the file to `tests/CMakeLists.txt` so it links against the same interface libraries (`Geometry`, `Meshing`, `Export`, `Readers`, `Common`).
- Use `docs/` for background context (e.g., `doc/Coding standards.md`, `doc/Other/MeshData_C++20_Improvements.md`) before refactoring core data structures; those notes explain rationale behind friend APIs and C++17 choices.
