---
name: meshing-patterns
description: Meshing architecture patterns for cMesh
globs:
  - "src/Meshing/**/*"
  - "src/Examples/**/*"
---

# Meshing Patterns for cMesh

## Context Pattern
Always access mesh data through contexts, never cache raw pointers:
```cpp
MeshingContext2D context(geometry, topology);
const MeshData2D& mesh = context.getMeshData();
```

## Strategy Pattern (IMesher)
Pluggable meshing algorithms:
```cpp
IMesher* mesher = new ConstrainedMesher();
mesher->generate(context);
```

## Friend Classes (Controlled Mutation)
- `MeshData2D` ↔ `MeshMutator2D`
- `MeshData3D` ↔ `MeshMutator3D`
Use mutators for low-level mesh modifications.

## Typical 2D Workflow
```cpp
MeshingContext2D context(geometry, topology);
ConstrainedDelaunay2D mesher(context);
mesher.generateConstrained();
const MeshData2D& mesh = context.getMeshData();
```

## Typical 3D Workflow
```cpp
MeshingContext3D context(geometry, topology);
ConstrainedMesher mesher;
mesher.generate(context);
const MeshData3D& mesh = context.getMeshData();
```

## Dual-Mode Design (ConstrainedDelaunay2D)
- **Context mode**: Tied to topology, geometry validation
- **Standalone mode**: Raw coordinates only

## Connectivity Rebuilding
Always rebuild connectivity after bulk operations:
```cpp
context.rebuildConnectivity();
```

## Mesh Export
```cpp
Export::VtkExporter exporter;
exporter.writeVtu(meshData, "output.vtu");
```
