---
name: error-handling
description: Error handling patterns for cMesh
globs:
  - "**/*.cpp"
  - "**/*.hpp"
  - "**/*.h"
---

# Error Handling in cMesh

## Exception Hierarchy
```
Exception (base)
├── GeometryException (codes 1000-1007)
│   ├── EntityNotFoundException
│   └── NullGeometryException
├── MeshException (codes 2000-2008)
│   ├── MeshVerificationException
│   ├── MaxIterationsException
│   └── MeshEntityNotFoundException
└── TopologyException (codes 4000-4002)
```

## When to Use What

### Exceptions (Programming Errors)
Entity lookups that should succeed:
```cpp
CMESH_THROW_ENTITY_NOT_FOUND("Node", id)
CMESH_REQUIRE_NOT_NULL(ptr, "meshData")
CMESH_THROW_CODE(MeshException, ErrorCode::INVALID_MESH, "msg")
```

### std::optional (Valid Absence)
Operations that may legitimately produce no result:
```cpp
std::optional<Point3D> result = computeCircumcenter(element);
if (result) { /* use *result */ }
```

### bool (Simple Success/Failure)
Simple yes/no or success/failure:
```cpp
bool success = tryEnforceConstraint(...);
bool valid = contains(...);
```

### Structured Results
Validation needing error lists:
```cpp
VerificationResult result = verify(mesh);
if (!result.isValid()) {
    for (const auto& error : result.errors()) { ... }
}
```

## Anti-Patterns (Avoid)
- Generic `std::runtime_error`
- Exceptions for valid absences (use `std::optional`)
- Assertions for validation
- `bool` for programming errors
