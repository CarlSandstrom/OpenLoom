# OpenLoom Error Handling Guide

Quick reference for standardized error handling in OpenLoom.

## Decision Tree

| Situation | Use | Example |
|-----------|-----|---------|
| Entity should exist (programming error if not) | **Exception** | `OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id)` |
| Geometric computation may fail (degenerate) | **`std::optional<T>`** | `std::optional<Circle2D> computeCircumcircle(...)` |
| Query where "not found" is valid | **`std::optional<T>`** or `nullptr` | `std::optional<std::string> findVertexId(...)` |
| Simple yes/no or try operation | **`bool`** | `bool tryEnforceConstraint(...)` |
| Operation with multiple failure modes | **Exception** with error code | `OPENLOOM_THROW_MAX_ITERATIONS(...)` |
| Validation needing error list | **Struct** | `VerificationResult verify()` |
| File I/O | **Stream exceptions** | `os.exceptions(std::ios::failbit)` |
| Null pointer validation | **Exception** | `OPENLOOM_REQUIRE_NOT_NULL(ptr, "name")` |

## Exception Hierarchy

```
OpenLoom::Exception (base)
├── GeometryException (error codes 1000-1007)
│   ├── EntityNotFoundException
│   └── NullGeometryException
├── MeshException (error codes 2000-2008)
│   ├── MeshVerificationException
│   ├── MaxIterationsException
│   └── MeshEntityNotFoundException
└── TopologyException (error codes 4000-4002)
```

## Exception Types

### GeometryException
**Location:** `Common/Exceptions/GeometryException.h`

**Error Codes:**
- `ENTITY_NOT_FOUND` (1000), `INVALID_GEOMETRY` (1001), `NULL_POINTER` (1002)
- `EMPTY_COLLECTION` (1003), `WIRE_BUILDING_FAILED` (1004), `EDGE_LOOP_INVALID` (1005)
- `PARAMETER_OUT_OF_RANGE` (1006), `DEGENERATE_GEOMETRY` (1007)

**Usage:**
```cpp
OPENLOOM_THROW_GEOMETRY("Invalid geometry");
OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", surfaceId);
OPENLOOM_REQUIRE_NOT_NULL(edge, "edge");
OPENLOOM_THROW_CODE(OpenLoom::GeometryException, ErrorCode::WIRE_BUILDING_FAILED, "msg");
```

### MeshException
**Location:** `Common/Exceptions/MeshException.h`

**Error Codes:**
- `GENERATION_FAILED` (2000), `VERIFICATION_FAILED` (2001), `DEGENERATE_TRIANGLE` (2002)
- `CONSTRAINT_VIOLATION` (2003), `MAX_ITERATIONS_REACHED` (2004), `INVALID_TOPOLOGY` (2005)
- `NODE_NOT_FOUND` (2006), `ELEMENT_NOT_FOUND` (2007), `INVALID_OPERATION` (2008)

**Usage:**
```cpp
OPENLOOM_THROW_MESH(GENERATION_FAILED, "Mesh generation failed");
OPENLOOM_THROW_VERIFICATION_FAILED("Verification failed", errorList);
OPENLOOM_THROW_MAX_ITERATIONS("Mesh refinement", 100);
throw OpenLoom::MeshEntityNotFoundException("Node", nodeId, __FILE__ ":" + std::to_string(__LINE__));
```

### TopologyException
**Location:** `Common/Exceptions/TopologyException.h`

**Error Codes:**
- `ENTITY_NOT_FOUND` (4000), `INVALID_CONNECTION` (4001), `INCONSISTENT_TOPOLOGY` (4002)

**Usage:**
```cpp
OPENLOOM_THROW_TOPOLOGY(ENTITY_NOT_FOUND, "Corner2D not found: " + id);
```

## Helper Macros

All macros automatically capture file:line location.

```cpp
// General
OPENLOOM_THROW(ExceptionType, message)
OPENLOOM_THROW_CODE(ExceptionType, code, message)
OPENLOOM_REQUIRE(condition, ExceptionType, message)

// Geometry
OPENLOOM_THROW_GEOMETRY(message)
OPENLOOM_THROW_ENTITY_NOT_FOUND(entityType, entityId)
OPENLOOM_REQUIRE_NOT_NULL(ptr, name)

// Mesh
OPENLOOM_THROW_MESH(code, message)
OPENLOOM_THROW_VERIFICATION_FAILED(message, errorVector)
OPENLOOM_THROW_MAX_ITERATIONS(operation, maxIter)

// Topology
OPENLOOM_THROW_TOPOLOGY(code, message)
```

## Usage Patterns

### Entity Lookup (should exist)
```cpp
ISurface3D* getSurface(const std::string& id) const {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id);
    }
    return it->second.get();
}
```

### Optional Result (may not exist)
```cpp
std::optional<Circle2D> computeCircumcircle(const Triangle& tri) const {
    if (std::abs(determinant) < 1e-10) {
        return std::nullopt;  // Degenerate
    }
    return Circle2D{center, radius};
}
```

### Simple Try Operation
```cpp
bool tryEnforceConstraint(const Edge& edge) {
    if (!canEnforce(edge)) {
        return false;
    }
    enforceEdge(edge);
    return true;
}
```

### Validation with Error Collection
```cpp
VerificationResult verify() const {
    VerificationResult result;
    result.isValid = true;

    if (!checkOrientation()) {
        result.errors.push_back("Invalid orientation");
        result.isValid = false;
    }
    return result;
}
```

### File I/O
```cpp
void exportMesh(const MeshData3D& mesh, const std::string& path) const {
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(path);  // Throws std::ios_base::failure if fails
    // write data...
}
```

### Null Pointer Validation
```cpp
void addHole(std::unique_ptr<EdgeLoop> hole) {
    OPENLOOM_REQUIRE_NOT_NULL(hole, "hole edge loop");
    holes_.push_back(std::move(hole));
}
```

## Anti-Patterns

❌ **Don't use generic `std::runtime_error`**
```cpp
throw std::runtime_error("Error");  // Use typed exceptions
```

❌ **Don't use exceptions for valid absences**
```cpp
// Bad: throws when point not in any triangle (valid case)
int getTriangleAt(Point2D p) { throw ...; }
// Good: returns optional
std::optional<int> getTriangleAt(Point2D p);
```

❌ **Don't use `bool` for programming errors**
```cpp
// Bad: caller can't tell what went wrong
bool tryGetSurface(const std::string& id, Surface*& out);
// Good: throw exception with context
Surface* getSurface(const std::string& id);
```

❌ **Don't use assertions for validation**
```cpp
assert(node != nullptr);  // Use OPENLOOM_REQUIRE_NOT_NULL instead
```

## Catching Exceptions

```cpp
try {
    mesh = generator.generateMesh(geometry);
}
catch (const OpenLoom::EntityNotFoundException& e) {
    // Specific handling - missing geometry
    spdlog::error("{} ({})", e.message(), e.getEntityId());
}
catch (const OpenLoom::MaxIterationsException& e) {
    // Could retry with different parameters
    spdlog::warn("{} after {}", e.getOperation(), e.getMaxIterations());
}
catch (const std::ios_base::failure& e) {
    // File I/O errors
}
catch (const OpenLoom::Exception& e) {
    // Generic OpenLoom error
    spdlog::error("Error at {}: {}", e.location(), e.message());
}
```

## Summary

**Use exceptions when:** Failure is exceptional and needs context
**Use `std::optional<T>` when:** Absence is valid, return full object
**Use `bool` when:** Simple success/failure, no error details needed
**Use struct when:** Need to collect multiple errors/warnings
