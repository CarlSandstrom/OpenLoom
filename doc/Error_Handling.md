# cMesh Error Handling Guide

This document describes the error handling strategy and exception hierarchy used in cMesh.

## Table of Contents

- [Overview](#overview)
- [Exception Hierarchy](#exception-hierarchy)
- [When to Use What](#when-to-use-what)
- [Exception Types](#exception-types)
- [Helper Macros](#helper-macros)
- [Usage Examples](#usage-examples)
- [Best Practices](#best-practices)
- [Error Handling at Call Sites](#error-handling-at-call-sites)
- [Testing Error Conditions](#testing-error-conditions)

## Overview

cMesh uses a hybrid error handling approach that combines:
- **Custom exceptions** for exceptional error conditions and unrecoverable failures
- **`std::optional<T>`** for operations that may legitimately produce no result
- **Standard library exceptions** (`std::ios_base::failure`) for file I/O
- **Structured results** (`VerificationResult`) for validation operations

This approach provides type safety, clear semantics, and detailed error context while maintaining idiomatic C++ patterns.

## Exception Hierarchy

All cMesh exceptions derive from `cMesh::Exception`, which inherits from `std::exception`:

```
std::exception
    └── cMesh::Exception
            ├── cMesh::GeometryException
            │       ├── EntityNotFoundException
            │       └── NullGeometryException
            ├── cMesh::MeshException
            │       ├── MeshVerificationException
            │       ├── MaxIterationsException
            │       └── MeshEntityNotFoundException
            └── cMesh::TopologyException
```

### Base Exception Features

The `cMesh::Exception` base class provides:
- **Error message**: Human-readable description
- **Location tracking**: File and line number where exception was thrown
- **Error codes**: Numeric codes for programmatic error handling
- **Type information**: Runtime type identification via `typeName()`

## When to Use What

### Use Exceptions When:

✅ **Precondition violations** - Invalid input to a public API
✅ **Entity not found** - Required resource doesn't exist (programming error)
✅ **Operation cannot proceed** - Unrecoverable failure (file I/O, wire building)
✅ **Contract violations** - Mesh verification failures, invalid topology
✅ **Invalid operations** - Attempting to remove a node still referenced by elements

**Examples:**
```cpp
// Entity lookup that should always succeed (programming error if it fails)
ISurface3D* GeometryCollection3D::getSurface(const std::string& id) const {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) {
        CMESH_THROW_ENTITY_NOT_FOUND("Surface", id);
    }
    return it->second.get();
}

// Null pointer check
void OpenCascade2DFace::addHole(std::unique_ptr<OpenCascade2DEdgeLoop> holeEdgeLoop) {
    CMESH_REQUIRE_NOT_NULL(holeEdgeLoop, "hole edge loop");
    // ...
}
```

### Use `std::optional<T>` When:

✅ **Legitimate absence** - Not finding something is a valid outcome
✅ **Degenerate cases** - Geometric computation on degenerate elements
✅ **Query operations** - Searching where "not found" is expected
✅ **Caller decides importance** - Let caller determine if absence is an error

**Examples:**
```cpp
// Geometric computation that may fail for degenerate triangles
std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri) const {
    double d = /* determinant calculation */;
    if (std::abs(d) < 1e-10) {  // Degenerate triangle
        return std::nullopt;
    }
    return CircumCircle2D{center, radius};
}

// Query where "not found" is a valid answer
std::optional<std::string> findVertexId(const Point3D& point, double tolerance) const {
    for (const auto& [id, vertex] : vertices_) {
        if (distance(vertex, point) < tolerance) {
            return id;
        }
    }
    return std::nullopt;  // Not found - that's okay
}
```

### Use Structured Results When:

✅ **Validation operations** - Need to collect all errors, not just first one
✅ **Multiple output values** - Success/failure plus additional data
✅ **Non-throwing validation** - Validation as a query, not enforcement

**Examples:**
```cpp
struct VerificationResult {
    bool isValid;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

VerificationResult MeshVerifier::verify() const {
    VerificationResult result;
    result.isValid = true;

    // Collect all errors, don't stop at first one
    if (!checkOrientation()) {
        result.errors.push_back("Invalid triangle orientation");
        result.isValid = false;
    }
    if (!checkOverlaps()) {
        result.errors.push_back("Overlapping triangles detected");
        result.isValid = false;
    }

    return result;
}
```

### Use Return Codes When:

✅ **Simple success/failure** - Operation either worked or didn't, no details needed
✅ **Expected failures** - Failure is a normal, anticipated outcome
✅ **Performance-critical paths** - Exception overhead is unacceptable
✅ **Optional operations** - Attempting something that may not work
✅ **Interface constraints** - Implementing an interface that requires `bool`

**Return code types:**
- `bool` - Simple success (true) / failure (false)
- `nullptr` - Pointer lookup where absence is valid
- `-1` or special values - Numeric sentinel values

**Examples:**

```cpp
// ✅ GOOD - Simple try operation
bool tryEnforceConstraint(const Edge& edge) {
    if (canEnforce(edge)) {
        enforceEdge(edge);
        return true;
    }
    return false;  // Couldn't enforce - that's okay
}

// ✅ GOOD - Query operation
bool contains(const Point2D& point) const {
    return classify(point) == PointLocation::Inside;
}

// ✅ GOOD - Optional pointer return
const Node2D* findNodeAt(const Point2D& point, double tolerance) const {
    for (const auto& [id, node] : nodes_) {
        if (distance(node->getCoordinates(), point) < tolerance) {
            return node.get();
        }
    }
    return nullptr;  // Not found
}

// ✅ GOOD - Interface requirement (even though we throw internally)
bool IExporter::exportMesh(const MeshData3D& mesh, const std::string& filePath) const override {
    // Enable stream exceptions - will throw std::ios_base::failure if file fails
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    writeData(os, mesh);
    return true;  // Required by interface
}
```

**When NOT to use return codes:**

❌ **DON'T use for programming errors** - Use exceptions instead
```cpp
// ❌ BAD - Entity should exist, failure is a bug
bool tryGetSurface(const std::string& id, ISurface3D*& outSurface) const {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) {
        return false;  // Caller won't know what went wrong!
    }
    outSurface = it->second.get();
    return true;
}

// ✅ GOOD - Throw exception, this is a programming error
ISurface3D* getSurface(const std::string& id) const {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) {
        CMESH_THROW_ENTITY_NOT_FOUND("Surface", id);  // Clear error with context
    }
    return it->second.get();
}
```

❌ **DON'T lose error information**
```cpp
// ❌ BAD - Can't tell WHY it failed
bool exportMesh(const MeshData3D& mesh, const std::string& filePath) const {
    std::ofstream os(filePath);
    if (!os.is_open()) {
        return false;  // Was it permissions? Bad path? Disk full?
    }
    // ...
}

// ✅ GOOD - Throw exception with context
void exportMesh(const MeshData3D& mesh, const std::string& filePath) const {
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);  // Throws std::ios_base::failure with detailed error
    // ...
}
```

❌ **DON'T use for recoverable errors needing details**
```cpp
// ❌ BAD - Caller can't distinguish between failure modes
bool refine() {
    if (iterations >= maxIterations_) {
        return false;  // Max iterations? Constraint violation? Can't tell!
    }
    if (constraintViolated()) {
        return false;  // Same return value, different meaning
    }
    // ...
    return true;
}

// ✅ GOOD - Use exceptions with error codes
void refine() {
    if (iterations >= maxIterations_) {
        CMESH_THROW_MAX_ITERATIONS("Mesh refinement", maxIterations_);
    }
    if (constraintViolated()) {
        CMESH_THROW_CODE(cMesh::MeshException,
                        cMesh::MeshException::ErrorCode::CONSTRAINT_VIOLATION,
                        "Edge constraint violated");
    }
    // ...
}
```

**Return Codes vs Optional vs Exceptions - Decision Guide:**

| Scenario | Use | Rationale |
|----------|-----|-----------|
| Entity should exist | **Exception** | Programming error if missing |
| Query "does X exist?" | **`bool`** | Yes/no question |
| Find entity that may not exist | **`std::optional<T>`** | Absence is valid, return full object |
| Find entity by pointer | **`T*` (nullptr)** | Legacy pattern, prefer optional |
| Try optional operation | **`bool`** | Success/failure is the answer |
| Operation with multiple failure modes | **Exception** | Need error details |
| Validation with error list | **Struct** | Multiple errors to report |
| File I/O | **Exception** | Detailed error needed |
| Performance-critical query | **`bool`** | Avoid exception overhead |

## Exception Types

### GeometryException

**Location:** `Common/Exceptions/GeometryException.h`

**Error Codes:**
- `ENTITY_NOT_FOUND` (1000) - Geometry entity (surface, edge, corner) not found
- `INVALID_GEOMETRY` (1001) - Invalid or malformed geometry
- `NULL_POINTER` (1002) - Null pointer encountered
- `EMPTY_COLLECTION` (1003) - Required collection is empty
- `WIRE_BUILDING_FAILED` (1004) - OpenCASCADE wire/face construction failed
- `EDGE_LOOP_INVALID` (1005) - Edge loop is invalid
- `PARAMETER_OUT_OF_RANGE` (1006) - Parameter value outside valid range
- `DEGENERATE_GEOMETRY` (1007) - Geometry is degenerate

**Specialized Types:**
- `EntityNotFoundException` - Specific entity not found with type and ID
- `NullGeometryException` - Null pointer in geometry operation

**Usage:**
```cpp
// Generic geometry error
CMESH_THROW_GEOMETRY("Invalid geometry configuration");

// Entity not found
CMESH_THROW_ENTITY_NOT_FOUND("Surface", surfaceId);

// Null pointer check
CMESH_REQUIRE_NOT_NULL(edge, "edge");

// Specific error code
CMESH_THROW_CODE(cMesh::GeometryException,
                cMesh::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                "Failed to create face from outer wire");
```

### MeshException

**Location:** `Common/Exceptions/MeshException.h`

**Error Codes:**
- `GENERATION_FAILED` (2000) - Mesh generation failed
- `VERIFICATION_FAILED` (2001) - Mesh verification failed
- `DEGENERATE_TRIANGLE` (2002) - Degenerate triangle detected
- `CONSTRAINT_VIOLATION` (2003) - Constraint violation
- `MAX_ITERATIONS_REACHED` (2004) - Algorithm did not converge
- `INVALID_TOPOLOGY` (2005) - Invalid mesh topology
- `NODE_NOT_FOUND` (2006) - Mesh node not found
- `ELEMENT_NOT_FOUND` (2007) - Mesh element not found
- `INVALID_OPERATION` (2008) - Invalid mesh operation

**Specialized Types:**
- `MeshVerificationException` - Mesh validation failure with error list
- `MaxIterationsException` - Convergence failure with operation name and iteration count
- `MeshEntityNotFoundException` - Mesh entity not found with type and ID

**Usage:**
```cpp
// Generic mesh error
CMESH_THROW_MESH(GENERATION_FAILED, "Mesh generation failed");

// Verification failure with error list
CMESH_THROW_VERIFICATION_FAILED("Mesh verification failed", result.errors);

// Max iterations
CMESH_THROW_MAX_ITERATIONS("Mesh refinement", maxIterations_);

// Entity not found
throw cMesh::MeshEntityNotFoundException("Node", nodeId,
    std::string(__FILE__) + ":" + std::to_string(__LINE__));

// Invalid operation
CMESH_THROW_CODE(cMesh::MeshException,
                cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                "Cannot remove node: still referenced by elements");
```

### TopologyException

**Location:** `Common/Exceptions/TopologyException.h`

**Error Codes:**
- `ENTITY_NOT_FOUND` (4000) - Topology entity not found
- `INVALID_CONNECTION` (4001) - Invalid topology connection
- `INCONSISTENT_TOPOLOGY` (4002) - Inconsistent topology state

**Usage:**
```cpp
CMESH_THROW_TOPOLOGY(ENTITY_NOT_FOUND, "Corner2D not found: " + id);
```

### File I/O Exceptions

**Standard Library:** `std::ios_base::failure`

For file I/O, enable stream exceptions to let the standard library handle errors:

```cpp
bool VtkExporter::exportMesh(const MeshData3D& mesh, const std::string& filePath) const {
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);  // Enable exceptions
    os.open(filePath);  // Throws std::ios_base::failure if it fails

    writeHeader(os);
    writePoints(os, mesh);
    writeCells(os, mesh);
    writeFooter(os);
    return true;
}
```

## Helper Macros

All macros automatically capture file and line number.

### General Exception Macros

```cpp
// Throw any exception type with location
CMESH_THROW(ExceptionType, message)

// Throw exception with error code
CMESH_THROW_CODE(ExceptionType, code, message)

// Conditional throw (precondition check)
CMESH_REQUIRE(condition, ExceptionType, message)
```

### Geometry-Specific Macros

```cpp
// Throw generic geometry exception
CMESH_THROW_GEOMETRY(message)

// Throw entity not found
CMESH_THROW_ENTITY_NOT_FOUND(entityType, entityId)

// Require non-null pointer
CMESH_REQUIRE_NOT_NULL(ptr, name)
```

### Mesh-Specific Macros

```cpp
// Throw mesh exception with error code
CMESH_THROW_MESH(code, message)

// Throw verification failure with error list
CMESH_THROW_VERIFICATION_FAILED(message, errors)

// Throw max iterations exception
CMESH_THROW_MAX_ITERATIONS(operation, maxIter)
```

### Topology-Specific Macros

```cpp
// Throw topology exception with error code
CMESH_THROW_TOPOLOGY(code, message)
```

## Usage Examples

### Example 1: Entity Lookup

```cpp
const Edge2D& Topology2D::getEdge(const std::string& id) const {
    auto it = edges_.find(id);
    if (it == edges_.end()) {
        CMESH_THROW_TOPOLOGY(ENTITY_NOT_FOUND, "Edge2D not found: " + id);
    }
    return it->second;
}
```

### Example 2: Mesh Verification

```cpp
void ConstrainedDelaunay2D::verifyMesh() const {
    MeshVerifier verifier(*meshData2D_);
    auto result = verifier.verify();

    if (!result.isValid) {
        for (const auto& error : result.errors) {
            spdlog::error(" - {}", error);
        }
        CMESH_THROW_VERIFICATION_FAILED("Mesh verification failed", result.errors);
    }
}
```

### Example 3: Null Pointer Validation

```cpp
OpenCascade2DFace::OpenCascade2DFace(std::unique_ptr<OpenCascade2DEdgeLoop> outerEdgeLoop)
    : outerEdgeLoop_(std::move(outerEdgeLoop))
    , faceBuilt_(false)
{
    CMESH_REQUIRE_NOT_NULL(outerEdgeLoop_, "outer edge loop");
}
```

### Example 4: Optional Return

```cpp
std::optional<CircumCircle2D> Computer2D::computeCircumcircle(const TriangleElement& tri) const {
    const auto& p1 = meshData_.getNode(tri.getNodeIds()[0])->getCoordinates();
    const auto& p2 = meshData_.getNode(tri.getNodeIds()[1])->getCoordinates();
    const auto& p3 = meshData_.getNode(tri.getNodeIds()[2])->getCoordinates();

    double d = /* determinant */;

    if (std::abs(d) < 1e-10) {  // Degenerate triangle
        return std::nullopt;
    }

    // Calculate circumcircle
    return CircumCircle2D{center, radius};
}

// Usage
auto circle = computer.computeCircumcircle(triangle);
if (circle) {
    // Use circle->center, circle->radius
} else {
    // Triangle is degenerate, handle appropriately
}
```

### Example 5: Invalid Operation

```cpp
void MeshMutator3D::validateNodeRemoval(size_t nodeId) const {
    if (connectivity_ && !connectivity_->canRemoveNode(nodeId)) {
        const auto& elements = connectivity_->getNodeElements(nodeId);
        CMESH_THROW_CODE(cMesh::MeshException,
                        cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                        "Cannot remove node " + std::to_string(nodeId) +
                        ": still referenced by " + std::to_string(elements.size()) + " element(s)");
    }
}
```

## Best Practices

### DO:

✅ **Use specific exception types** - Enables targeted catch blocks
✅ **Include context in messages** - IDs, values, file paths
✅ **Use macros for automatic location** - File and line number tracking
✅ **Log before throwing** - Use `spdlog::error()` for debugging
✅ **Validate at API boundaries** - Public methods should check preconditions
✅ **Document exceptions** - Note which exceptions functions can throw
✅ **Use `std::optional` for valid absences** - Don't throw for "not found" queries
✅ **Provide error codes** - Allows programmatic error handling

### DON'T:

❌ **Use generic `std::runtime_error`** - Always use typed exceptions
❌ **Use assertions for validation** - Use exceptions for maintainability in release builds
❌ **Throw from destructors** - Causes program termination
❌ **Throw from `noexcept` functions** - Violates contract
❌ **Use exceptions for control flow** - Only for exceptional conditions
❌ **Swallow exceptions silently** - At least log them
❌ **Mix "not found" semantics** - Be consistent (optional vs exception)

### Error Messages

**Good:**
```cpp
CMESH_THROW_ENTITY_NOT_FOUND("Surface", "roof-surface-123");
// Error: Surface with ID 'roof-surface-123' not found
//   at GeometryCollection3D.cpp:21

CMESH_THROW_CODE(cMesh::GeometryException,
                cMesh::GeometryException::ErrorCode::PARAMETER_OUT_OF_RANGE,
                "Hole index 5 out of range (size: 3)");
// Error: Hole index 5 out of range (size: 3)
//   at OpenCascade2DFace.cpp:48
```

**Bad:**
```cpp
throw std::runtime_error("Error");  // Too vague
throw std::runtime_error("Surface not found");  // Missing ID
throw std::runtime_error("Failed");  // No context
```

## Error Handling at Call Sites

### Catching Specific Exception Types

```cpp
#include "Common/Exceptions/GeometryException.h"
#include "Common/Exceptions/MeshException.h"
#include <spdlog/spdlog.h>

try {
    auto mesh = generator.generateMesh(geometry, settings);
    exporter.exportMesh(mesh, "output.vtu");
}
// Most specific first
catch (const cMesh::EntityNotFoundException& e) {
    spdlog::error("Missing geometry entity: {} (type: {}, id: {})",
                  e.message(), e.getEntityType(), e.getEntityId());
    // Could prompt user to fix geometry
}
catch (const cMesh::MaxIterationsException& e) {
    spdlog::warn("Algorithm did not converge: {} after {} iterations",
                 e.getOperation(), e.getMaxIterations());
    // Could retry with different parameters
}
catch (const std::ios_base::failure& e) {
    spdlog::error("File I/O error: {}", e.what());
    // Handle file system errors
}
// More general exception types
catch (const cMesh::MeshException& e) {
    spdlog::error("Mesh error (code {}): {}", e.errorCode(), e.message());
}
catch (const cMesh::GeometryException& e) {
    spdlog::error("Geometry error (code {}): {}", e.errorCode(), e.message());
}
// Base exception as fallback
catch (const cMesh::Exception& e) {
    spdlog::error("cMesh error at {}: {}", e.location(), e.message());
}
catch (const std::exception& e) {
    spdlog::error("Unexpected error: {}", e.what());
}
```

### Programmatic Error Handling

```cpp
try {
    mesh.refine();
}
catch (const cMesh::MeshException& e) {
    switch (e.code()) {
        case cMesh::MeshException::ErrorCode::MAX_ITERATIONS_REACHED:
            // Increase max iterations and retry
            settings.maxIterations *= 2;
            spdlog::warn("Retrying with {} iterations", settings.maxIterations);
            mesh.refine(settings);
            break;

        case cMesh::MeshException::ErrorCode::CONSTRAINT_VIOLATION:
            // Relax constraints or report to user
            spdlog::error("Constraint violation - cannot proceed");
            return false;

        default:
            // Generic handling
            spdlog::error("Mesh refinement failed: {}", e.message());
            throw;  // Re-throw
    }
}
```

### Error Recovery Example

```cpp
std::optional<MeshData2D> generateMeshWithFallback(
    const Geometry2D& geometry,
    const Settings& settings)
{
    try {
        return generator.generateMesh(geometry, settings);
    }
    catch (const cMesh::MaxIterationsException& e) {
        spdlog::warn("Mesh generation did not converge, trying with relaxed settings");

        Settings relaxedSettings = settings;
        relaxedSettings.maxIterations *= 2;
        relaxedSettings.qualityThreshold *= 0.8;

        try {
            return generator.generateMesh(geometry, relaxedSettings);
        }
        catch (const cMesh::MeshException& e2) {
            spdlog::error("Mesh generation failed even with relaxed settings");
            return std::nullopt;
        }
    }
    catch (const cMesh::GeometryException& e) {
        spdlog::error("Invalid geometry: {}", e.message());
        return std::nullopt;
    }
}
```

## Testing Error Conditions

### Testing Exception Throwing

```cpp
#include <gtest/gtest.h>
#include "Common/Exceptions/GeometryException.h"

TEST(GeometryCollection, ThrowsWhenSurfaceNotFound) {
    GeometryCollection3D collection;

    EXPECT_THROW({
        collection.getSurface("non-existent-id");
    }, cMesh::EntityNotFoundException);
}

TEST(GeometryCollection, EntityNotFoundHasCorrectInfo) {
    GeometryCollection3D collection;

    try {
        collection.getSurface("test-surface-123");
        FAIL() << "Should have thrown EntityNotFoundException";
    }
    catch (const cMesh::EntityNotFoundException& e) {
        EXPECT_EQ(e.getEntityType(), "Surface");
        EXPECT_EQ(e.getEntityId(), "test-surface-123");
        EXPECT_EQ(e.code(), cMesh::GeometryException::ErrorCode::ENTITY_NOT_FOUND);
        EXPECT_FALSE(e.location().empty());
    }
}

TEST(MeshVerifier, ThrowsOnInvalidMesh) {
    MeshData2D invalidMesh = createInvalidMesh();

    EXPECT_THROW({
        verifier.verifyOrThrow(invalidMesh);
    }, cMesh::MeshVerificationException);
}
```

### Testing Optional Return

```cpp
TEST(Computer2D, ReturnsNulloptForDegenerateTriangle) {
    Computer2D computer(meshData);
    TriangleElement degenerateTriangle = createDegenerateTriangle();

    auto result = computer.computeCircumcircle(degenerateTriangle);

    EXPECT_FALSE(result.has_value());
}

TEST(Computer2D, ReturnsCircumcircleForValidTriangle) {
    Computer2D computer(meshData);
    TriangleElement validTriangle = createValidTriangle();

    auto result = computer.computeCircumcircle(validTriangle);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->radius, expectedRadius, 1e-6);
}
```

### Testing File I/O Exceptions

```cpp
TEST(VtkExporter, ThrowsOnInvalidPath) {
    VtkExporter exporter;
    MeshData3D mesh;

    EXPECT_THROW({
        exporter.exportMesh(mesh, "/nonexistent/path/file.vtu");
    }, std::ios_base::failure);
}
```

## Migration Guide for New Code

When adding new code to cMesh, follow these guidelines:

### For Entity Lookups

```cpp
// ✅ GOOD - Throw if entity should exist (programming error if missing)
const Surface& getSurface(const std::string& id) const {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) {
        CMESH_THROW_ENTITY_NOT_FOUND("Surface", id);
    }
    return *it->second;
}

// ✅ GOOD - Return optional if "not found" is valid
std::optional<std::string> findSurfaceId(const Point3D& point) const {
    for (const auto& [id, surface] : surfaces_) {
        if (surface->contains(point)) {
            return id;
        }
    }
    return std::nullopt;
}
```

### For Validation

```cpp
// ✅ GOOD - Throw with specific error code
void addNode(const Point3D& point) {
    if (!isValidPoint(point)) {
        CMESH_THROW_CODE(cMesh::MeshException,
                        cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                        "Invalid point: coordinates must be finite");
    }
    nodes_.push_back(point);
}

// ✅ GOOD - Return structured result for non-throwing validation
VerificationResult validate() const {
    VerificationResult result;
    result.isValid = true;

    // Collect all errors
    if (!checkCondition1()) {
        result.errors.push_back("Condition 1 failed");
        result.isValid = false;
    }
    if (!checkCondition2()) {
        result.errors.push_back("Condition 2 failed");
        result.isValid = false;
    }

    return result;
}
```

### For Geometric Computations

```cpp
// ✅ GOOD - Return optional for legitimately failing computations
std::optional<Point2D> computeIntersection(const Line& l1, const Line& l2) const {
    double det = /* determinant */;

    if (std::abs(det) < 1e-10) {  // Parallel lines
        return std::nullopt;
    }

    return Point2D{x, y};
}
```

### For File I/O

```cpp
// ✅ GOOD - Enable stream exceptions
void writeToFile(const std::string& filePath) const {
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);  // Throws if fails

    os << data;
}
```

## Summary

cMesh's error handling strategy provides:

- **Type safety** through specific exception types
- **Clear semantics** distinguishing errors from valid absences
- **Rich context** with error codes, messages, and locations
- **Flexibility** for both throwing and non-throwing validation
- **Testability** with well-defined error conditions
- **Maintainability** through consistent patterns and helper macros

When in doubt:
- **Entity lookups that should succeed** → Exception
- **Queries where "not found" is valid** → `std::optional` or `bool`
- **Simple try operations** → `bool` return code
- **Operations with multiple failure modes** → Exception with error codes
- **Validation with error collection** → Structured result
- **File I/O** → Stream exceptions

**The Four-Way Decision:**
1. **Exception** - When failure is exceptional and needs context
2. **`std::optional<T>`** - When absence is valid and you want the full object
3. **`bool` return** - When it's a simple yes/no or success/failure
4. **Structured result** - When you need to collect multiple errors/warnings
