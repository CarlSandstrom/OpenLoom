---
globs:
  - "**/*.cpp"
  - "**/*.cc"
  - "**/*.h"
  - "**/*.hpp"
---

# C++ Coding Standards for OpenLoom

**IMPORTANT: Follow these standards for ALL code edits.**

## File Organization
- Headers: `.h` extension, sources: `.cpp` extension
- One class per file, use `#pragma once` for header guards
- Only declarations in headers, implementations in `.cpp` files
- No `using namespace` directives in headers (allowed in `.cpp` files)

## Naming Conventions
- **Classes**: `CamelCase` (e.g., `ConstrainedDelaunay2D`, `MeshData3D`)
- **Interfaces**: Start with capital `I` (e.g., `ICorner3D`, `IEdge3D`)
- **Variables**: `camelCase` (e.g., `nodeCount`, `triangleIndex`)
- **Member variables**: `camelCase_` with trailing underscore (e.g., `nodes_`, `connectivity_`)
- **Constants**: `ALL_CAPS` (e.g., `MAX_ITERATIONS`)
- **Getters**: `get...()` (e.g., `getNodes()`)
- **Setters**: `set...()` (e.g., `setTolerance()`)

## Functions & Methods
- Prefer `const` correctness (const methods, const references)
- Pass non-primitives by `const&` (e.g., `const Point3D&`)
- One declaration per line (no `int a, b, c;`)
- Use `auto` when it improves clarity

## Classes & Access Control
- No public data members (use getters/setters)
  - Exception: `struct` types used as data containers may have public members
- Ownership via `std::unique_ptr` for heap-allocated objects
- Only use `static_cast` and `dynamic_cast` (no C-style casts)
- Prefer composition over inheritance

## Constructors & Initialization
- Use `explicit` on single-argument constructors
- Member initializer lists: one member per line
- Default constructors: use `= default` when appropriate
- Destructors: use `= default` for simple classes, `virtual ~Class() = default` for base classes

## Headers & Includes
- Include paths relative to `src/` (e.g., `#include "Meshing/Data/MeshData2D.h"`)
- Favor forward declarations in headers
- Include concrete headers in `.cpp` files only
- **Include order**: Own header first (for .cpp), then all others alphabetically

## Comments
- Keep comments minimal - code should be self-documenting
- Use comments only for non-obvious logic or algorithmic decisions

## Error Handling
- Use custom exceptions from `Common/Exceptions/`:
  - `OPENLOOM_THROW_ENTITY_NOT_FOUND("Type", id)` for entity lookups
  - `OPENLOOM_REQUIRE_NOT_NULL(ptr, "name")` for null checks
  - `OPENLOOM_THROW_CODE(ExceptionType, ErrorCode::CODE, "msg")` for failures
- Use `std::optional<T>` for operations that may produce no result
- Use `bool` for simple success/failure
- Never use generic `std::runtime_error`

## Data Access Patterns
- Access mesh data through contexts (`MeshingContext2D/3D`)
- Never cache raw pointers to mesh data
- Use `MeshMutator2D/3D` for mutations
- Rebuild connectivity after bulk operations
