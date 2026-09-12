---
description: C++ coding standards for OpenLoom — applies to all C++ source and header files
paths:
  - "**/*.cpp"
  - "**/*.cc"
  - "**/*.h"
  - "**/*.hpp"
---

# C++ Coding Standards for OpenLoom

**IMPORTANT: Follow these standards for ALL code edits and code reviews.**

## File Organization
- Headers: `.h` extension, sources: `.cpp` extension
- One class per file, use `#pragma once` for header guards
- Only declarations in headers, implementations in `.cpp` files
- No `using namespace` directives in headers (allowed in `.cpp` files)
- File-local `struct` types used purely as data containers are allowed in a `.cpp` file alongside the class

## Single Responsibility
- **One class, one job**: each class is responsible for exactly one thing
- Helper logic that is not the core responsibility of the class belongs in a separate, dedicated class — never as private methods or lambdas buried inside the primary class
- If splitting seems inconvenient, raise it with the user for discussion rather than bundling everything into one class
- Data classes (plain structs or simple value types) are the exception: they may live alongside other classes in the same file if they are scoped to that file and have no behavior of their own

## Naming Conventions
- **Classes**: `CamelCase` (e.g., `ConstrainedDelaunay2D`, `MeshData3D`)
- **Interfaces**: Start with capital `I` (e.g., `ICorner3D`, `IEdge3D`)
- **Variables**: `camelCase` (e.g., `nodeCount`, `triangleIndex`); where no more descriptive name exists, name the variable after its type in `camelCase` (e.g., a `GeometryCollection3D` instance → `geometryCollection3D`)
- **Member variables**: `camelCase_` with trailing underscore (e.g., `nodes_`, `connectivity_`); same naming-after-type rule applies (e.g., `geometryCollection3D_`)
- **Constants**: `ALL_CAPS` (e.g., `MAX_ITERATIONS`)
- **Getters**: `get...()` (e.g., `getNodes()`)
- **Setters**: `set...()` (e.g., `setTolerance()`)
- **No abbreviations**: always use full words in all identifiers, file names, and comments — e.g., `discretization` not `disc`, `triangulation` not `tri`, `parameterization` not `param`, `configuration` not `config`

## Functions & Methods
- Prefer `const` correctness (const methods, const references)
- Pass non-primitives by `const&` (e.g., `const Point3D&`)
- One declaration per line (no `int a, b, c;`)
- Use `auto` when it improves clarity
- **Length**: a function body should fit on one screen. Past ~60 lines it needs either a split or a comment stating why it must stay whole (a single algorithm whose steps cannot be named independently). For calibration, the median function in `src/` is 8 lines and 90% are under 50, so 60 is already generous.
- **How to split**: extract the step into a file-local `static` free function in the `.cpp`, or into a dedicated class if it carries state. Do **not** add a private method to the primary class just to shorten another one — that grows the class's declared job (see Single Responsibility above).
- **Parameters**: more than 5 suggests a missing parameter struct. If several are passed together through a chain of calls, they are one concept.
- **Nesting**: more than 3 levels inside a loop body is a split signal. Prefer early `continue`/`return` over an `else` arm.
- **One level of abstraction**: a function should not mix orchestration with arithmetic. If it both decides what to do and computes how, the computation is the part to extract.
- **Applies going forward**: ~46 existing functions exceed the length rule. They are a backlog, not a blocker — do not rewrite them as a batch. Apply the rule to new code and to a function you are already modifying for another reason, and split it as its own step under the Refactoring rules in `CLAUDE.md`.

## Classes & Access Control
- No public data members (use getters/setters)
  - Exception: `struct` types used as data containers may have public members
- Ownership via `std::unique_ptr` for heap-allocated objects
- Only use `static_cast` and `dynamic_cast` (no C-style casts)
- Prefer composition over inheritance
- If a class only has a simple method to execute code and the only a getter, or a getter and a release method, use a static class instead is possible.

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
