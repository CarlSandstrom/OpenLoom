---
name: cpp-conventions
description: C++ coding conventions for cMesh
globs:
  - "**/*.cpp"
  - "**/*.hpp"
  - "**/*.h"
  - "**/*.cc"
---

# C++ Conventions for cMesh

## File Organization
- Headers: `.h` with `#pragma once`
- Sources: `.cpp` or `.cc`
- One class per file
- Include paths relative to `src/` (e.g., `#include "Meshing/Data/MeshData2D.h"`)

## Naming
- Classes: `CamelCase` (e.g., `ConstrainedDelaunay2D`)
- Interfaces: Prefix with `I` (e.g., `ICorner3D`, `IMesher`)
- Variables: `camelCase`
- Members: `camelCase_` with trailing underscore
- Constants: `ALL_CAPS`
- Getters: `get...()`, Setters: `set...()`

## Class Design
- No public data members (except data-only structs like `CircumCircle2D`)
- Use `std::unique_ptr` for heap-allocated objects
- Use `explicit` on single-argument constructors
- Default constructors: `= default` when appropriate
- Virtual base class destructors: `virtual ~ClassName() = default`

## Functions
- Pass non-primitives by `const&`
- Use `const` correctness throughout
- Use `auto` when it improves clarity
- Only `static_cast` and `dynamic_cast` (no C-style casts)

## Formatting
- Microsoft style, 4-space indentation
- Allman braces (braces on new lines)
- No namespace indentation
