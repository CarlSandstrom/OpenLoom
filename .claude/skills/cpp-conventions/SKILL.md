---
name: cpp-conventions
description: OpenLoom C++ house style - naming (camelCase_ members, I-prefixed interfaces), file layout, class design rules, const correctness, cast policy, and Microsoft/Allman formatting. Load before writing or reviewing any .h/.cpp/.cc file in this project.
---

# C++ Conventions for OpenLoom

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
- No public data members (except data-only structs like `Circle2D`)
- Use `std::unique_ptr` for heap-allocated objects
- Use `explicit` on single-argument constructors
- Default constructors: `= default` when appropriate
- Virtual base class destructors: `virtual ~ClassName() = default`

## Functions
- Pass non-primitives by `const&`
- Use `const` correctness throughout
- Use `auto` when it improves clarity
- Only `static_cast` and `dynamic_cast` (no C-style casts)
- Body fits on one screen; past ~60 lines, split it or state why it must stay whole
- Split into a file-local `static` free function in the `.cpp`, not a new private method on the class
- More than 3 levels of nesting in a loop body is a split signal

See `.claude/rules/cpp-coding-standards.md` for the full standard.

## Formatting
- Microsoft style, 4-space indentation
- Allman braces (braces on new lines)
- No namespace indentation
