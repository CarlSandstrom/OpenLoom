# Coding Standards

## Naming Conventions

**Files & Classes**
- Clear, descriptive names in English
- Headers: `.h` | Source files: `.cpp`
- Class names: `CamelCase` (e.g., `MeshGenerator`)
- One class per file

**Functions & Variables**
- Setters: `set...()` | Getters: `get...()`
- Variables: `camelCase` (e.g., `nodeCount`)
- Member variables: `camelCase_` with trailing underscore (e.g., `meshData_`)
- Constants: `ALL_CAPS` (e.g., `MAX_ITERATIONS`)

## Code Structure

**Headers**
- Use `#pragma once`
- Forward declarations preferred over includes
- No `using namespace` directives

**Classes**
- No public or protected member variables
- No friend classes
- Use `std::unique_ptr` for ownership management
- Only `static_cast` and `dynamic_cast` allowed

**Functions**
- Pass non-primitives by `const` reference when possible
- Apply `const` to member functions and arguments where applicable
- One declaration per line

## Best Practices

- Use `auto` for type deduction whenever practical
- Wrap namespace usage in braces: `namespace X { ... }`
- Apply code formatter consistently