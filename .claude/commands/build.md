---
description: Build the cMesh project
---

Build the cMesh project using CMake.

If build directory doesn't exist, configure first:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```

Then build:
```bash
cmake --build build -j$(nproc)
```

Report any compilation errors with file and line numbers.
