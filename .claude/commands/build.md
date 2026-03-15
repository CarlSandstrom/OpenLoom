---
description: Build the OpenLoom project
---

Build the OpenLoom project using CMake.

If build directory doesn't exist, configure first:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DOPENLOOM_USE_OPENMP=On
```

Then build:
```bash
cmake --build build -j$(nproc)
```

Report any compilation errors with file and line numbers.
