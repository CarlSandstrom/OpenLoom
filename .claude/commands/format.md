---
description: Format C++ source files
---

Format C++ files using clang-format.

If $ARGUMENTS is provided, format that specific file:
```bash
clang-format -i $ARGUMENTS
```

If no argument, format all changed files:
```bash
git diff --name-only --diff-filter=d | grep -E '\.(cpp|hpp|h|cc)$' | xargs -r clang-format -i
```

Report which files were formatted.
