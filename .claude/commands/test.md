---
description: Run cMesh tests
---

Run the cMesh test suite.

First ensure the project is built, then run tests:
```bash
ctest --test-dir build/tests --output-on-failure
```

If $ARGUMENTS is provided, use it as a gtest filter:
```bash
./build/tests/runTests --gtest_filter="$ARGUMENTS"
```

Report test results clearly, highlighting any failures with details.
