---
description: Run a OpenLoom example
---

Run a OpenLoom example executable.

Available examples:
- SimpleDelaunay2D
- RectangleWithHole2D
- RectangleWithCrack
- SquareWithCircularHole
- BoxWithHole
- ConstrainedBoxExample
- CreateBox

If $ARGUMENTS is provided, run that example:
```bash
./build/src/Examples/$ARGUMENTS
```

If no argument provided, list available examples and ask which one to run.

After running, mention that output can be viewed with: `paraview output.vtu`
