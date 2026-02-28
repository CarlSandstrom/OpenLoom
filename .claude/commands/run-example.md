---
description: Run a OpenLoom example
---

Run a OpenLoom example executable.

2D examples (in `build/src/Examples2D/`):
- SimpleDelaunay2D
- RectangleWithHole2D
- RectangleWithCrack
- SquareWithCircularHole
- SquareWithCircleAndTwinEdges
- SquareWithInternalCircles
- MeshStepFile2D

3D examples (in `build/src/Examples3D/`):
- BoxWithHole
- ConstrainedBoxExample
- CreateBox
- CylinderSurfaceMesh
- ShewchukBox
- ShewchukBoxWithHole
- SurfaceMeshEdges

If $ARGUMENTS is provided, determine whether it is a 2D or 3D example and run it from the correct directory:
```bash
./build/src/Examples2D/$ARGUMENTS   # for 2D examples
./build/src/Examples3D/$ARGUMENTS   # for 3D examples
```

If no argument provided, list available examples and ask which one to run.

After running, mention that output can be viewed with: `paraview output.vtu`
