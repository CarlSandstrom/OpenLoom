# Tidy checklist

Every class under `src/`, one row per header/source pair, grouped by directory. Work through it with `/tidy <ClassName>`; tick the row when the result is green and committed. Line counts are `.h` / source.

`[x]` marks classes already tidied (from the git history). `WeightedDualRestriction`, `SurfaceDelaunayRefiner` and `MeshConnectivity` had only a partial pass, so they stay open.


119 of 151 done.


## `Common/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `BoundingBox2D` | 23 | 34 |
| [x] | `DebugFlags` | 50 | - |
| [x] | `Logging` | 15 | - |
| [x] | `MathConstants` | 6 | - |
| [x] | `TwinManager2D` | 75 | 38 |
| [x] | `Types` | 19 | - |

## `Common/Exceptions/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `Exception` | 77 | - |
| [ ] | `GeometryException` | 112 | - |
| [ ] | `MeshException` | 159 | - |
| [ ] | `TopologyException` | 49 | - |

## `Examples/2D/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `FullyPeriodicSquare` | - | 273 |
| [x] | `ManyCirclesInSquare` | - | 189 |
| [x] | `MeshStepFile2D` | - | 60 |
| [x] | `NearCoincidentConstraintEdges` | - | 110 |
| [ ] | `NestedConcentricRings` | - | 178 |
| [ ] | `RectangleWithCrack` | - | 97 |
| [ ] | `RectangleWithHole2D` | - | 104 |
| [x] | `SimpleDelaunay2D` | - | 53 |
| [ ] | `SpiralChannel` | - | 143 |
| [ ] | `SquareWithCircleAndTwinEdges` | - | 251 |
| [ ] | `SquareWithCircularHole` | - | 161 |
| [ ] | `SquareWithInternalCircles` | - | 157 |
| [ ] | `StarPolygon` | - | 97 |
| [ ] | `ThinSliverDomain` | - | 112 |

## `Examples/3D/Surface/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `BoxWithHoleSurface` | - | 76 |
| [x] | `CylinderSurfaceMesh` | - | 83 |
| [x] | `HexNutChamferedSurfaceMesh` | - | 149 |
| [x] | `HexNutSurfaceMesh` | - | 124 |
| [x] | `MeshStepFile3DSurface` | - | 58 |
| [x] | `SaddleSurfaceMesh` | - | 257 |
| [x] | `SharpCreaseBracket` | - | 173 |
| [x] | `ThinFinSurfaceMesh` | - | 72 |
| [x] | `TorusSurfaceMesh` | - | 71 |

## `Examples/3D/Volume/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `BoxWithHole` | - | 71 |
| [x] | `CreateBox` | - | 49 |
| [x] | `MeshStepFile3DVolume` | - | 60 |

## `Export/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `IExporter` | 29 | - |
| [ ] | `TsvExporter` | 56 | 316 |
| [ ] | `VtkExporter` | 63 | 597 |
| [ ] | `VtkGrid` | 57 | - |

## `Geometry/2D/Base/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `Corner2D` | 29 | 12 |
| [x] | `DiscretizationSettings2D` | 45 | - |
| [x] | `GeometryCollection2D` | 40 | 47 |
| [x] | `ICorner2D` | 22 | - |
| [x] | `IEdge2D` | 30 | - |
| [x] | `LinearEdge2D` | 38 | 32 |

## `Geometry/2D/OpenCascade/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `OpenCascade2DCorner` | 28 | 31 |
| [x] | `OpenCascade2DEdge` | 36 | 69 |

## `Geometry/3D/Base/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `DiscretizationSettings3D` | 73 | - |
| [x] | `GeometryCollection3D` | 39 | 48 |
| [x] | `ICorner3D` | 22 | - |
| [x] | `IEdge3D` | 39 | - |
| [x] | `ISurface3D` | 101 | 22 |
| [x] | `IVolume3D` | 24 | - |

## `Geometry/3D/OpenCascade/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `OpenCascadeCorner` | 24 | 29 |
| [x] | `OpenCascadeEdge` | 32 | 136 |
| [x] | `OpenCascadeSurface` | 69 | 221 |
| [x] | `OpenCascadeVolume` | 23 | 21 |

## `Meshing/Connectivity/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `EdgeKey` | 48 | - |
| [ ] | `FaceKey` | 79 | - |
| [ ] | `TetrahedronKey` | 66 | - |

## `Meshing/Core/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `ConstraintStructures` | 45 | - |

## `Meshing/Core/2D/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `BoundaryDiscretizer2D` | 48 | 153 |
| [x] | `BoundarySplitSynchronizer2D` | 51 | 46 |
| [x] | `ConstrainedDelaunay2D` | 39 | 74 |
| [x] | `ConstraintChecker2D` | 33 | 36 |
| [x] | `Delaunay2D` | 26 | 56 |
| [x] | `ElementGeometry2D` | 76 | 149 |
| [x] | `ElementQuality2D` | 38 | 76 |
| [x] | `GeometryStructures2D` | 12 | - |
| [x] | `GeometryUtilities2D` | 64 | 205 |
| [x] | `MeshDebugUtilities2D` | 27 | 37 |
| [x] | `MeshOperations2D` | 71 | 709 |
| [x] | `MeshQueries2D` | 91 | 615 |
| [x] | `MeshVerifier2D` | 47 | 184 |
| [x] | `MeshingContext2D` | 75 | 73 |
| [x] | `PeriodicDomain2D` | 36 | - |
| [x] | `PeriodicMeshData2D` | 58 | 82 |
| [x] | `PeriodicOffsetTable2D` | 44 | 50 |
| [x] | `ShewchukQualityController2D` | 34 | 71 |
| [x] | `ShewchukRefiner2D` | 103 | 309 |

## `Meshing/Core/3D/General/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `BoundaryDiscretizer3D` | 78 | 344 |
| [x] | `ElementGeometry3D` | 38 | 80 |
| [x] | `ExactArithmetic3D` | 74 | 132 |
| [x] | `LocalFeatureSize3D` | 106 | 292 |
| [x] | `MeshOperations3D` | 168 | 377 |
| [x] | `MeshQueries3D` | 83 | 152 |
| [x] | `MeshSnapshotWriter` | 51 | 96 |
| [x] | `MeshingContext3D` | 76 | 77 |
| [x] | `RegularPredicates3D` | 61 | 248 |
| [x] | `RobustPredicates3D` | 78 | 108 |
| [x] | `SizingField3D` | 79 | 55 |
| [x] | `SizingFieldBuilder3D` | 88 | 203 |

## `Meshing/Core/3D/RCDT/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `AmbientTetrahedronClassifier` | 46 | 71 |
| [x] | `AmbientTetrahedronRemover` | 26 | 43 |
| [x] | `CurveSegmentBuilder` | 45 | 96 |
| [x] | `MinimumEdgeLengthEstimator` | 34 | 67 |
| [x] | `ProtectingBallPlacer` | 62 | 562 |
| [x] | `ProtectionExemption` | 28 | 102 |
| [x] | `RCDTMeshExtractor` | 47 | 115 |
| [x] | `RCDTMesher` | 85 | 284 |
| [x] | `RegularConflictRegion` | 45 | 86 |
| [x] | `RestrictedFaceAudit` | 80 | 165 |
| [x] | `RestrictedFaceTypes` | 51 | - |
| [x] | `SnapshotGridBuilder` | 53 | 139 |
| [x] | `SurfaceDelaunayRefiner` | 136 | 265 |
| [x] | `SurfaceFacetCriteria` | 54 | 96 |
| [x] | `SurfaceMeshSmoother` | 53 | 138 |
| [x] | `SurfaceProjector` | 65 | 127 |
| [x] | `SurfaceTessellation` | 61 | 252 |
| [x] | `TetrahedronDelaunayRefiner` | 71 | 153 |
| [x] | `TetrahedronInversionGuard` | 46 | 69 |
| [x] | `TriangleSoupIndex` | 88 | 214 |
| [x] | `WeightedDualRestriction` | 81 | 146 |

## `Meshing/Core/3D/Surface/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `SurfaceMesher3D` | 73 | 36 |

## `Meshing/Core/3D/Volume/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `Delaunay3D` | 67 | 55 |
| [x] | `VolumeMesher3D` | 74 | 36 |

## `Meshing/Data/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `ConstraintRole` | 12 | - |
| [ ] | `CurveSegmentManager` | 61 | 123 |

## `Meshing/Data/2D/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `DiscretizationResult2D` | 35 | - |
| [x] | `MeshQualitySettings2D` | 27 | - |
| [x] | `MeshData2D` | 53 | 56 |
| [x] | `MeshMutator2D` | 37 | 50 |
| [x] | `Node2D` | 19 | 11 |
| [x] | `TriangleElement` | 30 | 57 |

## `Meshing/Data/3D/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `DiscretizationResult3D` | 38 | - |
| [x] | `MeshData3D` | 65 | 105 |
| [x] | `MeshMutator3D` | 55 | 133 |
| [x] | `Node3D` | 30 | 31 |
| [x] | `SurfaceMesh3D` | 37 | - |
| [x] | `SurfaceMesh3DQualitySettings` | 53 | - |
| [x] | `TetrahedronElement` | 27 | 49 |
| [x] | `VolumeMesh3D` | 43 | - |

## `Meshing/Data/Base/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `IElement` | 30 | - |
| [ ] | `MeshConnectivity` | 36 | 110 |

## `Meshing/Interfaces/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `IQualityController2D` | 24 | - |
| [ ] | `ISurfaceMesher3D` | 24 | - |
| [ ] | `IVolumeMesher3D` | 24 | - |

## `Readers/OpenCascade/`

| Done | Class | .h | source |
|---|---|---|---|
| [ ] | `OpenCascadeGeometryCollection` | 44 | 130 |
| [ ] | `StepReader2D` | 56 | 174 |
| [ ] | `StepReader3D` | 27 | 30 |
| [ ] | `TopoDS_ShapeConverter` | 47 | 348 |

## `Topology/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `Corner3D` | 22 | 15 |
| [x] | `Edge3D` | 28 | 32 |
| [x] | `SeamCollection` | 44 | 43 |
| [x] | `Surface3D` | 32 | 34 |
| [x] | `Topology3D` | 43 | 80 |
| [x] | `Volume3D` | 22 | 15 |

## `Topology2D/`

| Done | Class | .h | source |
|---|---|---|---|
| [x] | `Corner2D` | 27 | 12 |
| [x] | `Edge2D` | 30 | 15 |
| [x] | `Topology2D` | 50 | 49 |
