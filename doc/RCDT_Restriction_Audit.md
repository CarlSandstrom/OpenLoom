# RCDT restriction/repair audit (OPE-208)

Status: **inventory complete; the ablation was deliberately not run.** See
[What was decided, and what was done](#what-was-decided-and-what-was-done) at
the end for why, and for the state of every row that changed.

Unless a cell says otherwise, each "measured" figure cites an existing
measurement from a commit message, a code comment or a project memory rather
than a new experiment. The three numbers this audit produced itself are
called out where they appear (rows 1.11, 1.14, and the closure figures in the
closing section).

Scope, per the ticket: `DualEdgeRestrictionOracle`, `RestrictedFaceAudit`,
`SurfaceCandidates`, `CurveProtectionScheme` / `CurveProtectionSubdivider`,
`MinimumEdgeLengthEstimator`, `NonManifoldEdgeRefiner`. `SurfaceTessellation`
is included where it supplies the oracle's crossing test, because two of the
oracle's constants live there. `RCDTPointInserter`'s guards are listed at the
end as *adjacent*: they are out of scope to change, but they are what actually
gates priority 4, so priority 4 cannot be judged without them.

## How to read the bucket column

| bucket | definition |
| -- | -- |
| **A — theory-bearing** | Implements a definition or theorem with checkable preconditions. When it fails you can name the violated precondition. |
| **B — compensation** | Exists because something in A is unreliable. Legitimate, but must name the mechanism it compensates for **and a retirement condition**. |
| **C — fitted** | A constant or rule chosen because it moved a benchmark number, with no derivation. |

A row marked **A/C** is a mechanism whose *constraint* is derived but whose
*value* is not: `DISJOINT_FACTOR < 0.5` is a theorem, `= 0.45` is a choice.
Those two halves retire separately and are recorded as one row to keep the
derivation attached to the number it justifies.

The `retirement condition` column is the deliverable. Everywhere it says
"none recorded", that is the gap this ticket exists to close.

---

## 1. DualEdgeRestrictionOracle

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 1.1 | **Dual-edge crossing test** (`crossesSurface` on the segment between the two adjacent tets' circumcenters) | The restricted-Delaunay definition itself: a face is on the boundary iff its dual Voronoi edge crosses the surface | Textbook RCDT; `2d221ee` (OPE-169) replaced a tolerance-based surface test with the exact tessellation crossing | **A, precondition known violated** | Retires with OPE-186. Its precondition — well-shaped tets, i.e. an ε-sample — is measured false on ~90% of faces (89.8% exceed the bad-tet threshold; `meshSurface()` runs `runPipeline(false)`). This is the one row where the violated precondition is already named. |
| 1.2 | **`verticesWithinTrimmedBoundary`** gate (all three vertices project inside the trimmed patch) | Necessary condition before any surface can be a candidate | `2d221ee` | **A** | Never — a face cannot be restricted to a patch none of its vertices lies on. Note it is *not local*: >95% of protected-edge candidates pass it from unrelated points elsewhere on the same patch (measured, OPE-176). That non-locality is why 1.5 and 1.7 exist. |
| 1.3 | **Convex-hull face route** (one adjacent tet → accept on vertex-in-trim alone) | Handles the half-infinite dual ray, which has no second circumcenter | none recorded | **A, precondition unchecked** | The comment asserts the ray "always crosses the surface when the circumcenter is on the interior side, which is guaranteed for Delaunay triangulations of boundary points on a closed solid". That guarantee is stated, never checked, and the ambient triangulation contains supertet nodes that are not boundary points. Retires when the replacement oracle has no circumcenter-dependent special case. **Probe: how many faces take this route on a real model?** |
| 1.4 | **Protected-edge shortcut** (`candidates.size()==1` + a curve-chain-adjacent edge + 1.5) | Accepts a face the dual-edge test declines, when a protecting ball certifies its edge belongs to exactly one crease | `0de31c8` (OPE-176), part of 863→126→84 | **B** — compensates for 1.1's false negatives at creases | Retires when the oracle no longer returns false negatives on near-degenerate crease faces. Compensates 1.1 specifically, not the classifier generally. |
| 1.5 | **`isUniqueEdgeStarCandidate`** — 0 rivals allowed, rivals judged by their *own* `crossesSurface()` | Narrows 1.4 to the case it was built for by requiring this face to be the only candidate across the whole edge star | Trusting a single-candidate protected face without it accepted spurious faces from elsewhere in the tet ring (tried and reverted, OPE-176) | **B** — a guard on a guard | Retires with 1.4. The "rivals judged by their own crossing test" choice is *derived*, not fitted: vertex-in-trim alone is non-local (see 1.2), so it would reject almost everything. |
| 1.6 | **Phase-boundary test** `isPhaseBoundaryFace` (both adjacent tets' **centroids** classified against every `IVolume3D`; accept when they differ) | The interface definition: a face is boundary iff its two tets are in different regions | `63e23b4` (OPE-176), 84 → 28 | **A** — and the most important row in this table | Does **not** retire with OPE-186 — it *is* OPE-186's proposed direction (region labelling from centroids, interface = faces whose tets differ). Today it is demoted to a guarded shortcut inside a B-layer. Its precondition is checkable and it declines honestly when violated: both centroids must be definitive. OPE-187's root cause is that decline, falling through to 1.1 which answers wrongly. |
| 1.7 | **`isUniquePhaseBoundaryCandidate`** — **at most one** rival | Locality guard on 1.6 | `63e23b4`: barring all rivals measured zero improvement | **B, derived — CLOSED `4e494cd`** | The "why one?" **is** answered, in `63e23b4`'s commit message and nowhere in the code: an ordinary interior edge's two genuine incident faces are on the *same* surface, so a face's own genuine partner is exactly one rival. 0 rivals would disqualify every legitimate interior face. The derivation is **now in the code**. Retires when parity around the edge cycle replaces rival counting. |
| 1.8 | **`periodicSurfaceIds_`** — skip 1.6/1.7 entirely on seam surfaces | Scoped carve-out for periodic surfaces | `63e23b4`: without it, unbounded refinement growth on `RCDTMesherCylinderTest`; attributed to a small persistent misclassification rate on seam surfaces, root cause **not found** | **B, self-confessed** ("a scoped safety net, not a fix … Root cause not yet found") | Retires when the seam misclassification stream is root-caused, **or** when 1.6 becomes primary and its seam behaviour is re-measured. Highest-value row to re-test: it was measured 2026-08-16, before OPE-184's pruning, OPE-207's fix and OPE-182's caching. |
| 1.9 | **`TESSELLATION_CELL_SIZE_FACTOR = 0.5`** | Tessellation cell size = 0.5 × `minimumEdgeLength` | `297d244` (OPE-172) | **C — accepted as is** | The comment claims cells below `minimumEdgeLength/2` are "guaranteed fine enough to classify any face whose shortest edge is at or above that floor". That is an assertion with no derivation and no stated failure mode. Retires when the crossing test no longer needs a discrete proxy, or when the claim is turned into a checkable statement. Cost is quadratic in the factor, so this is not free. |
| 1.10 | **`GRID_JITTER_U = 0.37`, `GRID_JITTER_V = 0.61`** | Offsets the UV sample grid so a crossing point rarely coincides with a grid vertex or a cell diagonal | OPE-169: exact predicates *correctly* reject a crossing that touches a vertex, and CAD geometry lands on simple fractions constantly | **C, accepted** | The *requirement* is derived and checkable (not a simple fraction; U ≠ V); the two values are arbitrary within it. Retires only if the crossing query stops being a triangle-soup query. Recommend: accept explicitly, record the requirement, do not ablate. |
| 1.11 | **`MAXIMUM_SAMPLES_PER_DIRECTION = 400`** | Caps tessellation memory/cost | none recorded | **C — CLOSED `9c8cca1`** | Was: silently clamped, voiding 1.9's guarantee with no diagnostic. Retirement condition was "show it never binds, or log when it does". **It now logs, and it binds** — `HexNutChamferedSurfaceMesh` asks 562 columns on two surfaces and gets 400 (cells 0.0496 vs a 0.0354 target, 40% coarse). Every other example is clear of it. Kept as a cost cap, no longer a silent one. See the closing section for the open causal question. |
| 1.12 | **Circumcenter preferred over bounding-node substitution** in `computeDualEdgeEndpoint` | Uses the real circumcenter even for supertet-touching tets; falls back to the bounding node's coordinates only if the circumsphere solve fails | OPE-169: substituting the bounding node shifted crossing points away from the face under test | **A** | Retires with 1.1 — it exists only because dual edges do. |
| 1.13 | **`centroidPhaseByTetrahedron_` / `nodeWithinTrimmedBoundaryBySurface_` caches** | Memoization, keyed on node sets (not element ids) | OPE-182: `classifyPointPhase` was 85% of runtime; saddle 511 s → 50 s, **byte-identical output** | **A** — performance, not compensation | Not a candidate for this audit. Listed so it is not mistaken for one. Depends on the fixed-node-coordinates invariant; that invariant is documented. |
| 1.14 | **`FaceRestriction::Unconfirmed`** | A third state distinguishing "no surface" from "no answer" | its own doc: "lets a classification failure be counted rather than silently becoming a missing face" | **was C (dead as built) — now A, CLOSED `4e494cd`** | Was produced at four sites and consumed nowhere; every caller tested only `== Restricted`, so the failures it exists to make countable were counted by nothing. `buildFrom()` now collects the distinct unconfirmed faces and `runPipeline` logs the total. First numbers it produced: cylinder 76 restricted / **78 unconfirmed**, hexnut-chamfered 410 / **494**. An upper bound on real misses, not a defect count — but it is the classification's own error signal, which is what OPE-186 needs to compare oracles on. |

## 2. RestrictedFaceAudit

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 2.1 | **Topology-derived expected count** in `findNonManifoldEdges` | Reads per-edge expected face count off `Edge3D::getAdjacentSurfaceIds()` instead of assuming 2 | `df3aafe` (OPE-184) | **A** | Never. It is a genuine improvement on the manifold assumption it replaced: a triple line in a multi-material model legitimately carries 3+. Regression-tested (`RemoveDefectiveFaces_LeavesAGenuineTripleLineAlone`). |
| 2.2 | **Off-curve rule: expect 2 faces on one surface**; when the incident faces disagree about the surface, expect the count only | Supplies an expectation where the topology fixes none | none recorded | **A/B** | The count-2 half is the surface-interior manifold condition (A). The "when they disagree, give up on the surface" half is a reporting choice that turns into a `SurfaceMismatch` — which is the defect class nobody has investigated (2 on every stock-density saddle run, invariant to the floor). Retirement: once `SurfaceMismatch` is investigated, this either becomes a real expectation or stays a documented can't-tell. |
| 2.3 | **`removeChordFaces`** | Post-hoc removal of faces using a same-curve chord edge (endpoints on one curve but not chain-adjacent) | `5b1e997` (OPE-176). Rejecting chord faces *during* classification regressed 28 → 69; refinement supersedes most of them on its own (~24 at `buildInitial` vs 15 residual) | **B** — compensates for 1.1 accepting chord faces | Retires when the oracle cannot accept a chord face. Note the replacement direction makes this *representable* still, so this is one OPE-186 probably must subsume, not drop. |
| 2.4 | **Chord-face safety rule**: drop only while no non-chord edge is at exactly 2 | Stops a chord removal tearing a hole in healthy surface | `e4820ed` (OPE-176): the unconditional version "tore fresh holes … trading duplicates here for gaps there" | **B** — a guard on 2.3 | Retires with 2.3. |
| 2.5 | **`removeExcessFaces`** — prune over-acceptance flaps as whole connected components | Post-hoc manifold enforcement; over-covered edges act as cuts so a flap falls out as its own component | `df3aafe` (OPE-184): 17 → 1, 65 → 2, 146 → 0, 134 → 1 | **B** — compensates for 1.1 over-accepting in surface interiors | Retires when the restricted set cannot contain a flap by construction (the region-labelling direction makes `ExcessFace` unrepresentable). The component-based identification is derived, not fitted — per-edge "keep the best 2" cannot tell a flap from a triple line. |
| 2.6 | **"never remove the largest component"** | Stops the pass eating the surface | none recorded | **C** — heuristic riding along in an otherwise A/B pass | A flap is small *on the models tested*. Nothing rules out a defect larger than the sheet on a model with several disconnected surface components — where the largest component is not privileged at all. Retirement: replace with a criterion that does not depend on relative size, or state the model class it assumes. |
| 2.7 | **"smallest component first"** | Ordering, so a large component is not judged against counts a smaller one has been credited with | none recorded | **C** | Same as 2.6. The *determinism* half (ties on smallest `FaceKey`) is A — without it the mesh depends on hash order. The *smallest-first* half is a heuristic. |
| 2.8 | **Greedy to a fixed point** (both passes) | Removal changes the counts remaining candidates are judged against | ordering-dependence argument, stated in both comments | **A** | Never — this is an order-independence requirement, not a tuning knob. |
| 2.9 | **Seam curve counted twice** in the expected multiset | A seam bounds its surface's UV domain on both sides, so it expects 2 same-surface faces | none recorded, but stated and load-bearing | **A** | Never, while seams exist in the topology. Related to 1.8: both are seam special cases, reached from opposite directions. |

## 3. SurfaceCandidates

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 3.1 | **geometry-id → surface resolution** (surface → itself, curve → surfaces it bounds, corner → surfaces meeting there) | Resolves what a node's tag means before any face can be classified | — | **A** | Never. |
| 3.2 | **Intersection across the face's three nodes** | A face can only lie on a surface all three corners lie on | — | **A** | Never. |

**This module contains no constants, no compensation and no fitted rules.** It
is the control case for what the rest of the table should look like.

## 4. CurveProtectionScheme / CurveProtectionSubdivider

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 4.1 | **`INTERIOR_FACTOR = 0.6`** — radius = 0.6 × *longer* adjacent segment | Property 1: consecutive balls on one curve overlap unconditionally, whatever the sampling non-uniformity | Derivation in the comment: both endpoints' radii are ≥ f·L for the shared segment, so > 0.5 forces overlap | **A/C** — `> 0.5` derived, `0.6` chosen | The value has a known cost: 0.6 > 0.5 makes a segment midpoint always fall inside its own endpoints' balls, which is why **priority 1 has been dead since `84fe2db`** (OPE-180). Retirement of the *value*: a sizing field that couples radius to spacing (CGAL `insert_balls`). Retirement of the *constraint*: never. |
| 4.2 | **`CORNER_FACTOR = 0.3`** — corner radius from the *shortest* incident first step | Corners get the smallest ("strong") balls so curve radii shrink toward them | "kept below `INTERIOR_FACTOR`", "strictly less than 1" | **C** | The bounds are derived; 0.3 is not. OPE-181 measured the companion finding: sizing a corner from its *longest* incident step hides the finest curve's first points (that was the 697). The *shortest* rule is A; the 0.3 is C. |
| 4.3 | **`CORNER_OVERLAP_SLACK = 1.1`** | Lets a corner-adjacent interior point bridge to the corner's final radius in one jump | Full algebraic derivation in the comment; **OPE-181 stage 2 proved it is a geometric invariant, not debt** (the wholesale replacement had to reimplement it under a new name) | **A/C** — `> 1` derived, `1.1` chosen | Do not delete. The ticket's own "known starting points" list does not include it; OPE-181's disproven acceptance signal ("if `CORNER_OVERLAP_SLACK` survives, the change didn't land") is recorded as **disproven** and must not be revived. |
| 4.4 | **`DISJOINT_FACTOR = 0.45`** | Property 2: clamp each radius to 0.45 × distance to the nearest unrelated point | Derivation in the comment: `< 0.5` makes any unrelated pair's radii sum below their separation | **A/C** — `< 0.5` derived, `0.45` chosen | Constraint never retires. The value's only content is headroom below 0.5. |
| 4.5 | **`CORNER_DILUTION_THRESHOLD = 0.5`** | Trusts 4.3's compensation only while the corner's pooled minimum first step is ≥ 0.5 × this edge's own first step | `2e529f0` (OPE-176): without a bound, a corner starved by a finer sibling inflated a neighbour's ball past the local scale (torus radius 2.64 vs minor radius 1.5) | **C** — mechanism derived, threshold not | The failure it prevents is real and named. 0.5 is not derived. Retirement: a sizing field makes "is this edge the one that set the corner's radius?" unnecessary, because both curves read the same `h`. |
| 4.6 | **Union-find relatedness over shared corners** | Two curves sharing a corner are one feature for property 2 | `d3efd31` (OPE-176): per-edge relatedness produced a runaway split loop with no fixed point on the torus | **A** | Never — it is a correctness requirement, not a tuning choice. |
| 4.7 | **Clamp against non-feature points too** | A coarse edge's legitimate ball must not swallow a face-interior sample | `52fdc46` (OPE-176): confirmed on the saddle's flat bottom face, orphaned points during the first triangulation | **A** | Never while face-interior samples exist. |
| 4.8 | **`MAX_SUBDIVISION_ITERATIONS = 10000`** | Bounds the subdivision loop | none recorded | **C** — termination safety device | Same class as `minimumEdgeLength`: a safety device, not a physical quantity. Retirement: show it never binds, or that binding is reported rather than silent. |
| 4.9 | **Subdivider stops at `minimumEdgeLength`** | A conflict that survives to the finest useful scale is left as a logged gap | stated: same "leave it, don't retry forever" guarantee as the refiner's unrefinable sets | **B** | Retires when property-1/property-2 conflicts cannot arise — i.e. when points and radii are emitted together from one field. |

## 5. MinimumEdgeLengthEstimator

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 5.1 | **`fromPointSpacing` = median nearest-neighbour ÷ 10** | The default floor when the caller sets none | *median* rather than minimum is derived (a periodic curve's seam remainder segment is unrepresentative). **The ÷ 10 has no derivation anywhere.** | **C** | The ÷ 10 is the single most load-bearing undocumented number in the module: it sets the refinement floor, the subdivider's stopping size, *and* (via 1.9) the tessellation oracle's resolution — three unrelated jobs from one constant. Retirement: separate the three consumers, then derive each. OPE-181 measured that setting it *at* `h_min` collapses the torus, so it cannot simply be removed. |
| 5.2 | **`fromSizingField` = 10th-percentile source size ÷ 10** | The floor when `h(x)` is available | `9d32abf`, 2026-09-13, chosen from: percentile 0.05 → 1 defect, 0.10 → 1, 0.25 → 7, 0.50 → 12, global minimum → 9 | **C — annotated, `4e494cd`** | **Flagging honestly: this constant was selected ten days ago by sweeping a parameter and reading off non-manifold defect counts — the exact method this ticket forbids.** That does not make the value wrong; the reasoning about outliers is sound and independent. It makes the *evidence* the wrong kind. Retirement: re-derive the percentile against a property of the field (e.g. the fraction of sources that are sampling artifacts) rather than a defect count. |
| 5.3 | **Shared `MINIMUM_EDGE_LENGTH_DIVISOR = 10.0`** across both estimators | — | header: the floor is a **sliver guard, not a size target**; setting it at the target collapsed the torus to degenerate triangles | **C, with a named failure mode** | Same as 5.1. The header's warning is the best-documented constraint in the module and should survive any change. |

## 6. NonManifoldEdgeRefiner (priority 4)

| # | mechanism | what it does | what it was measured to fix | bucket | retirement condition |
| -- | -- | -- | -- | -- | -- |
| 6.1 | **The refiner itself** | Inserts a point at a non-manifold defect to repair it | OPE-170/171 | **B** — compensates for the restricted set violating the per-edge invariant | Retires when the invariant holds by construction. Under OPE-186's region-labelling direction, `MissingFace` and `ExcessFace` become unrepresentable and this class becomes an **assertion**, not a work list. Strongest "may drop" candidate for the OPE-186 summary. |
| 6.2 | **Split the curve segment rather than project onto a surface** | Keeps the repair point exactly on the crease | OPE-170/171: a projected point lands near the crease but not on it, which *grew* the defect count | **A/B** | Retires with 6.1. |
| 6.3 | **Size floor `length <= minimumEdgeLength_` → unrefinable** | Advertised as the stopping rule, "same reasoning as the other priorities" | **OPE-184 instrumented all seven refusal sites over the 65-defect repro: 107 refusals, size floor fired 0 times.** | **C — kept, comment corrected, `4e494cd`** | It is not the real cutoff. The real cutoff is `2 × minimumEdgeLength`, enforced invisibly by the proximity guard (6.5): 50 of 56 proximity refusals were blocked by the defect edge's *own* endpoint, at distance `length/2` by construction. Kept rather than deleted — "never fires" was one model at one pinned floor — but the misleading comment is gone and the real cutoff is now stated beside it. |
| 6.4 | **`unrefinableNonManifoldEdges_` give-up set** | Stops retrying a defect forever | stated termination guarantee | **B** | Necessary *given* 6.5. Retires when repair cannot be structurally blocked. |
| 6.5 | *(adjacent)* **`RCDTPointInserter` proximity guard** — reject any point within `minimumEdgeLength_` of an existing node | Prevents near-duplicate nodes corrupting the mesh | OPE-184: 56 of 107 priority-4 refusals; 53 of 56 defect edges had length in `(floor, 2·floor)` | **B masquerading as A** | Out of scope to change, **in scope to name**: the advertised size floor (6.3) is not the operative one, and the operative one is a duplicate-detection guard doing double duty. Any statement about priority 4's behaviour that does not mention this is wrong. OPE-184 measured that exempting it *alone* is disproven (65 → 1242): **do not retry that.** |
| 6.6 | *(adjacent)* **`encroachesProtectingBall`** | Never insert inside a protecting ball | OPE-176 | **A** | Retires with the protecting-ball scheme. Named here because it is 12 of the 107 refusals. |

---

## What the inventory pass found before any measurement

Four things fall out of reading alone, and all four are cheap to act on:

1. **`FaceRestriction::Unconfirmed` is produced and never consumed** (1.14).
   Its documented job — making a classification failure countable instead of a
   silent hole — is not done by anything. It is also exactly the shape of the
   dependent variable step 2 needs.
2. **The "why one rival?" question has an answer** (1.7), written in
   `63e23b4`'s commit message and absent from the code. It is derived, not
   fitted, so it belongs in bucket B rather than C — one of the ticket's
   starting points resolves without an experiment.
3. **The most recent constant in the module was fitted by the forbidden
   method** (5.2, `9d32abf`, 2026-09-13): a percentile swept against
   non-manifold defect counts. Evidence for the ticket's premise, and a
   concrete first thing to re-derive.
4. **Priority 4's advertised size floor never fires** (6.3); the operative
   cutoff is twice that, enforced by an unrelated guard (6.5). The module's
   own comment ("same reasoning as the other priorities") is not true of the
   code as it runs.

## Bucket totals

| bucket | count | rows |
| -- | -- | -- |
| **A** (incl. A-with-violated-precondition) | 14 | 1.1, 1.2, 1.3, 1.6, 1.12, 1.13, 2.1, 2.8, 2.9, 3.1, 3.2, 4.6, 4.7, 6.6 |
| **B** | 12 | 1.4, 1.5, 1.7, 1.8, 2.3, 2.4, 2.5, 4.9, 6.1, 6.2, 6.4, 6.5 |
| **C** | 13 | 1.9, 1.10, 1.11, 1.14, 2.6, 2.7, 4.2, 4.5, 4.8, 5.1, 5.2, 5.3, 6.3 |
| **A/C** (constraint derived, value chosen) | 3 | 4.1, 4.3, 4.4 |
| **A/B** | 1 | 2.2 |

43 rows in all. Six of the thirteen C rows are **termination or cost safety
devices** (1.11, 4.8, 5.1, 5.2, 5.3, 6.3) rather than tuning knobs. The
ticket's rule about `minimumEdgeLength` generalises to all of them: their
failure mode is *silent degradation*, so the useful question is not "what
value?" but "does it bind, and does anything say so when it does?"

## What was decided, and what was done

### The ablation was dropped, on purpose

The ticket's method was: inventory, instrument the validity conditions, then
ablate each B and C mechanism against that instrumentation. **Steps 2 and 3
were not run, by decision, and this is not an omission to be quietly
corrected later.**

The reason is the shape the inventory exposed. Roughly eleven of the
forty-three rows — 1.4, 1.5, 2.3, 2.4, 2.5, 6.1, 6.2, 6.4 and the heuristics
riding along in 2.6/2.7/6.3 — compensate for **one** mechanism, the dual-edge
test in 1.1. OPE-186 replaces it. Deriving careful retirement conditions for
layers that retire together, by instrumenting a primitive that is about to be
deleted, is work thrown away on completion.

What the ticket actually needed from this exercise was the requirements list
for OPE-186 — what the replacement must subsume and what it may drop. That is
produced by the inventory alone, and is recorded as a comment on OPE-186.

**This does not retire the instrumentation idea.** Step 2's dependent variable
— per classification query, which route answered, and whether the dual edge was
in a regime where the crossing test is meaningful — is still the right way to
judge a replacement. It should be built **against the new oracle**, where the
answer is load-bearing, rather than against the old one, where it only
confirms what OPE-186's own measurement already established. `LocalFeatureSize3D`
(OPE-181 stage 1: committed, tested, no consumer) is still the natural
denominator when that happens.

### Rows closed in this pass

Four needed no measurement to settle, and one turned out to need a one-line
measurement that had never been taken.

| row | disposition |
| -- | -- |
| 1.14 `Unconfirmed` | **Wired** (`4e494cd`). Was produced at four sites, consumed nowhere. Now counted at initial classification and logged. Bucket C → A. |
| 1.7 one-rival rule | **Derivation moved into the code** (`4e494cd`). It was in `63e23b4`'s commit message only, which made a derived guard look like a fitted constant. Bucket C → B. |
| 6.3 priority-4 size floor | **Kept, comment corrected** (`4e494cd`). It never fires; the operative cutoff is twice it, via an unrelated proximity guard. |
| 5.2 `SOURCE_SIZE_PERCENTILE` | **Marked fitted, retirement condition recorded** (`4e494cd`). Its plateau was found by the exact method this ticket forbids. |
| 1.11 sample cap | **Now reports when it binds** (`9c8cca1`) — and it binds. See below. |

The other C rows are explicitly accepted with their retirement conditions in
the table above. 4.8 (`MAX_SUBDIVISION_ITERATIONS`) needed nothing: it already
warns when it binds, and says to investigate as a bug.

### The one new finding, and the question deliberately left open

`MAXIMUM_SAMPLES_PER_DIRECTION` binds on **`HexNutChamferedSurfaceMesh`**: two
of its surfaces ask for 562 sample columns and get 400, so cells come out
0.0496 across against a 0.0354 target — 40% coarser than the floor calls for,
which voids 1.9's guarantee. Every other example model is clear of the cap.

That same model is the worst-behaved in the set: **494 unconfirmed faces
against 410 restricted** at initial classification, and **12 holes plus 3
excess** surviving to the end. It has no golden, so nothing was tracking it.

**Whether the coarse tessellation causes those holes is untested, on purpose.**
It is one model, and the way to settle it — raise the cap, re-run, read off
the defect count — is precisely the method this ticket exists to stop. It
would also be measuring the oracle OPE-186 is replacing. The correlation is
recorded here and on OPE-186; if it is still interesting after the oracle
changes, it can be settled then against a dependent variable that means
something.

### Coverage added alongside

Nothing asserted that a closed solid produced a closed surface. Only the
BoxWithHole volume test checked watertightness, and only the torus checked an
Euler characteristic — so a puncture in the cylinder, sphere or box surface
mesh passed every test those models had. `IsAClosedTwoManifold` now covers all
four (`a06ac31`), built on `tests/Meshing/Core/SurfaceMeshTopology.{h,cpp}`.

Measured from the goldens before the tests were written: cylinder χ=2, HexNut
χ=0, BoxWithHoleSurface χ=0, all edges on exactly 2 triangles. The saddle has
one edge on 4 — the known OPE-184 residual, on a stress benchmark rather than
a unit-test model, so it is not gated.

The library still only *logs* its defect breakdown on the surface path;
`requireClosedBoundary` guards `meshVolume()` alone, and it tests missing
faces only. Filed as **OPE-209**.

### Also removed while here

Three dead things found by tracing the same call graph, none of them in the
inventory's scope but all of the same class:

* `MeshVerifier3D` and `verifyMesh3D` (`acdcc4d`) — no callers, so
  `CHECK_MESH_EACH_ITERATION` verified nothing in 3D while `CLAUDE.md` told
  every debugging session to set it.
* `tetElementLimit` and the two `IQualityController3D` methods reading it
  (`26477a5`) — documented as capping volume refinement, never enforced.
* `IQualityController` (`26477a5`) — a third quality-controller interface with
  no implementor at all.
