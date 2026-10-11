---
description: Find the next thing to work on in the OpenLoom project
---

Query Plane (project OpenLoom, identifier OPENL) for the next incomplete item to work on.

1. Resolve the OpenLoom project with `project list`, and its Todo / In Progress states with `state list`
2. List the project's work items with `workitem list` and pql `stateGroup IN ("unstarted","started")`
3. From all collected issues, pick the one with the highest priority (Urgent > High > Medium > Low > None). If there is a tie, pick the lowest issue number (e.g. OPENL-70 before OPENL-71)
4. Report concisely:
   - Issue ID and title (e.g. **OPENL-70** — S4.1: SurfaceMesher3D top-level class)
   - Module (find it with `module list` + `module list_workitems`; ignore the Plane demo modules)
   - Status (Todo / In Progress)
   - Priority

Keep the response short. Do not list completed work.
