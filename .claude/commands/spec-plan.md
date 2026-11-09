---
description: Produce a staged plan plus stage-spec skeletons for a task too big for one session
argument-hint: <kebab-case-task-name>
---

Task: $ARGUMENTS

You are creating a long-lived planning document for a large piece of work. It is the source of truth that later sessions pick up from. Do not implement anything and do not edit source files.

All files go in `doc/specs/$ARGUMENTS/`, named exactly:

```text
doc/specs/$ARGUMENTS/$ARGUMENTS-plan.md
doc/specs/$ARGUMENTS/$ARGUMENTS-stage-01.md
doc/specs/$ARGUMENTS/$ARGUMENTS-stage-02.md
```

Create only as many stage files as the plan needs.

## Phase 1: Read the code

Study the relevant code before drafting: entry points, existing types and interfaces, tests, build and test commands, and the parts of `CLAUDE.md`, `doc/` and the memory notes that apply, especially earlier attempts that failed. Use the LSP to trace symbols. Record real paths and symbol names. If something you need is missing from the repo, state that instead of guessing.

## Phase 2: Questions

Question me before drafting. Constraints:

1. At most 5 per round, the most important first.
2. Only ask what the code cannot answer.
3. Give concrete options with a recommended default instead of open-ended questions.
4. Cover edge cases, failure behaviour, performance and memory limits, compatibility with existing behaviour, and what is out of scope.
5. Continue until what remains is small enough to list as open questions.
6. Then say you are ready to draft and wait for my go-ahead.

## Phase 3: Write the plan

Save the plan as `doc/specs/$ARGUMENTS/$ARGUMENTS-plan.md` with exactly these sections:

1. `# <Task Title>`
2. `## Metadata`
3. `## Goal`
4. `## Current Problem`
5. `## Target Design`
6. `## Agreed Semantics`
7. `## Scope`
8. `## Constraints`
9. `## Assumptions`
10. `## Behavior Baseline`
11. `## Code Reading Notes`
12. `## Stage Plan`
13. `## Validation Strategy`
14. `## Open Questions`
15. `## Decision Log`
16. `## Handoff Notes`

### Rules for the plan

1. `Metadata` lists `Status`, `Created`, `Owner`, `Area`, `Plane ticket` and `Intended delivery`.
2. `Scope` has two separate lists, `In scope:` and `Out of scope:`.
3. `Assumptions` is a numbered list so items can be cited.
4. `Open Questions` is a numbered list for the same reason.
5. `Behavior Baseline` says what must not change and how that is shown (for example `./scripts/refactor-check.sh` against the goldens), and records current numbers measured now rather than recalled.
6. In `Stage Plan` call each unit a `Stage`, never a PR. Each stage gives:
   1. its purpose,
   2. the changes expected,
   3. how a reviewer should read it,
   4. the stage spec file,
   5. what validation concentrates on.
7. `Handoff Notes` names the file the next session reads first and the stage spec to write next.

## Phase 4: Stage skeletons

Write one file per stage in the plan, `doc/specs/$ARGUMENTS/$ARGUMENTS-stage-NN.md`, with exactly these sections:

1. `# <Task Title>: Stage NN`
2. `## Metadata`
3. `## Parent Plan`
4. `## Stage Goal`
5. `## Context`
6. `## Scope`
7. `## Assumptions`
8. `## Current State`
9. `## Design`
10. `## Acceptance Criteria`
11. `## Open Questions`
12. `## Implementation Steps`
13. `## Validation`
14. `## Handoff Notes`

### Rules for stage files

1. `Parent Plan` links to `$ARGUMENTS-plan.md`.
2. `Metadata` holds `Status`, `Stage` and `Created`.
3. `Assumptions` and `Open Questions` are numbered lists.
4. `Acceptance Criteria` are executable checks only: a command to run, a test to add, or a file or output to inspect, each with the expected result.
5. `Implementation Steps` are ordered and small, and reviewable on their own wherever possible. One move per step, each ending in a green build, the fast tests and the goldens.
6. A skeleton may contain TODOs, but must already carry the stage's purpose and scope from the plan.

## Keeping files consistent

1. The plan owns the overall goal, the split into stages, constraints that span stages, decisions, and the order of handoff.
2. Each stage file owns the implementation detail for its own stage.
3. If a stage alters the overall design or the split, update the plan in the same session.
4. Plan and stage files must never drift apart unnoticed. Log every deliberate change in `Decision Log`.

## General rules

1. Never invent requirements, figures or decisions. Anything not from me or the code goes under `Assumptions` or `Open Questions`.
2. If one session can hold the whole task, write a single stage and say so in `Stage Plan`.
3. Keep the plan brief enough to be usable, but detailed enough in code paths and decisions that a fresh session can carry on without the earlier conversation.
4. A refactor stage must not include a bug fix or a tolerance change. Give those their own stage.
5. Do not commit any of the files.
6. When the files are saved, print `Assumptions`, `Open Questions`, `Stage Plan` and `Handoff Notes` in chat.
