---
description: Write a compact implementation spec for one change, after reading the code and questioning me
argument-hint: <short description of the change>
---

Change to specify: $ARGUMENTS

The goal is a spec that a new session can implement with no other context. Do not implement anything and do not touch source files. Work through the phases in order.

## Phase 1: Read the code

Before you ask or write anything, read what the change touches: entry points, the types and interfaces involved, existing tests, how to build and test, and whatever in `CLAUDE.md`, `doc/` or the memory notes applies (prior failed attempts matter here). Use the LSP for symbol work. Record real paths and symbol names. If you need something the repo does not contain, say that rather than guessing.

## Phase 2: Questions

Ask me questions before you draft. Constraints:

- No more than 5 per round, with the most important first.
- Only ask what reading the code cannot settle.
- Offer concrete options and mark a recommended default, rather than asking open-ended questions.
- Cover these areas deliberately: edge cases, behaviour on failure, performance and memory limits, compatibility with existing behaviour (the goldens), and what is out of scope.
- Repeat until what is left is small enough to become open questions. Then say you are ready and wait for my go-ahead.

## Phase 3: Write the spec

Save it as `doc/specs/<kebab-case-name>.md`. Two pages at most. Leave out anything a reader can see from the code. Open with a one- or two-sentence summary of the work, then use exactly these sections:

1. **Context**: why the change is needed, in 2-4 sentences.
2. **Goal**: what holds once the work is finished.
3. **Non-goals**: what the change intentionally leaves alone.
4. **Current state**: the existing code that matters, with real paths and symbols.
5. **Design**: signatures, data flow and the invariants that must hold. Give the decisions and the reasons, not the goal again.
6. **Acceptance criteria**: executable checks only. Each one is a command to run or a test to add, plus the expected result. Nothing that is only prose.
7. **Assumptions**: every decision you took that I did not state. Be thorough and direct. The section is required, and "none" is rarely true. Numbered list.
8. **Open questions**: what I must decide before or during implementation. Numbered list.
9. **Implementation steps**: small steps, in order, each of which could be committed alone and each with the check that shows it worked. Numbered list.

## Rules

- Prefer numbered lists throughout, so items are easy to cite.
- Never invent requirements, figures or decisions. Anything that did not come from me or the code belongs under Assumptions or Open questions.
- A refactor step must not carry a bug fix or a tolerance change. If both are needed, split them.
- If the work is larger than one reviewable change, say so and propose separate specs.
- Do not commit the spec.
- After saving the file, print the Assumptions and Open questions sections in chat so I can read them without opening it.
