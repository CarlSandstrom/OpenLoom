---
name: tidy
description: Bring a given piece of code (a file, directory, class or function) in line with the OpenLoom coding standards, then simplify it. With no argument it takes the files changed against main. Checks against .claude/rules/cpp-coding-standards.md, formats, runs /simplify, and re-verifies. Leaves the result uncommitted.
---

# Tidy a piece of code

Usage: `/tidy <target>`, where target is a file, a directory, a class name or a function name. With no target, use the `.h` and `.cpp` files changed against `main` (`git diff main --name-only`, plus untracked files).

- **File:** that file and its header or source twin.
- **Directory:** every `.h` and `.cpp` in it.
- **Class or function:** find it with the LSP (`goToDefinition`), then take its declaring and defining files, but only change that class or function.

Everything outside the target is out of scope, including its callers.

The standards live in `.claude/rules/cpp-coding-standards.md`. Read it; do not work from memory, and do not copy its rules anywhere else.

## Steps

1. **Check the standards.** Look for the departures the rules file names. The ones most often missed:
   - a function body over ~60 lines, or more than 3 levels of nesting in a loop body
   - `const` on a local variable, or `const auto*` / `auto*` where plain `auto` is meant
   - an abbreviated identifier, file name or comment
   - a private method added only to shorten another method (use a file-local free function in the `.cpp` instead)
   - include order: own header first, then alphabetical
   - comments that narrate steps or restate ticket history
   - a nullable `const T*` argument where `std::optional<T>` is meant
   - a generic `std::runtime_error` instead of the `OPENLOOM_THROW_*` macros

   A function that must stay long gets a comment saying why, not a split.
2. **Make one kind of change at a time**, as the Refactoring rules in `CLAUDE.md` ask. Renaming or moving a symbol first needs an LSP `findReferences` inventory of its call sites, and the callers are updated in the same step even though they are otherwise out of scope. Splitting a long function is its own step.
3. **Format** the target files with `/format`.
4. **Simplify** with `/simplify`, naming the target files explicitly, because `/simplify` looks at changes and the target may have none.
5. **Verify after each step.** Build, then `ctest --test-dir build/tests -j$(nproc) -LE slow --output-on-failure`. Existing code is being restructured, so also run `./scripts/refactor-check.sh`.
6. **Report** what changed, grouped by file, in a few lines, plus anything left alone and why.

## Rules

- Do not commit and do not stage. Leave the changes in the working tree. If a step is green and more steps follow, offer a commit instead of making one.
- Behaviour must not change. If a fix would alter behaviour, a tolerance, or a concept's meaning, leave it and say so.
- If `refactor-check.sh` reports a golden diff, stop and report which model moved. Do not run `--accept`.
- If the target is a directory or large file, work through it file by file and stop to report after each, rather than rewriting everything in one pass.
