#!/usr/bin/env bash
#
# Guards the Key Modules table in CLAUDE.md against drift (OPE-188).
#
# Stale architecture docs are worse than absent ones: reasoning from a wrong
# table sends a refactor at the wrong module, or proposes deleting code that is
# still reachable. This check asserts that every backticked name in the table
# still exists -- a path ending in "/" must be a directory under src/, anything
# else must be a class with a header somewhere under src/.
#
# It caught RCDTContext, which the table still named long after da0e212 deleted
# it. Keep class names in backticks when editing a row, or they go unchecked.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOC="$ROOT/CLAUDE.md"

table="$(awk '/^### Key Modules/{inTable=1; next} /^### /{inTable=0} inTable && /^\|/' "$DOC")"
if [[ -z "$table" ]]; then
    echo "check-docs: could not find the Key Modules table in CLAUDE.md" >&2
    exit 1
fi

missing=0
checked=0

# Every `backticked` token in the table, deduplicated.
names="$(grep -o '`[^`]*`' <<<"$table" | tr -d '`' | sort -u)"

while IFS= read -r name; do
    [[ -z "$name" ]] && continue
    checked=$((checked + 1))

    if [[ "$name" == */ ]]; then
        if [[ ! -d "$ROOT/src/$name" ]]; then
            echo "CLAUDE.md names module $name -- no such directory src/$name"
            missing=$((missing + 1))
        fi
    elif ! find "$ROOT/src" -name "$name.h" -print -quit | grep -q .; then
        # Not a class with its own header. It may still be a legitimate name --
        # an enum value, or a type sharing a header with another -- so accept it
        # if it appears anywhere in the sources, and report it only if the name
        # has vanished entirely.
        if ! grep -rqw "$name" "$ROOT/src"; then
            echo "CLAUDE.md names $name -- it appears nowhere in src/"
            missing=$((missing + 1))
        fi
    fi
done <<<"$names"

if [[ $missing -eq 0 ]]; then
    echo "OK: $checked names in the Key Modules table all resolve."
    exit 0
fi
echo "FAIL: $missing stale reference(s) in the Key Modules table."
exit 1
