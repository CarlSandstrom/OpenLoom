#!/usr/bin/env bash
#
# Guards the layering between the shared 3D meshing code and RCDT (OPE-130).
#
# Meshing/Core/3D/General/ is the algorithm-agnostic layer: a second meshing
# algorithm is meant to be built on it without touching RCDT/. That only holds
# while nothing in General/ depends on RCDT/, so this check fails on any
# #include of an RCDT/ header from a .h or .cpp under General/.
#
#   scripts/check-layers.sh              check src/Meshing/Core/3D/General
#   scripts/check-layers.sh DIRECTORY    check another directory (for testing
#                                        the check itself)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIRECTORY="${1:-$ROOT/src/Meshing/Core/3D/General}"

if [[ ! -d "$DIRECTORY" ]]; then
    echo "check-layers: no such directory $DIRECTORY" >&2
    exit 2
fi

violations="$(grep -rnE '^[[:space:]]*#[[:space:]]*include[[:space:]]+"[^"]*RCDT/' \
    --include='*.h' --include='*.cpp' "$DIRECTORY" || true)"

if [[ -z "$violations" ]]; then
    checked="$(find "$DIRECTORY" \( -name '*.h' -o -name '*.cpp' \) | wc -l)"
    echo "OK: none of the $checked files under $DIRECTORY includes RCDT/."
    exit 0
fi

echo "$violations"
echo "FAIL: shared code includes RCDT/ -- move what it needs out of RCDT/ instead."
exit 1
