#!/usr/bin/env bash
#
# Behaviour-preservation oracle for refactoring (OPE-188).
#
# A refactor changes structure, not output. This script meshes a set of
# representative models and compares every .vtu against a committed golden
# file. Any difference means the change is not a refactor.
#
#   scripts/refactor-check.sh            compare against the goldens
#   scripts/refactor-check.sh --accept   re-bless the goldens
#   TIER=full scripts/refactor-check.sh  include the slow models
#
# Re-blessing is deliberately a separate, explicit action: it turns an
# intentional behaviour change into a reviewable diff in the commit instead
# of a silent drift.
#
# VtkExporter writes ASCII, so a mismatch produces a readable diff showing
# which nodes or cells actually changed.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXAMPLES_DIR="$ROOT/build/src/Examples"
GOLDEN_DIR="$ROOT/tests/golden"

# Fast tier: one model per meshing pathway, ~9 s total.
FAST_EXAMPLES=(
    "2D/SimpleDelaunay2D"          # 2D Delaunay core
    "2D/SquareWithCircularHole"    # 2D constraints and holes
    "3D/Volume/CreateBox"          # 3D volume tetrahedralization
    "3D/Surface/CylinderSurfaceMesh"  # single periodic surface, seam handling
    "3D/Surface/HexNutSurfaceMesh"    # planar faces plus a periodic bore
)
# Both surface models above resolve to AmbientRCDT: SurfaceMesher3D's Auto
# strategy picks it whenever the topology has seams, and a cylindrical face
# always produces one. The legacy PerFaceUV pipeline has no example and no
# golden -- its only coverage is two unit tests on a unit box.

# Full tier adds the slow models, including the crease-protection benchmark.
FULL_EXAMPLES=(
    "3D/Surface/BoxWithHoleSurface"
    "3D/Volume/BoxWithHole"
    "3D/Surface/SaddleSurfaceMesh"
)

accept=false
if [[ "${1:-}" == "--accept" ]]; then
    accept=true
elif [[ $# -gt 0 ]]; then
    echo "usage: $(basename "$0") [--accept]   (TIER=fast|full)" >&2
    exit 2
fi

examples=("${FAST_EXAMPLES[@]}")
if [[ "${TIER:-fast}" == "full" ]]; then
    examples+=("${FULL_EXAMPLES[@]}")
fi

workDirectory="$(mktemp -d)"
trap 'rm -rf "$workDirectory"' EXIT

# Cheap and closely related: a refactor that moves or deletes a module should
# not leave the architecture table in CLAUDE.md describing the old shape.
if ! $accept && ! "$ROOT/scripts/check-docs.sh"; then
    echo "(architecture table is stale -- fix CLAUDE.md before continuing)" >&2
    exit 1
fi

changed=0
missing=0
compared=0

for example in "${examples[@]}"; do
    name="$(basename "$example")"
    binary="$EXAMPLES_DIR/$example"

    if [[ ! -x "$binary" ]]; then
        echo "MISSING BINARY: $example (build first)" >&2
        exit 1
    fi

    runDirectory="$workDirectory/$name"
    mkdir -p "$runDirectory"
    if ! ( cd "$runDirectory" && SPDLOG_LEVEL=warn "$binary" >"$runDirectory/.stdout" 2>&1 ); then
        echo "FAILED TO RUN: $name" >&2
        tail -20 "$runDirectory/.stdout" >&2
        exit 1
    fi

    shopt -s nullglob
    outputs=("$runDirectory"/*.vtu)
    shopt -u nullglob
    if [[ ${#outputs[@]} -eq 0 ]]; then
        echo "NO OUTPUT: $name produced no .vtu" >&2
        exit 1
    fi

    for output in "${outputs[@]}"; do
        golden="$GOLDEN_DIR/$name/$(basename "$output")"

        if $accept; then
            mkdir -p "$(dirname "$golden")"
            cp "$output" "$golden"
            echo "blessed  $name/$(basename "$output")"
            continue
        fi

        compared=$((compared + 1))
        if [[ ! -f "$golden" ]]; then
            echo "NO GOLDEN: $name/$(basename "$output") -- run with --accept"
            missing=$((missing + 1))
        elif ! diff -q "$golden" "$output" >/dev/null; then
            echo
            echo "CHANGED: $name/$(basename "$output")"
            diff "$golden" "$output" | head -20
            changed=$((changed + 1))
        fi
    done
done

if $accept; then
    echo
    echo "Goldens re-blessed. Review the diff before committing."
    exit 0
fi

echo
if [[ $changed -eq 0 && $missing -eq 0 ]]; then
    echo "OK: $compared outputs unchanged (tier=${TIER:-fast})."
    exit 0
fi
echo "FAIL: $changed changed, $missing missing, out of $compared outputs."
exit 1
