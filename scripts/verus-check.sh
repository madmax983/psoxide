#!/bin/sh
# Run the Verus formal-verification proofs for psoxide.
#
# Mirrors scripts/verus-check.ps1 for Linux/macOS. Verus is installed
# out-of-band (it is not a Cargo dependency); if no `verus` binary is
# discoverable this script skips gracefully and exits 0, so it never breaks
# a checkout without Verus.
#
# Verus is located via (in order): the VERUS env var, the VERUS_BIN env var,
# or a `verus` on PATH.
set -eu

verus="${VERUS:-${VERUS_BIN:-verus}}"

if ! command -v "$verus" >/dev/null 2>&1; then
    echo "Verus not found (looked for VERUS / VERUS_BIN / 'verus' on PATH), skipping checks."
    exit 0
fi

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
proof_files="bus_map.rs decode.rs map_region.rs timing.rs"

for name in $proof_files; do
    file="$repo_root/crates/psoxide-proof/src/$name"
    echo "[verus] checking $file"
    "$verus" "$file"
done

echo "All Verus checks passed."
