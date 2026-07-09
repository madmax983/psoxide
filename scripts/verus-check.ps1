$ErrorActionPreference = "Stop"

$verus = "verus"

try {
    Get-Command $verus -ErrorAction Stop > $null
} catch {
    Write-Host "Verus not found, skipping checks."
    exit 0
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$proofFiles = @(
    (Join-Path $repoRoot "crates/psoxide-proof/src/bus_map.rs"),
    (Join-Path $repoRoot "crates/psoxide-proof/src/decode.rs")
)

foreach ($file in $proofFiles) {
    Write-Host "[verus] checking $file"
    & $verus $file
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

Write-Host "All Verus checks passed."
