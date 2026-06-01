# Push chauffeur-stop branches to GitHub under SpacePirates6
# Requires: GitHub CLI (gh) logged in — run: gh auth login

$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent

Set-Location $Root

gh auth status | Out-Null
if ($LASTEXITCODE -ne 0) {
  Write-Host "Not logged in. Run: gh auth login" -ForegroundColor Yellow
  exit 1
}

$GhUser = (gh api user -q ".login").Trim()
Write-Host "GitHub user: $GhUser"

# Fork upstream repos if missing (no-op if already forked)
gh repo fork mvl-boston/opendbc --clone=false 2>$null
gh repo fork mvl-boston/openpilot --clone=false 2>$null

# Push opendbc submodule
Set-Location "$Root\opendbc_repo"
if (-not (git remote get-url fork 2>$null)) {
  git remote add fork "https://github.com/$GhUser/opendbc.git"
}
git push -u fork chauffeur-stop

# Push openpilot
Set-Location $Root
if (-not (git remote get-url fork 2>$null)) {
  git remote add fork "https://github.com/$GhUser/openpilot.git"
}
git push -u fork chauffeur-stop

Write-Host ""
Write-Host "Done. Branches pushed:" -ForegroundColor Green
Write-Host "  https://github.com/$GhUser/openpilot/tree/chauffeur-stop"
Write-Host "  https://github.com/$GhUser/opendbc/tree/chauffeur-stop"
Write-Host ""
Write-Host "Install on comma device (Custom Software): $GhUser/openpilot/chauffeur-stop"
