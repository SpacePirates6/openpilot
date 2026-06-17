# GitHub setup — Chauffeur Stop / CSR

Two openpilot branches:

| Branch | Purpose |
|--------|---------|
| `chauffeur-stop` | Original Chauffeur Stop fork (pre-comma backport) |
| `csr` | Chauffeur Stop + comma + sunnypilot backport (use this) |

## Install on Comma device

Custom Software:

```
SpacePirates6/openpilot/csr
```

Requires alpha longitudinal for full Chauffeur Stop brake shaping on Honda. Gas Override Smoothing needs a Comma Pedal / gas interceptor.

## Submodule

`opendbc_repo` must resolve to your fork at commit `e4baa27` (branch `chauffeur-stop` on `SpacePirates6/opendbc`). That commit includes Chauffeur Stop + gas override smoothing in the Honda carcontroller and the Ridgeline park/reverse TCM fault fix.

`.gitmodules` already points at:

```
https://github.com/SpacePirates6/opendbc
branch = chauffeur-stop
```

After cloning:

```powershell
git submodule update --init --recursive opendbc_repo
```

## Push script

Requires GitHub CLI (`gh auth login`):

```powershell
cd "c:\Users\space\Downloads\comma fix"
.\scripts\push_chauffeur_stop_github.ps1
```

## Sunnylink toggles (Cruise)

- `ChauffeurStopEnabled` — soft stop below 2 mph with hill compensation
- `GasOverrideSmoothEnabled` — smooth handoff after gas-pedal override
