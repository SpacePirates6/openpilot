# GitHub setup — Chauffeur Stop

Local branch **`chauffeur-stop`** is committed and ready to push.

## One-time: log in to GitHub CLI

```powershell
gh auth login
```

Choose: GitHub.com → HTTPS → Login with browser.

## Push (automated)

```powershell
cd c:\Users\space\Desktop\comma
.\scripts\push_chauffeur_stop_github.ps1
```

This will:

1. Fork `mvl-boston/openpilot` and `mvl-boston/opendbc` under your account (if needed)
2. Push `chauffeur-stop` to **both** repos

## Manual push (if you prefer)

### opendbc (Honda carcontroller changes)

```powershell
cd opendbc_repo
git remote add fork https://github.com/SpacePirates6/opendbc.git
git push -u fork chauffeur-stop
```

### openpilot (main chauffeur stop + learner)

```powershell
cd c:\Users\space\Desktop\comma
git remote add fork https://github.com/SpacePirates6/openpilot.git
git push -u fork chauffeur-stop
```

## Install on Comma device

After push, on the device choose **Custom Software**:

```
SpacePirates6/openpilot/chauffeur-stop
```

Requires alpha longitudinal + Honda Bosch for full brake shaping (opendbc submodule must resolve to your opendbc fork commit).

## Submodule note

The openpilot commit pins `opendbc_repo` to commit `16841bf` on branch `chauffeur-stop`. For a public clone to build, either:

- Push opendbc to your fork (script does this), and ensure `.gitmodules` URL matches your fork, **or**
- Keep using `mvl-boston/opendbc` only if that commit is merged upstream (it is not today).

To point clones at your opendbc fork permanently, edit `.gitmodules`:

```
url = https://github.com/SpacePirates6/opendbc
branch = chauffeur-stop
```

## Sunnylink toggles

- `ChauffeurStopEnabled`
- `GasOverrideSmoothEnabled`

Both appear under **Cruise** after install.
