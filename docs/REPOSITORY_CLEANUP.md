# Repository cleanup policy

## Why generated files are removed

A source checkout should contain the algorithms, compilation configuration, calibrated board files, figures and source documentation required for continued development. It does not need Visual Studio database caches, precompiled header caches, intermediate object files or debugging binaries. The GitHub `main` snapshot contained approximately **120 generated files** of this type, totalling approximately **989 MB** according to the reported Git tree sizes.

## Removal rules applied by the assembly script

- All files in `.vs/`, and temporary `x64/Debug/`, `x64/Release/`, `Win32/Debug/`, `Win32/Release/`, `ipch/` and other transient build folders.
- Visual Studio user state: `*.suo`, `*.wsuo`, `*.vcxproj.user`, `*.VC.db`, `*.VC.opendb`, `*.vsidx`.
- Compiled intermediates: `*.ipch`, `*.obj`, `*.pdb`, `*.ilk`, `*.tlog`, `*.recipe`, `*.lastbuildstate` and build logs.
- The old `AsierInhoUDP.vcxproj.bak` backup.

Where a Git checkout is present, deletion is reflected in the Git index so the subsequent commit removes these files from the *current branch*. Earlier commits remain in Git history unless history is separately rewritten; history rewriting is outside the scope of this source revision.

## Files retained deliberately

- `GSPAT_SolverV2/`, `AsierInhoUDP/`, `WirelessBoard.sln` and `.vcxproj` build definitions.
- Four-array and historical AcoustoBots C++ source, including the older PhaseSpace acquisition example, as development provenance.
- Calibration and test input files (`*.pat`, `hologramSolver_V2.cl`, `Inputs/*.txt`).
- Diagrams, screenshots and transport demonstration media in `ContactlessCaging_Images/` and `ContactlessCaging_Videos/`.

Historical code remains labelled as a previous development configuration; it is not selected as the main four-board executable. No user-supplied experiment measurement files are deleted by the cleanup policy.

## Inspect deletion list before publishing

```bash
git status --short | grep '^D\|^ D'  # Linux / macOS
# On Windows PowerShell:
git status --short | Select-String '^[ D]D '
```

For the full path list, review `docs/REMOVED_BUILD_ARTIFACTS.txt` generated during assembly. Do not delete original data, firmware or additional research files merely because their relevance is uncertain.
