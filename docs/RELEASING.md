# Releasing

Releases are automatic: every push to `main` runs `.github/workflows/release.yaml`. If the Conventional Commits since
the last tag call for a release, it cuts one; otherwise it does nothing. The pipeline follows CrashLoggerSSE,
open-shaders and devbench, and uploads through `alandtse/nexus-workflows`.

## Flow

1. **Build and test** (`_build.yaml`, also run by `build.yaml` on every PR and push): builds the DLL and the Catch2
   suite with xmake, runs the suite (WARP, no GPU needed), and stages the DLL, PDB, `shaders/` and the shader cache.
   The test run compiles the solver kernels through the plugin's own shader path, so it fills the cache that ships
   (`GPUSMP_SHADER_CACHE_DIR`). A failing test stops the release.
2. **Semantic release**: reads the commit history, picks the version (`feat` minor, `fix`/`perf` patch, `!` or a
   `BREAKING CHANGE` major), bumps `local version` in `xmake.lua`, commits it with `[skip ci]`, tags `vX.Y.Z` and creates
   a **draft** GitHub Release whose body is the changelog.
3. **Package** (`tools/release/package.ps1`): builds `GPUSMPPhysicsNG-vX.Y.Z.7z`, rooted at the game's `Data` folder:
   `SKSE/Plugins/GPUSMPPhysicsNG.dll`, `.pdb`, and `GPUSMPPhysicsNG/Shaders` plus `GPUSMPPhysicsNG/ShaderCache`.
   `Settings.json` is left out on purpose so an update never overwrites a user's config. It attaches the archive and
   promotes the draft to published, so a failed build leaves a hidden draft instead of a tag with no download.
4. **Nexus** (`nexus-upload.yaml`): a dry run by default. It reports what it would upload and whether Nexus already has
   the version.

## The shader cache

`RigidSetup` takes about 3 s to compile (53 s before the joint rows were streamed), and the archive ships the compiled blobs so a first launch skips even that. Entries are keyed by shader path
under the shader root and checked by a content hash of the source and its includes, so they only serve the exact
shader bytes they were built from. That is why the Shaders folder in the archive comes from the build job, never a
fresh checkout. If the cache is missing or stale the plugin compiles as before. Locally,
`tools/game/build_shader_cache.ps1` builds one.

## Setup once (repo owner)

- Create the GitHub repository (this repo has no remote yet) and push `main`. Jobs run only when
  `github.repository_owner == 'alandtse'`, so forks never get release credentials.
- Secrets `UNEX_APIKEY` and `UNEX_NEXUSMODS_SESSION_COOKIE` (check them with the **Nexus: Check Credentials**
  workflow). The cookie expires and is managed centrally by `nexus-workflows`.
- Repository variable `NEXUS_AUTO_UPLOAD=true` turns the dry run into a real upload. Until then the Nexus job only
  reports.
- Branch protection: require the **Build**, **Lint** and **PR: Lint** checks; squash-merge so the PR title (validated
  as a Conventional Commit) becomes the commit subject.

## Releasing by hand

- **Re-run a Nexus upload:** run **Nexus: Upload Release** with a published tag and `dry_run=false`.
- **Check the archive locally:** build, then `tools/release/package.ps1 -Tag v0.0.0-test -Bin <folder>` where the folder
  holds the DLL, PDB, `Shaders/` and `ShaderCache/`.
