<#
.SYNOPSIS
  Assembles the mod archive from a build's staged binaries. CI (release.yaml) and a local dry run use the same script.
.DESCRIPTION
  The archive root is the game's Data folder, so a mod manager installs it as is.
  Settings.json stays out: the plugin writes it on first run and the in-game menu rewrites it, so an update must not
  clobber a user's config. Shaders ship because the plugin compiles them on the user's machine; ShaderCache is the
  prebuilt blobs that skip that compile, valid only against these exact shader bytes, so both come from the build job.
.PARAMETER Bin
  Folder holding GPUSMPPhysicsNG.dll, GPUSMPPhysicsNG.pdb, Shaders/ and (optional) ShaderCache/.
#>
param(
    [Parameter(Mandatory)][string]$Tag,
    [string]$Bin = 'bin',
    [string]$Out = '.'
)
$ErrorActionPreference = 'Stop'
$seven = (Get-Command 7z, 7zz, 7za -ErrorAction SilentlyContinue | Select-Object -First 1).Source
if (-not $seven) { $seven = "$env:ProgramFiles\7-Zip\7z.exe" }
if (-not (Test-Path $seven)) { throw '7z not found' }

foreach ($required in 'GPUSMPPhysicsNG.dll', 'GPUSMPPhysicsNG.pdb', 'Shaders') {
    if (-not (Test-Path (Join-Path $Bin $required))) { throw "missing $required in $Bin" }
}

$stage = Join-Path ([IO.Path]::GetTempPath()) ("gpusmp-pkg-" + [guid]::NewGuid().ToString('N'))
$plugins = Join-Path $stage 'SKSE/Plugins'
$data = Join-Path $plugins 'GPUSMPPhysicsNG'
New-Item -ItemType Directory -Force $data | Out-Null
Copy-Item (Join-Path $Bin 'GPUSMPPhysicsNG.dll'), (Join-Path $Bin 'GPUSMPPhysicsNG.pdb') $plugins
Copy-Item (Join-Path $Bin 'Shaders') (Join-Path $data 'Shaders') -Recurse
$cache = Join-Path $Bin 'ShaderCache'
if (Test-Path $cache) { Copy-Item $cache (Join-Path $data 'ShaderCache') -Recurse } else { Write-Warning 'no ShaderCache: first launch compiles every shader' }

$archive = Join-Path (Resolve-Path $Out) "GPUSMPPhysicsNG-$Tag.7z"
if (Test-Path $archive) { Remove-Item $archive }
Push-Location $stage
try { & $seven a -t7z -mx=9 $archive 'SKSE' | Out-Null; if ($LASTEXITCODE -ne 0) { throw "7z failed ($LASTEXITCODE)" } } finally { Pop-Location }
Remove-Item $stage -Recurse -Force

& $seven l $archive | Select-Object -Last 12
if ($env:GITHUB_OUTPUT) { "archive=$archive" | Out-File -Append -Encoding utf8 $env:GITHUB_OUTPUT }
Write-Host "Built $archive"
