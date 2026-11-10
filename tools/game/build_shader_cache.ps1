<#
.SYNOPSIS
  Fills a ShaderCache folder by running the unit tests, which compile the solver kernels through the plugin's own path.
  Ship the folder as Data/SKSE/Plugins/GPUSMPPhysicsNG/ShaderCache so a first launch skips the compile.
#>
param([string]$Out = (Join-Path $PSScriptRoot '..\..\build\ShaderCache'))
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$exe = Join-Path $root 'build\windows\x64\releasedbg\GPUSMPPhysicsNG-Tests.exe'
if (-not (Test-Path $exe)) { throw "build GPUSMPPhysicsNG-Tests first: $exe" }
New-Item -ItemType Directory -Force $Out | Out-Null
$env:GPUSMP_SHADER_CACHE_DIR = (Resolve-Path $Out).Path
& $exe '[d3d],[rigid_gpu],[contact_gpu]' | Select-Object -Last 2
"$((Get-ChildItem $Out -Filter *.bin).Count) blobs in $Out"
