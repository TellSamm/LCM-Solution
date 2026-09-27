# LCM Solution - one-command launcher (Windows PowerShell).
#   .\run.ps1 judge C:\path\to\bag_dir [tram_id] [rate]   # node + bag play + record + metrics + resources
#   .\run.ps1 node  [tram_id]                              # only the node - play bags / judge from another terminal (docker exec)
#   .\run.ps1 shell                                        # interactive shell inside the image
param([string]$Mode = "help", [string]$Arg1 = "", [string]$Arg2 = "", [string]$Arg3 = "")
$ErrorActionPreference = "Stop"
$IMAGE = if ($env:IMAGE) { $env:IMAGE } else { "ghcr.io/tellsamm/lcm-odometry:latest" }
Set-Location $PSScriptRoot
function Ensure-Image {
  Write-Host "[run] checking image $IMAGE ..."
  docker pull -q $IMAGE *> $null; if ($LASTEXITCODE -eq 0) { return }
  docker image inspect $IMAGE *> $null; if ($LASTEXITCODE -eq 0) { Write-Host "[run] registry unreachable, using local image"; return }
  Write-Host "[run] pull failed -> building locally (3-5 min)"; docker build -t $IMAGE -f docker/Dockerfile .
}
switch ($Mode) {
  "judge" {
    if (-not $Arg1 -or -not (Test-Path $Arg1 -PathType Container)) {
      Write-Host "ERROR: bag folder not found: '$Arg1'"
      Write-Host "Pass the path to ONE bag folder on your disk (it contains <name>_0.db3 and metadata.yaml), e.g.:"
      Write-Host "  .\run.ps1 judge C:\hackathon\data\30618_0e41eac3"
      Write-Host "Note: a path inside a .zip (C:\...\data.zip\...) is not a folder - extract the archive first."
      exit 1
    }
    if (-not (Test-Path (Join-Path $Arg1 "metadata.yaml"))) { Write-Host "ERROR: no metadata.yaml in '$Arg1' - this is not a bag folder. Expected something like ...\30618_0e41eac3"; exit 1 }
    $bag = Resolve-Path $Arg1; $parent = Split-Path $bag -Parent; $name = Split-Path $bag -Leaf
    $tram = if ($Arg2) { $Arg2 } else { "30618" }; $rate = if ($Arg3) { $Arg3 } else { "1.0" }
    New-Item -ItemType Directory -Force results | Out-Null; Ensure-Image
    docker run --rm --cpus=2 --memory=512m -v "${parent}:/data:ro" -v "${PWD}\results:/out" $IMAGE bash /tools/judge_run.sh "/data/$name" $tram $rate
  }
  "node" {
    $tram = if ($Arg1) { $Arg1 } else { "30618" }; Ensure-Image
    docker run --rm -it --name lcm $IMAGE ros2 launch tram_odometry odometry.launch.py tram_id:=$tram
  }
  "shell" { Ensure-Image; docker run --rm -it $IMAGE bash }
  default { Get-Content $PSCommandPath | Select-Object -First 4 }
}
