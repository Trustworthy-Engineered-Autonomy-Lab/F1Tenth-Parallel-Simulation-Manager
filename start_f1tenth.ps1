param(
    [ValidateRange(1, 100)]
    [int]$NumSessions = 3,
    [int]$MaxLaps = 3
)

$imageName = "f1tenth_gym_ros"
$ErrorActionPreference = 'Stop'
$repoRoot = $PSScriptRoot
Set-Location -LiteralPath $repoRoot
$resultsRoot = Join-Path $repoRoot "results"

Write-Host "[1/4] Creating results root..."
New-Item -ItemType Directory -Force -Path $resultsRoot | Out-Null

Write-Host "[2/4] Building Docker image..."
docker build -t $imageName .
if ($LASTEXITCODE -ne 0) { throw 'Docker build failed; no sessions were started.' }

Write-Host "[3/4] Removing old session containers if they exist..."
# combine with loop below
$oldSessions = @(docker ps -aq --filter "name=f1tenth_session_")
if ($oldSessions.Count -gt 0) { docker rm -f @oldSessions | Out-Null }

Write-Host "[4/4] Launching $NumSessions parallel simulation containers..."
for ($i = 1; $i -le $NumSessions; $i++) {
    $containerName = "f1tenth_session_$i"
    $novncName = "novnc_session_$i"  
    $vncPort = 8080 + $i 
    $sessionResults = Join-Path $resultsRoot "session_$i"
    New-Item -ItemType Directory -Force -Path $sessionResults | Out-Null

    # Copy overtake settings to results
    $configSource = Join-Path $repoRoot "config\opp_overtake_settings.yaml"
    $configDest = Join-Path $sessionResults "opp_overtake_settings.yaml"
    Copy-Item -Path $configSource -Destination $configDest -Force
    Write-Host "Copied overtake settings to $sessionResults"

    Write-Host "Starting Session $i..."

    # remove old containers
    $oldVnc = @(docker ps -aq --filter "name=^/${novncName}$")
    if ($oldVnc.Count -gt 0) { docker rm -f @oldVnc | Out-Null }

    # launch no vnc with the according port address for the session id#
    docker run -d `
        --name $novncName `
        -p "${vncPort}:8080" `
        theasp/novnc:latest
    if ($LASTEXITCODE -ne 0) { throw "Failed to start $novncName" }

    # launch the sim
    # get unique identifiers and domains for each
    $rosDomainId = $i    
    $containerCmd = "/sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh"
    
    docker run -d `
        --name $containerName `
        --link "${novncName}:novnc" `
        -e DISPLAY=novnc:0.0 `
        -e SESSION_ID=$i `
        -e ROS_DOMAIN_ID=$rosDomainId `
        -e MAX_LAPS=$MaxLaps `
        -e RESULTS_DIR="/sim_ws/results/session_$i" `
        -e ATTACH_TMUX=0 `
        -v "${repoRoot}:/sim_ws/src/f1tenth_gym_ros" `
        -v "${sessionResults}:/sim_ws/results/session_$i" `
        $imageName `
        $containerCmd
    if ($LASTEXITCODE -ne 0) { throw "Failed to start $containerName" }
}

Write-Host ""
Write-Host "All sessions launched."
Write-Host "Check running containers with:"
Write-Host "  docker ps"
Write-Host ""
Write-Host "Check logs with:"
Write-Host "  docker logs -f f1tenth_session_1"
Write-Host "  docker logs -f f1tenth_session_2"
Write-Host "  docker logs -f f1tenth_session_3"
Write-Host ""
Write-Host "Dashboard to view sessions: file://$($repoRoot.Replace('\', '/'))/dashboard.html" -ForegroundColor Cyan
