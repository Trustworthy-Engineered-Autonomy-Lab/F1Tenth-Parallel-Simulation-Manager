param(
    [int]$NumSessions = 3,
    [int]$MaxLaps = 3
)

$imageName = "f1tenth_gym_ros"
$repoRoot = (Get-Location).Path
$resultsRoot = Join-Path $repoRoot "results"

Write-Host "[1/4] Creating results root..."
New-Item -ItemType Directory -Force -Path $resultsRoot | Out-Null

Write-Host "[2/4] Building Docker image..."
docker build -t $imageName .

Write-Host "[3/4] Removing old session containers if they exist..."
for ($i = 1; $i -le $NumSessions; $i++) {
    $containerName = "f1tenth_session_$i"
    docker rm -f $containerName 2>$null | Out-Null
}

Write-Host "[4/4] Launching $NumSessions parallel simulation containers..."
for ($i = 1; $i -le $NumSessions; $i++) {
    $containerName = "f1tenth_session_$i"
    $novncName = "novnc_session_$i"  
    $vncPort = 8080 + $i 
    $sessionResults = Join-Path $resultsRoot "session_$i"
    New-Item -ItemType Directory -Force -Path $sessionResults | Out-Null

    Write-Host "Starting Session $i (GUI at http://localhost:$vncPort) ..."

    # remove old no vncs
    docker rm -f $novncName 2>$null | Out-Null

    # launch no vnc with the according port address for the session id#
    docker run -d `
        --name $novncName `
        -p "${vncPort}:8080" `
        theasp/novnc:latest

    # launch the sim
    $containerCmd = "chmod +x /sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh && /sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh"
    
    docker run -d `
        --name $containerName `
        --link "${novncName}:novnc" `
        -e DISPLAY=novnc:0.0 `
        -e SESSION_ID=$i `
        -e MAX_LAPS=$MaxLaps `
        -e RESULTS_DIR="/sim_ws/results/session_$i" `
        -e ATTACH_TMUX=0 `
        -v "${repoRoot}:/sim_ws/src/f1tenth_gym_ros" `
        -v "${sessionResults}:/sim_ws/results/session_$i" `
        $imageName `
        -lc $containerCmd
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