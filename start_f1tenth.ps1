param(
    [int]$NumSessions = 3,
    [int]$MaxLaps = 3
)

if (-not (Get-Module -ListAvailable -Name powershell-yaml)) {
    Write-Host "Installing powershell-yaml..."
    Install-Module -Name powershell-yaml -Force -Scope CurrentUser
}
Import-Module powershell-yaml

$imageName = "f1tenth_gym_ros"
$repoRoot = (Get-Location).Path
$resultsRoot = Join-Path $repoRoot "results"
$configFile = Join-Path (Join-Path $repoRoot "config") "manager.yaml"

# Default values
$numSimulations = 3
$overtakeMargin = 3.0

# Read from manager.yaml if it exists
if (Test-Path $configFile) {
    Write-Host "Reading configuration from manager.yaml..."
    
    try {
        $yaml = Get-Content $configFile | ConvertFrom-Yaml
        
        $numSimulations = [int]$yaml.num_simulations
        $overtakeMargin = [float]$yaml.overtake_margin

        if ($yaml.num_sessions) {
            $NumSessions = [int]$yaml.num_sessions
        }
        if ($yaml.max_laps) {
            $MaxLaps = [int]$yaml.max_laps
        }
    } catch {
        Write-Host "Warning: Could not parse YAML. Using defaults."
    }
} else {
    Write-Host "Warning: manager.yaml not found, using defaults"
}

Write-Host "Configuration: NumSessions=$NumSessions, MaxLaps=$MaxLaps, NumSimulations=$numSimulations, OvertakeMargin=$overtakeMargin"
Write-Host ""
Write-Host "[1/4] Creating results root..."
New-Item -ItemType Directory -Force -Path $resultsRoot | Out-Null

Write-Host "[2/4] Building Docker image..."
docker build -t $imageName .

Write-Host "[3/4] Removing old session containers if they exist..."
docker rm -f $(docker ps -aq --filter "name=f1tenth_session_") 2>$null

Write-Host "[4/4] Launching $NumSessions parallel simulation containers..."
for ($i = 1; $i -le $NumSessions; $i++) {
    $containerName = "f1tenth_session_$i"
    $novncName = "novnc_session_$i"  
    $vncPort = 8080 + $i 
    $sessionResults = Join-Path $resultsRoot "session_$i"
    New-Item -ItemType Directory -Force -Path $sessionResults | Out-Null

    # Copy overtake settings to results
    $configSource = Join-Path (Join-Path $repoRoot "config") "opp_overtake_settings.yaml"
    $configDest = Join-Path $sessionResults "opp_overtake_settings.yaml"
    Copy-Item -Path $configSource -Destination $configDest -Force
    Write-Host "Copied overtake settings to $sessionResults"

    Write-Host "Starting Session $i..."

    # remove old containers
    docker rm -f $novncName 2>$null | Out-Null
    docker rm -f $containerName 2>$null | Out-Null

    # launch novnc
    docker run -d `
        --name $novncName `
        -p "${vncPort}:8080" `
        theasp/novnc:latest

    # launch the sim
    $rosDomainId = $i    
    $containerCmd = "/sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh"
    
    docker run -d `
        --name $containerName `
        --link "${novncName}:novnc" `
        -e DISPLAY=novnc:0.0 `
        -e SESSION_ID=$i `
        -e ROS_DOMAIN_ID=$rosDomainId `
        -e MAX_LAPS=$MaxLaps `
        -e NUM_SIMULATIONS=$numSimulations `
        -e OVERTAKE_MARGIN=$overtakeMargin `
        -e CONFIG_FILE="/sim_ws/src/f1tenth_gym_ros/config/manager.yaml" `
        -e RESULTS_DIR="/sim_ws/results/session_$i" `
        -e ATTACH_TMUX=0 `
        -v "${repoRoot}:/sim_ws/src/f1tenth_gym_ros" `
        -v "${sessionResults}:/sim_ws/results/session_$i" `
        $imageName `
        $containerCmd
}

Write-Host ""
Write-Host "All sessions launched."
Write-Host "Check running containers with: docker ps"
Write-Host "Check logs with: docker logs -f f1tenth_session_1"
Write-Host "Dashboard: file://$($repoRoot.Replace('\', '/'))/dashboard.html" -ForegroundColor Cyan