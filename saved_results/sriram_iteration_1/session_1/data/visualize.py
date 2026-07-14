#!/usr/bin/env python3
"""
Corrected F1TENTH telemetry and overtake visualizer (Multi-CSV Edition).

Core behavior:
- Loads ALL telemetry CSVs in a target directory.
- Processes each file individually to prevent cross-file teleport glitches.
- Aggregates the data into a unified dashboard.
- Modifies visualizations to exclusively show Opponent overtakes (Ego hidden).
- Produces one four-panel figure:
    1. Track map and OPPONENT overtake locations
    2. Spatial 2D Heatmap of Close-Quarters Battle Zones
    3. Statistics Table (Overtake amounts)
    4. Time spent ahead pie chart
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# USER CONFIGURATION
# ---------------------------------------------------------------------------
TRACK_LENGTH_M = None  
LEADER_HYSTERESIS_M = 0.5
RESET_JUMP_DISTANCE_M = 4.0
RESET_IMPLIED_SPEED_MPS = 30.0
RESET_CLUSTER_SECONDS = 0.30
OVERTAKE_TARGET = "opp"  
OVERTAKE_TARGET_GAP_M = 5.0
TRACK_HEATMAP_BINS = 18
CLOSE_ENCOUNTER_GAP_M = 10.0 

OUTPUT_FILENAME = "aggregated_telemetry_dashboard.png"
SHOW_PLOT = True

REQUIRED_COLUMNS = {
    "timestamp", "lap", "ego_x", "ego_y", "ego_vel", "ego_s", "ego_d",
    "opp_x", "opp_y", "opp_vel", "opp_s", "opp_d", "rel_s",
}

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize all F1TENTH telemetry CSVs in a folder.")
    parser.add_argument("--dir", default=".", help="Directory containing the CSVs (defaults to current folder).")
    parser.add_argument("--track-length", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()

def load_and_clean(csv_filepath: Path) -> tuple[pd.DataFrame, int, float]:
    df = pd.read_csv(csv_filepath, low_memory=False)

    numeric_columns = sorted(REQUIRED_COLUMNS.intersection(df.columns))
    for column in numeric_columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")

    df = df.dropna(subset=["timestamp"])
    if not df["timestamp"].is_monotonic_increasing:
        df = df.sort_values("timestamp", kind="stable")
    df = df.reset_index(drop=True)

    zero_columns = ["ego_x", "ego_y", "ego_s", "ego_d", "opp_x", "opp_y", "opp_s", "opp_d"]
    zero_mask = df[zero_columns].fillna(0.0).abs().le(1e-9).all(axis=1).to_numpy()

    first_valid = 0
    while first_valid < len(df) and bool(zero_mask[first_valid]):
        first_valid += 1

    startup_zero_rows = first_valid
    startup_zero_duration = 0.0

    if startup_zero_rows and first_valid < len(df):
        startup_zero_duration = float(df["timestamp"].iloc[first_valid] - df["timestamp"].iloc[0])
        df = df.iloc[first_valid:].copy().reset_index(drop=True)

    if not df.empty:
        df["time_sec"] = df["timestamp"] - float(df["timestamp"].iloc[0])
        df["time_diff"] = df["timestamp"].diff().replace([np.inf, -np.inf], np.nan).clip(lower=0).fillna(0.0)
    
    return df, startup_zero_rows, startup_zero_duration

def estimate_track_length(df: pd.DataFrame) -> float:
    values = pd.concat([df["ego_s"], df["opp_s"]], ignore_index=True)
    maximum = float(values.replace([np.inf, -np.inf], np.nan).max())
    return maximum * 1.001 if maximum > 0 else 360.0

def wrap_gap(raw_gap: pd.Series | np.ndarray, track_length: float):
    return ((raw_gap + track_length / 2.0) % track_length) - track_length / 2.0

def add_wrapped_relative_gap(df: pd.DataFrame, track_length: float) -> None:
    raw_gap = df["opp_s"] - df["ego_s"]
    df["rel_s_wrapped"] = wrap_gap(raw_gap, track_length)

def detect_reset_clusters(df: pd.DataFrame) -> tuple[pd.Series, pd.DataFrame]:
    dt = df["timestamp"].diff()
    ego_jump = np.hypot(df["ego_x"].diff(), df["ego_y"].diff())
    opp_jump = np.hypot(df["opp_x"].diff(), df["opp_y"].diff())

    with np.errstate(divide="ignore", invalid="ignore"):
        ego_implied_speed = ego_jump / dt
        opp_implied_speed = opp_jump / dt

    candidate = (
        (ego_jump > RESET_JUMP_DISTANCE_M) | (opp_jump > RESET_JUMP_DISTANCE_M) |
        (ego_implied_speed > RESET_IMPLIED_SPEED_MPS) | (opp_implied_speed > RESET_IMPLIED_SPEED_MPS)
    ).fillna(False)

    indices = np.flatnonzero(candidate.to_numpy())
    events: list[dict] = []

    if indices.size:
        groups: list[list[int]] = [[int(indices[0])]]
        for raw_idx in indices[1:]:
            idx = int(raw_idx)
            previous = groups[-1][-1]
            if float(df["timestamp"].iloc[idx] - df["timestamp"].iloc[previous]) <= RESET_CLUSTER_SECONDS:
                groups[-1].append(idx)
            else:
                groups.append([idx])

        for event_id, group in enumerate(groups, start=1):
            events.append({
                "reset_id": event_id, "start_idx": max(0, group[0] - 1), "end_idx": group[-1],
                "start_time_sec": float(df["time_sec"].iloc[max(0, group[0] - 1)]),
            })

    reset_events = pd.DataFrame(events, columns=["reset_id", "start_idx", "end_idx", "start_time_sec"])
    reset_mask = pd.Series(False, index=df.index)
    for event in events:
        reset_mask.loc[event["start_idx"] : event["end_idx"]] = True

    return reset_mask, reset_events

def stable_leader(gap: float, current_leader: str | None) -> str | None:
    if not math.isfinite(gap): return current_leader
    if gap >= LEADER_HYSTERESIS_M: return "opp"
    if gap <= -LEADER_HYSTERESIS_M: return "ego"
    return current_leader

def detect_natural_overtakes(df: pd.DataFrame, reset_mask: pd.Series, reset_events: pd.DataFrame) -> pd.DataFrame:
    overtakes: list[dict] = []
    current_leader: str | None = None
    previous_reset = False
    reset_end_indices = set(reset_events["end_idx"].astype(int).tolist() if not reset_events.empty else [])

    for idx in range(len(df)):
        gap = float(df["rel_s_wrapped"].iloc[idx])
        if reset_mask.iloc[idx]:
            previous_reset = True
            continue

        if previous_reset or (idx - 1) in reset_end_indices:
            current_leader = stable_leader(gap, None)
            previous_reset = False
            continue

        new_leader = stable_leader(gap, current_leader)
        if current_leader is None:
            current_leader = new_leader
            continue

        if new_leader is not None and new_leader != current_leader:
            overtakes.append({
                "time": float(df["time_sec"].iloc[idx]), "overtaker": new_leader,
                "x": float(df["ego_x"].iloc[idx]) if new_leader == "ego" else float(df["opp_x"].iloc[idx]),
                "y": float(df["ego_y"].iloc[idx]) if new_leader == "ego" else float(df["opp_y"].iloc[idx]),
                "gap_m": gap
            })
            current_leader = new_leader

    return pd.DataFrame(overtakes, columns=["time", "overtaker", "x", "y", "gap_m"])

def break_trajectory_at_resets(df: pd.DataFrame, reset_events: pd.DataFrame, x_column: str, y_column: str) -> tuple[np.ndarray, np.ndarray]:
    x, y = df[x_column].to_numpy(dtype=float).copy(), df[y_column].to_numpy(dtype=float).copy()
    if not reset_events.empty:
        for _, event in reset_events.iterrows():
            start_idx = int(event["start_idx"])
            end_idx = int(event["end_idx"])
            if 0 <= start_idx < len(x) and 0 <= end_idx < len(x):
                x[start_idx:end_idx + 2] = np.nan
                y[start_idx:end_idx + 2] = np.nan
    return x, y

def plot_track_and_overtakes(ax: plt.Axes, df: pd.DataFrame, overtakes_df: pd.DataFrame) -> None:
    # We plot the already severed trajectories directly
    ax.plot(df['ego_x_plot'], df['ego_y_plot'], label="Ego trajectory", alpha=0.55, linewidth=1.5, color="gray")
    ax.plot(df['opp_x_plot'], df['opp_y_plot'], label="Opponent trajectory", alpha=0.35, linewidth=1.0, color="blue")

    if not overtakes_df.empty:
        # FILTER: Only show opponent overtakes!
        opp_pts = overtakes_df[overtakes_df["overtaker"] == "opp"]
        if not opp_pts.empty: 
            ax.scatter(opp_pts["x"], opp_pts["y"], marker="v", s=100, color="red", edgecolors="black", label="Opponent Overtakes", zorder=5)

    ax.set_title("Track Map: Opponent Overtake Locations")
    ax.set_xlabel("X coordinate (m)")
    ax.set_ylabel("Y coordinate (m)")
    ax.axis("equal")
    ax.legend(fontsize=10)

def plot_spatial_heatmap(fig: plt.Figure, ax: plt.Axes, df: pd.DataFrame) -> None:
    battles = df[df['rel_s_wrapped'].abs() < CLOSE_ENCOUNTER_GAP_M]
    
    if battles.empty:
        ax.text(0.5, 0.5, f"No close encounters (<{CLOSE_ENCOUNTER_GAP_M}m)", ha="center", va="center")
        ax.set_title('Heatmap: Close-Quarters Battle Zones')
        return

    hb = ax.hexbin(battles['ego_x'], battles['ego_y'], gridsize=40, cmap='magma', mincnt=1)
    cb = fig.colorbar(hb, ax=ax, fraction=0.045, pad=0.04)
    cb.set_label(f'Density of Close Encounters (< {CLOSE_ENCOUNTER_GAP_M}m gap)')
    
    ax.plot(df['ego_x_plot'], df['ego_y_plot'], color='black', alpha=0.25, linewidth=1.0, zorder=2) 
    ax.plot(df['opp_x_plot'], df['opp_y_plot'], color='black', alpha=0.25, linewidth=1.0, zorder=2) 
    
    ax.set_title('Spatial Heatmap: Battle Zones\n(Bright spots = cars get stuck fighting)')
    ax.set_xlabel('X coordinate (m)')
    ax.set_ylabel('Y coordinate (m)')
    ax.axis("equal") 

def plot_overtake_table(ax: plt.Axes, files_count: int, opp_overtakes: int, total_time: float) -> None:
    ax.axis('off')
    
    table_data = [
        ["Total Files Analyzed", f"{files_count}"],
        ["Total Session Time", f"{total_time / 60:.2f} min"],
        ["Opponent Overtakes", f"{opp_overtakes}"]
    ]
    
    table = ax.table(
        cellText=table_data, 
        colLabels=["Metric", "Value"], 
        loc='center', 
        cellLoc='center', 
        colWidths=[0.5, 0.3]
    )
    
    table.auto_set_font_size(False)
    table.set_fontsize(14)
    table.scale(1, 2.5)
    
    # Styling the table
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#cccccc')
        if row == 0:
            cell.set_text_props(weight='bold', color='white')
            cell.set_facecolor('#333333')
        elif col == 0:
            cell.set_text_props(weight='bold')
            cell.set_facecolor('#f5f5f5')
            
    ax.set_title("Simulation Session Statistics", fontsize=16, weight="bold")

def plot_time_ahead_pie(ax: plt.Axes, ego_time: float, opp_time: float, tied_time: float) -> None:
    values = [ego_time, opp_time, tied_time]
    labels = ["Ego ahead", "Opponent ahead", f"Within ±{LEADER_HYSTERESIS_M:.1f} m"]
    colors = ['#2ecc71', '#e74c3c', '#95a5a6']

    if sum(values) > 0:
        ax.pie(values, labels=labels, colors=colors, autopct=lambda pct: f"{pct:.1f}%" if pct >= 1.0 else "", startangle=90)
    else:
        ax.text(0.5, 0.5, "No duration data", ha="center", va="center")
    
    ax.set_title("Total Time Spent Ahead (Aggregated)")

def process_directory(target_dir: Path, track_length: float | None, output_path: Path, show_plot: bool) -> None:
    csv_files = list(target_dir.glob("*.csv"))
    
    # Filter out any summary CSVs generated by previous runs of this script
    csv_files = [f for f in csv_files if not f.name.endswith((
        "_natural_overtakes.csv", "_reset_events.csv", "session_summary.csv"
    ))]

    if not csv_files:
        print(f"No valid telemetry CSVs found in {target_dir}")
        return

    print(f"Found {len(csv_files)} CSV files. Processing...")

    all_dfs = []
    all_overtakes = []
    
    total_ego_time = 0.0
    total_opp_time = 0.0
    total_tied_time = 0.0
    total_session_time = 0.0
    
    # Needs a track length estimation if None, base it on the first valid file
    if track_length is None:
        for f in csv_files:
            preview_df, _, _ = load_and_clean(f)
            if not preview_df.empty:
                track_length = estimate_track_length(preview_df)
                break
        if track_length is None: track_length = 360.0 # Ultimate fallback

    for f in csv_files:
        df, _, _ = load_and_clean(f)
        if df.empty: continue
        
        add_wrapped_relative_gap(df, track_length)
        reset_mask, reset_events = detect_reset_clusters(df)
        overtakes_df = detect_natural_overtakes(df, reset_mask, reset_events)
        
        # Pre-process severed trajectories so we can safely concatenate the dataframes later
        df['ego_x_plot'], df['ego_y_plot'] = break_trajectory_at_resets(df, reset_events, "ego_x", "ego_y")
        df['opp_x_plot'], df['opp_y_plot'] = break_trajectory_at_resets(df, reset_events, "opp_x", "opp_y")
        
        # Aggregate times
        ego_t = float(df.loc[df["rel_s_wrapped"] < -LEADER_HYSTERESIS_M, "time_diff"].sum())
        opp_t = float(df.loc[df["rel_s_wrapped"] > LEADER_HYSTERESIS_M, "time_diff"].sum())
        tied_t = float(df.loc[df["rel_s_wrapped"].abs() <= LEADER_HYSTERESIS_M, "time_diff"].sum())
        
        total_ego_time += ego_t
        total_opp_time += opp_t
        total_tied_time += tied_t
        total_session_time += float(df['time_sec'].iloc[-1]) if not df.empty else 0.0

        # Only append Opponent overtakes for the master list
        if not overtakes_df.empty:
            opp_only = overtakes_df[overtakes_df['overtaker'] == 'opp'].copy()
            if not opp_only.empty:
                all_overtakes.append(opp_only)

        # Append a row of NaNs to the dataframe before storing it. 
        # This guarantees that when we plot the aggregated data, lines don't connect across different files!
        nan_row = pd.DataFrame(np.nan, index=[0], columns=df.columns)
        all_dfs.append(pd.concat([df, nan_row], ignore_index=True))

    # Combine everything for plotting
    master_df = pd.concat(all_dfs, ignore_index=True) if all_dfs else pd.DataFrame()
    master_overtakes = pd.concat(all_overtakes, ignore_index=True) if all_overtakes else pd.DataFrame()
    total_opp_overtakes = len(master_overtakes)

    print("\n" + "=" * 50)
    print("🏁 AGGREGATED TELEMETRY SUMMARY 🏁")
    print("=" * 50)
    print(f"Files Processed: {len(csv_files)}")
    print(f"Total Session Time: {total_session_time / 60:.2f} minutes")
    print(f"Opponent Overtake Events: {total_opp_overtakes}")
    print("=" * 50 + "\n")

    # Start Visuals
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(16, 12))

    # Panel 1: Track Map (Opponent only)
    ax1 = fig.add_subplot(2, 2, 1)
    if not master_df.empty: plot_track_and_overtakes(ax1, master_df, master_overtakes)

    # Panel 2: Heatmap
    ax2 = fig.add_subplot(2, 2, 2)
    if not master_df.empty: plot_spatial_heatmap(fig, ax2, master_df) 

    # Panel 3: Statistics Table (Replaces Timeline)
    ax3 = fig.add_subplot(2, 2, 3)
    plot_overtake_table(ax3, len(csv_files), total_opp_overtakes, total_session_time)

    # Panel 4: Pie Chart
    ax4 = fig.add_subplot(2, 2, 4)
    plot_time_ahead_pie(ax4, total_ego_time, total_opp_time, total_tied_time)

    fig.suptitle("Aggregated F1TENTH Telemetry Analysis", fontsize=18, weight='bold')
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved aggregated dashboard to: {output_path}")

    if show_plot: plt.show()
    else: plt.close(fig)

def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    
    target_dir = Path(args.dir).expanduser().resolve()
    if not target_dir.is_dir():
        print(f"Error: Directory '{target_dir}' does not exist.")
        return 1

    output_path = Path(args.output).expanduser() if args.output else target_dir / OUTPUT_FILENAME
    if not output_path.is_absolute(): output_path = target_dir / output_path
    
    process_directory(target_dir, args.track_length, output_path.resolve(), SHOW_PLOT and not args.no_show)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())