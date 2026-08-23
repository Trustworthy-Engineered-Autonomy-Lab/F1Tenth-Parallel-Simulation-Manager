#!/usr/bin/env python3
"""
F1TENTH telemetry and opponent-overtake visualizer, multi-CSV edition.
Decoupled Ego/Opponent validation to prevent telemetry drops from erasing session metrics.
Updated to safely parse dropped logger columns, read data-driven overtakes, and restore heatmap tracking.
"""
from __future__ import annotations

import argparse
import ast
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Fix for OverflowError: Exceeded cell block limit in Agg
plt.rcParams['agg.path.chunksize'] = 10000

# ---------------------------------------------------------------------------
# USER CONFIGURATION
# ---------------------------------------------------------------------------
TRACK_LENGTH_M = None

# Overtake detection tuning (Kept for backwards compatibility and pie charts)
LEADER_HYSTERESIS_M = 0.5
OVERTAKE_RESET_GAP_M = 0.0

# Plot-break tuning
PLOT_BREAK_TIME_GAP_SECONDS = 2.0
PLOT_BREAK_MIN_XY_JUMP_M = 25.0

# Plot tuning
CLOSE_ENCOUNTER_GAP_M = 10.0
HEATMAP_GRIDSIZE = 40
OUTPUT_FILENAME = "aggregated_telemetry_dashboard.png"
SHOW_PLOT = True

REQUIRED_COLUMNS = {
    "timestamp", "lap", "ego_x", "ego_y", "ego_vel", "ego_s", "ego_d",
    "opp_x", "opp_y", "opp_vel", "opp_s", "opp_d", "rel_s", "overtake_num"
}

SUMMARY_SUFFIXES = (
    "_natural_overtakes.csv",
    "_reset_events.csv",
    "_overtakes.csv",
    "_plot_breaks.csv",
    "session_summary.csv",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize F1TENTH telemetry CSVs.")
    parser.add_argument("--dir", default=".", help="Directory containing CSVs; defaults to current folder.")
    parser.add_argument("--track-length", type=float, default=None, help="Track length in meters.")
    parser.add_argument("--output", default=None, help="Output PNG path.")
    parser.add_argument("--no-show", action="store_true", help="Save the figure without opening a GUI window.")
    parser.add_argument("--write-debug-csv", action="store_true", help="Write detected events to CSVs.")
    return parser.parse_args()


def load_and_clean(csv_filepath: Path) -> pd.DataFrame:
    cleaned_rows = []
    
    # Safely parse the CSV row-by-row to catch the dropped overtake_num column
    with open(csv_filepath, 'r') as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return pd.DataFrame()

        # Clean potential logging typos in the header
        header = [x.strip().replace("csvtimestamp", "timestamp")
                  .replace("overtake num", "overtake_num")
                  .replace("overtake_number", "overtake_num") for x in header]

        for row in reader:
            if not row:
                continue
                
            # If the logger dropped a column (19 items instead of 20),
            # pad index 2 (overtake_num) with a '0' to prevent ego_x from shifting left.
            if len(row) == len(header) - 1:
                row.insert(2, '0')
                
            cleaned_rows.append(row)

    df = pd.DataFrame(cleaned_rows, columns=header)

    missing_columns = sorted(REQUIRED_COLUMNS.difference(df.columns))
    if missing_columns:
        raise ValueError(f"{csv_filepath.name} is missing required columns: {missing_columns}")

    for column in sorted(REQUIRED_COLUMNS):
        df[column] = pd.to_numeric(df[column], errors="coerce")

    # Wipe Ego 0.0 zero-states (prevents gray starbursts to the map origin)
    ego_zeros = (df["ego_x"].fillna(0) == 0.0) & (df["ego_y"].fillna(0) == 0.0)
    df.loc[ego_zeros, ["ego_x", "ego_y", "ego_s", "ego_d", "ego_vel"]] = np.nan

    # Wipe Opp 0.0 zero-states (prevents blue starbursts)
    opp_zeros = (df["opp_x"].fillna(0) == 0.0) & (df["opp_y"].fillna(0) == 0.0)
    df.loc[opp_zeros, ["opp_x", "opp_y", "opp_s", "opp_d", "opp_vel"]] = np.nan

    # Parse trajectory strings into Python coordinate lists safely
    if "imm_trajectory" in df.columns:
        def parse_trajectory(val):
            if pd.isna(val) or val == "[]":
                return []
            if isinstance(val, str):
                try:
                    return ast.literal_eval(val)
                except (ValueError, SyntaxError):
                    return []
            if isinstance(val, list):
                return val
            return []
        df["imm_trajectory"] = df["imm_trajectory"].apply(parse_trajectory)

    # Format the imm_active flag purely for trajectory rendering, without wiping the opponent track
    if "imm_active" in df.columns:
        df["imm_active"] = df["imm_active"].astype(str).str.lower().isin(["true", "1", "t", "yes"])

    df = df.dropna(subset=["timestamp"]).copy()
    if not df["timestamp"].is_monotonic_increasing:
        df = df.sort_values("timestamp", kind="stable")
    df = df.reset_index(drop=True)

    if not df.empty:
        df["time_sec"] = df["timestamp"] - float(df["timestamp"].iloc[0])
        df["time_diff"] = df["timestamp"].diff().replace([np.inf, -np.inf], np.nan).clip(lower=0).fillna(0.0)

    return df


def wrap_gap(raw_gap: pd.Series | np.ndarray, track_length: float) -> pd.Series | np.ndarray:
    return ((raw_gap + track_length / 2.0) % track_length) - track_length / 2.0


def estimate_track_length(all_dfs: list[pd.DataFrame]) -> float:
    values = []
    for df in all_dfs:
        if not df.empty:
            values.append(df["ego_s"])
            values.append(df["opp_s"])
    if not values:
        return 360.0

    combined = pd.concat(values, ignore_index=True).replace([np.inf, -np.inf], np.nan).dropna()
    if combined.empty:
        return 360.0

    maximum = float(combined.max())
    return maximum * 1.001 if maximum > 0 else 360.0


def detect_opponent_overtakes(df: pd.DataFrame) -> pd.DataFrame:
    columns = [
        "time", "overtaker", "x", "y", "gap_m", "xy_gap_m", "d_gap_m",
        "event_idx", "episode_start_time", "episode_end_time"
    ]
    if df.empty or "overtake_num" not in df.columns:
        return pd.DataFrame(columns=columns)

    # Forward fill to handle any sporadic NaNs in telemetry logs, filling initial NaNs with 0
    ot_series = df["overtake_num"].ffill().fillna(0)
    
    # Check for increments in the logged overtake counter
    overtake_mask = ot_series.diff() > 0
    overtake_indices = df.index[overtake_mask].tolist()

    overtakes: list[dict] = []
    last_episode_start = np.nan

    for idx in overtake_indices:
        row = df.iloc[idx]
        
        # Fallback to Ego position if opponent track is briefly invalid during the event log
        opp_x = float(row["opp_x"]) if pd.notna(row["opp_x"]) else float(row["ego_x"])
        opp_y = float(row["opp_y"]) if pd.notna(row["opp_y"]) else float(row["ego_y"])
        
        xy_gap = math.hypot(
            float(row["opp_x"] - row["ego_x"]) if pd.notna(row["opp_x"]) and pd.notna(row["ego_x"]) else 0.0,
            float(row["opp_y"] - row["ego_y"]) if pd.notna(row["opp_y"]) and pd.notna(row["ego_y"]) else 0.0,
        )
        d_gap = abs(float(row["opp_d"] - row["ego_d"])) if pd.notna(row["opp_d"]) and pd.notna(row["ego_d"]) else 0.0
        
        overtakes.append({
            "time": float(row["time_sec"]),
            "overtaker": "opp",
            "x": opp_x,
            "y": opp_y,
            "gap_m": float(row.get("rel_s_wrapped", 0.0)),
            "xy_gap_m": float(xy_gap),
            "d_gap_m": float(d_gap),
            "event_idx": int(idx),
            "episode_start_time": float(last_episode_start),
            "episode_end_time": float(row["time_sec"]),
        })
        last_episode_start = float(row["time_sec"])

    return pd.DataFrame(overtakes, columns=columns)


def plot_track_and_overtakes(ax: plt.Axes, df: pd.DataFrame, overtakes_df: pd.DataFrame) -> None:
    ax.plot(df["ego_x_plot"], df["ego_y_plot"], label="Ego trajectory", alpha=0.55, linewidth=1.5, color="gray")
    ax.plot(df["opp_x_plot"], df["opp_y_plot"], label="Opponent trajectory", alpha=0.35, linewidth=1.0, color="blue")

    # Render active IMM Trajectories
    if "imm_trajectory" in df.columns and "imm_active" in df.columns:
        active_df = df[df["imm_active"] == True]
        if not active_df.empty:
            step = max(1, len(active_df) // 150)
            plotted_legend = False
            
            for _, row in active_df.iloc[::step].iterrows():
                traj = row["imm_trajectory"]
                if isinstance(traj, list) and len(traj) > 0:
                    xs = [pt[0] for pt in traj]
                    ys = [pt[1] for pt in traj]
                    ax.plot(xs, ys, color="orange", alpha=0.4, linewidth=1.0, zorder=2)
                    plotted_legend = True

            if plotted_legend:
                ax.plot([], [], color="orange", alpha=0.6, linewidth=1.5, label="Active IMM Trajectories")

    if not overtakes_df.empty:
        ax.scatter(
            overtakes_df["x"], overtakes_df["y"],
            marker="v", s=110, color="red", edgecolors="black",
            label="Logged Overtake Events", zorder=5,
        )
        if len(overtakes_df) <= 40:
            for event_number, (_, row) in enumerate(overtakes_df.iterrows(), start=1):
                ax.annotate(
                    str(event_number),
                    (row["x"], row["y"]),
                    textcoords="offset points",
                    xytext=(6, 6),
                    fontsize=9,
                    weight="bold",
                )

    ax.set_title("Track Map: Overtakes (Data-Driven)")
    ax.set_xlabel("X coordinate (m)")
    ax.set_ylabel("Y coordinate (m)")
    ax.axis("equal")
    ax.legend(fontsize=10)


def plot_spatial_heatmap(fig: plt.Figure, ax: plt.Axes, df: pd.DataFrame) -> None:
    # Only map heat zones when both cars are actively tracked
    valid = df.get("both_valid", pd.Series(False, index=df.index)).fillna(False).astype(bool)
    battles = df[
        valid & df["rel_s_wrapped"].abs().lt(CLOSE_ENCOUNTER_GAP_M)
    ].copy()

    if battles.empty:
        ax.text(0.5, 0.5, f"No clean close encounters (<{CLOSE_ENCOUNTER_GAP_M} m)", ha="center", va="center")
        ax.set_title("Heatmap: Clean Close-Quarters Battle Zones")
        return

    battles["battle_x"] = (battles["ego_x"] + battles["opp_x"]) / 2.0
    battles["battle_y"] = (battles["ego_y"] + battles["opp_y"]) / 2.0

    hb = ax.hexbin(battles["battle_x"], battles["battle_y"], gridsize=HEATMAP_GRIDSIZE, cmap="magma", mincnt=1)
    cb = fig.colorbar(hb, ax=ax, fraction=0.045, pad=0.04)
    cb.set_label(f"Close-encounter density (< {CLOSE_ENCOUNTER_GAP_M} m gap)")

    ax.plot(df["ego_x_plot"], df["ego_y_plot"], color="black", alpha=0.25, linewidth=1.0, zorder=2)
    ax.plot(df["opp_x_plot"], df["opp_y_plot"], color="black", alpha=0.25, linewidth=1.0, zorder=2)

    ax.set_title("Spatial Heatmap: Battle Zones")
    ax.set_xlabel("X coordinate (m)")
    ax.set_ylabel("Y coordinate (m)")
    ax.axis("equal")


def plot_overtake_table(ax: plt.Axes, files_count: int, opp_overtakes: int, total_time: float, plot_breaks: int) -> None:
    ax.axis("off")

    table_data = [
        ["Total Files Analyzed", f"{files_count}"],
        ["Total Session Time", f"{total_time / 60:.2f} min"],
        ["Overtake Events (Logged Data)", f"{opp_overtakes}"],
        ["Plot Line Breaks", f"{plot_breaks}"],
    ]

    table = ax.table(
        cellText=table_data,
        colLabels=["Metric", "Value"],
        loc="center",
        cellLoc="center",
        colWidths=[0.55, 0.3],
    )

    table.auto_set_font_size(False)
    table.set_fontsize(13)
    table.scale(1, 2.35)

    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if row == 0:
            cell.set_text_props(weight="bold", color="white")
            cell.set_facecolor("#333333")
        elif col == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#f5f5f5")

    ax.set_title("Simulation Session Statistics", fontsize=16, weight="bold")


def plot_time_ahead_pie(ax: plt.Axes, ego_time: float, opp_time: float, tied_time: float) -> None:
    values = [ego_time, opp_time, tied_time]
    labels = ["Ego ahead", "Opponent ahead", f"Within ±{LEADER_HYSTERESIS_M:.1f} m"]
    colors = ["#2ecc71", "#e74c3c", "#95a5a6"]

    if sum(values) > 0:
        ax.pie(
            values,
            labels=labels,
            colors=colors,
            autopct=lambda pct: f"{pct:.1f}%" if pct >= 1.0 else "",
            startangle=90,
        )
    else:
        ax.text(0.5, 0.5, "No duration data", ha="center", va="center")

    ax.set_title("Total Tracked Interaction Time")


def find_telemetry_csvs(target_dir: Path) -> tuple[list[Path], list[tuple[str, str]]]:
    csv_files: list[Path] = []
    skipped: list[tuple[str, str]] = []

    for f in sorted(target_dir.glob("*.csv")):
        if f.name.endswith(SUMMARY_SUFFIXES):
            skipped.append((f.name, "summary/debug suffix"))
            continue
        try:
            # Safely check headers with csv reader
            with open(f, 'r') as fp:
                reader = csv.reader(fp)
                header = next(reader)
            header = [x.strip().replace("csvtimestamp", "timestamp").replace("overtake num", "overtake_num").replace("overtake_number", "overtake_num") for x in header]
        except Exception as exc:
            skipped.append((f.name, f"could not read header: {exc}"))
            continue

        missing = sorted(REQUIRED_COLUMNS.difference(header))
        if missing:
            skipped.append((f.name, f"not telemetry; missing {missing}"))
            continue
        csv_files.append(f)

    return csv_files, skipped


def process_directory(
    target_dir: Path,
    track_length: float | None,
    output_path: Path,
    show_plot: bool,
    write_debug_csv: bool,
) -> None:
    csv_files, skipped_csvs = find_telemetry_csvs(target_dir)

    if skipped_csvs:
        print("Skipped non-telemetry/debug CSVs:")
        for name, reason in skipped_csvs:
            print(f"  - {name}: {reason}")

    if not csv_files:
        print(f"No valid telemetry CSVs found in {target_dir}")
        return

    loaded: list[tuple[Path, pd.DataFrame]] = []
    for f in csv_files:
        df = load_and_clean(f)
        if not df.empty:
            loaded.append((f, df))

    if not loaded:
        print(f"No non-empty telemetry CSVs found in {target_dir}")
        return

    if track_length is None:
        track_length = estimate_track_length([df for _, df in loaded])

    print(f"Found {len(loaded)} non-empty CSV files. Processing...")
    print(f"Using track length: {track_length:.3f} m")

    all_dfs: list[pd.DataFrame] = []
    all_overtakes: list[pd.DataFrame] = []

    total_ego_time = 0.0
    total_opp_time = 0.0
    total_tied_time = 0.0
    total_session_time = 0.0
    total_plot_breaks = 0

    for f, df in loaded:
        # Wrap gap based on exact Track length
        raw_gap = df["opp_s"] - df["ego_s"]
        df["rel_s_wrapped"] = wrap_gap(raw_gap, track_length)

        # 1. Establish Math Validity (are positions NaN?)
        df["ego_valid"] = df[["ego_x", "ego_y", "ego_s"]].notna().all(axis=1)
        df["opp_valid"] = df[["opp_x", "opp_y", "opp_s"]].notna().all(axis=1)
        df["both_valid"] = df["ego_valid"] & df["opp_valid"]

        # 2. Establish Plot Breaks (Jumps or Timelapses)
        ego_xy_step = pd.Series(np.hypot(df["ego_x"].diff(), df["ego_y"].diff()), index=df.index)
        opp_xy_step = pd.Series(np.hypot(df["opp_x"].diff(), df["opp_y"].diff()), index=df.index)
        time_gap = df["timestamp"].diff() > PLOT_BREAK_TIME_GAP_SECONDS
        
        df["ego_plot_break"] = (~df["ego_valid"]) | (ego_xy_step > PLOT_BREAK_MIN_XY_JUMP_M) | time_gap
        df["opp_plot_break"] = (~df["opp_valid"]) | (opp_xy_step > PLOT_BREAK_MIN_XY_JUMP_M) | time_gap

        # Apply NaNs specifically for Matplotlib to physically sever the lines
        df["ego_x_plot"] = df["ego_x"].where(~df["ego_plot_break"], np.nan)
        df["ego_y_plot"] = df["ego_y"].where(~df["ego_plot_break"], np.nan)
        df["opp_x_plot"] = df["opp_x"].where(~df["opp_plot_break"], np.nan)
        df["opp_y_plot"] = df["opp_y"].where(~df["opp_plot_break"], np.nan)

        # Count discrete break triggers (transitioning from valid to broken)
        ego_break_events = df["ego_plot_break"] & ~df["ego_plot_break"].shift(1, fill_value=False)
        opp_break_events = df["opp_plot_break"] & ~df["opp_plot_break"].shift(1, fill_value=False)
        total_plot_breaks += int((ego_break_events | opp_break_events).sum())

        overtakes_df = detect_opponent_overtakes(df)
        if not overtakes_df.empty:
            overtakes_df = overtakes_df.copy()
            overtakes_df["source_file"] = f.name
            all_overtakes.append(overtakes_df)
            event_times = ", ".join(f"{t:.2f}s" for t in overtakes_df["time"].tolist())
            print(f"{f.name}: {len(overtakes_df)} overtakes flagged from data at {event_times}")
        else:
            print(f"{f.name}: 0 logged overtakes detected")

        # Session time is strictly tied to Ego valid driving time
        total_session_time += float(df.loc[df["ego_valid"], "time_diff"].sum())
        
        # Overtake metrics are evaluated only when both cars are actively tracked
        both = df["both_valid"]
        total_ego_time += float(df.loc[both & (df["rel_s_wrapped"] < -LEADER_HYSTERESIS_M), "time_diff"].sum())
        total_opp_time += float(df.loc[both & (df["rel_s_wrapped"] > LEADER_HYSTERESIS_M), "time_diff"].sum())
        total_tied_time += float(df.loc[both & (df["rel_s_wrapped"].abs() <= LEADER_HYSTERESIS_M), "time_diff"].sum())

        nan_row = pd.DataFrame(np.nan, index=[0], columns=df.columns)
        all_dfs.append(pd.concat([df, nan_row], ignore_index=True))

    master_df = pd.concat(all_dfs, ignore_index=True) if all_dfs else pd.DataFrame()
    master_overtakes = pd.concat(all_overtakes, ignore_index=True) if all_overtakes else pd.DataFrame()

    if not master_overtakes.empty:
        master_overtakes = master_overtakes.reset_index(drop=True)
        master_overtakes["event_number"] = np.arange(1, len(master_overtakes) + 1)

    if write_debug_csv:
        debug_prefix = output_path.with_suffix("")
        master_overtakes.to_csv(debug_prefix.with_name(debug_prefix.name + "_overtakes.csv"), index=False)

    print("\n" + "=" * 54)
    print("🏁 AGGREGATED TELEMETRY SUMMARY 🏁")
    print("=" * 54)
    print(f"Files Processed: {len(loaded)}")
    print(f"Total Session Time: {total_session_time / 60:.2f} minutes")
    print(f"Total Overtake Events (Logged Data): {len(master_overtakes)}")
    print(f"Plot Line Breaks Inserted: {total_plot_breaks}")
    print("=" * 54 + "\n")

    plt.style.use("seaborn-v0_8-darkgrid")
    fig = plt.figure(figsize=(16, 12))

    ax1 = fig.add_subplot(2, 2, 1)
    if not master_df.empty:
        plot_track_and_overtakes(ax1, master_df, master_overtakes)

    ax2 = fig.add_subplot(2, 2, 2)
    if not master_df.empty:
        plot_spatial_heatmap(fig, ax2, master_df)

    ax3 = fig.add_subplot(2, 2, 3)
    plot_overtake_table(ax3, len(loaded), len(master_overtakes), total_session_time, total_plot_breaks)

    ax4 = fig.add_subplot(2, 2, 4)
    plot_time_ahead_pie(ax4, total_ego_time, total_opp_time, total_tied_time)

    fig.suptitle("Aggregated F1TENTH Telemetry Analysis", fontsize=18, weight="bold")
    plt.tight_layout(rect=(0, 0, 1, 0.96))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved aggregated dashboard to: {output_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    args = parse_args()
    target_dir = Path(args.dir).expanduser().resolve()
    if not target_dir.is_dir():
        print(f"Error: Directory '{target_dir}' does not exist.")
        return 1

    output_path = Path(args.output).expanduser() if args.output else target_dir / OUTPUT_FILENAME
    if not output_path.is_absolute():
        output_path = target_dir / output_path

    process_directory(
        target_dir=target_dir,
        track_length=args.track_length if args.track_length is not None else TRACK_LENGTH_M,
        output_path=output_path.resolve(),
        show_plot=SHOW_PLOT and not args.no_show,
        write_debug_csv=args.write_debug_csv,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
