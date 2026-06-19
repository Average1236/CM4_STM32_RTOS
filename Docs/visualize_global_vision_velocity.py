#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize global vision velocity data with 3-line record format.

Data format:
    Line 1: timestamp
    Line 2: invalid_data global_vision_vx
    Line 3: invalid_data global_vision_vy

Example:
    1781277524434
    0 -2.44758
    0 -1.49242

Generated outputs:
    global_vision_velocity_vx_vy.png
    global_vision_velocity_magnitude.png
    global_vision_velocity_xy_plane.png
    global_vision_velocity_parsed.csv

Usage:
    python visualize_global_vision_velocity.py --input go2Pos_vel_real_2
    python visualize_global_vision_velocity.py --input go2Pos_vel_real_2 --out ./figures
    python visualize_global_vision_velocity.py --input go2Pos_vel_real_2 --bin 0.05
    python visualize_global_vision_velocity.py --input go2Pos_vel_real_2 --no-show
"""

import argparse
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def resolve_input_path(input_arg: str) -> Path:
    input_path = Path(input_arg)
    if input_path.exists():
        return input_path

    script_relative_path = Path(__file__).resolve().parent / input_arg
    if script_relative_path.exists():
        return script_relative_path

    raise FileNotFoundError(
        f"Input file not found: {input_arg}\n"
        f"Tried: {input_path.resolve()} and {script_relative_path}"
    )


def parse_three_line_velocity_file(input_path: Path) -> pd.DataFrame:
    """
    Parse data where every sample consists of 3 lines:
        timestamp
        invalid_value global_vision_vx
        invalid_value global_vision_vy
    """
    lines = input_path.read_text(encoding="utf-8", errors="replace").splitlines()
    lines = [line.strip() for line in lines if line.strip()]

    if len(lines) < 3:
        raise ValueError("Input file has fewer than 3 non-empty lines.")

    usable_line_count = (len(lines) // 3) * 3
    dropped_line_count = len(lines) - usable_line_count

    if dropped_line_count > 0:
        print(
            f"Warning: {dropped_line_count} trailing line(s) ignored because "
            f"the total line count is not divisible by 3."
        )

    records: list[dict[str, float]] = []

    for i in range(0, usable_line_count, 3):
        line_no = i + 1

        ts_line = lines[i]
        vx_line = lines[i + 1].split()
        vy_line = lines[i + 2].split()

        try:
            timestamp = float(ts_line)
        except ValueError as exc:
            raise ValueError(f"Invalid timestamp at line {line_no}: {ts_line}") from exc

        if len(vx_line) < 2:
            raise ValueError(
                f"Line {line_no + 1} should contain at least 2 columns: "
                f"invalid_data global_vision_vx"
            )

        if len(vy_line) < 2:
            raise ValueError(
                f"Line {line_no + 2} should contain at least 2 columns: "
                f"invalid_data global_vision_vy"
            )

        try:
            invalid_vx = float(vx_line[0])
            global_vx = float(vx_line[1])
            invalid_vy = float(vy_line[0])
            global_vy = float(vy_line[1])
        except ValueError as exc:
            raise ValueError(
                f"Invalid numeric data near line {line_no}:\n"
                f"{lines[i]}\n{lines[i + 1]}\n{lines[i + 2]}"
            ) from exc

        records.append(
            {
                "timestamp": timestamp,
                "invalid_x": invalid_vx,
                "global_vision_vx": global_vx,
                "invalid_y": invalid_vy,
                "global_vision_vy": global_vy,
            }
        )

    df = pd.DataFrame(records)

    # Timestamp is usually in ms in this log. Convert to elapsed seconds.
    # If your timestamp is in another unit, modify this section accordingly.
    df["t_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0

    df["global_vision_speed"] = np.sqrt(
        df["global_vision_vx"] ** 2 + df["global_vision_vy"] ** 2
    )

    return df


def downsample_by_time(df: pd.DataFrame, bin_s: float) -> pd.DataFrame:
    if bin_s <= 0:
        return df.copy()

    plot_df = df.copy()
    plot_df["t_bin"] = (plot_df["t_s"] / bin_s).round() * bin_s
    plot_df = plot_df.groupby("t_bin", as_index=False).mean(numeric_only=True)
    return plot_df


def add_interactive_legend(ax: plt.Axes, artists: list[Any]) -> None:
    legend = ax.legend(loc="best")
    legend_map: dict[Any, Any] = {}

    legend_handles = getattr(legend, "legend_handles", None)
    if legend_handles is None:
        legend_handles = legend.legendHandles

    for legend_handle, artist in zip(legend_handles, artists):
        legend_handle.set_picker(True)
        legend_handle.set_pickradius(8)
        legend_handle.set_alpha(1.0 if artist.get_visible() else 0.25)
        legend_map[legend_handle] = artist

    for legend_text, artist in zip(legend.get_texts(), artists):
        legend_text.set_picker(True)
        legend_text.set_alpha(1.0 if artist.get_visible() else 0.25)
        legend_map[legend_text] = artist

    def on_pick(event: Any) -> None:
        artist = legend_map.get(event.artist)
        if artist is None:
            return

        visible = not artist.get_visible()
        artist.set_visible(visible)
        event.artist.set_alpha(1.0 if visible else 0.25)
        ax.figure.canvas.draw_idle()

    ax.figure.canvas.mpl_connect("pick_event", on_pick)

    ax.text(
        0.99,
        0.01,
        "Click legend items to show/hide",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        alpha=0.65,
    )


def plot_vx_vy(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    fig, ax = plt.subplots(figsize=(12, 5))

    x_col = "t_bin" if "t_bin" in plot_df.columns else "t_s"

    (line_vx,) = ax.plot(
        plot_df[x_col],
        plot_df["global_vision_vx"],
        label="global_vision_vx",
    )
    (line_vy,) = ax.plot(
        plot_df[x_col],
        plot_df["global_vision_vy"],
        label="global_vision_vy",
    )

    ax.axhline(0, linestyle="--", linewidth=1)
    ax.set_title("Global vision velocity")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Velocity")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [line_vx, line_vy])

    fig.tight_layout()
    fig.savefig(output_dir / "global_vision_velocity_vx_vy.png", dpi=180)

    if not interactive:
        plt.close(fig)


def plot_speed_magnitude(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    fig, ax = plt.subplots(figsize=(12, 5))

    x_col = "t_bin" if "t_bin" in plot_df.columns else "t_s"

    (line_speed,) = ax.plot(
        plot_df[x_col],
        plot_df["global_vision_speed"],
        label="sqrt(vx^2 + vy^2)",
    )

    ax.set_title("Global vision velocity magnitude")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Speed magnitude")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [line_speed])

    fig.tight_layout()
    fig.savefig(output_dir / "global_vision_velocity_magnitude.png", dpi=180)

    if not interactive:
        plt.close(fig)


def plot_xy_plane(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    fig, ax = plt.subplots(figsize=(7, 7))

    scatter = ax.scatter(
        plot_df["global_vision_vx"],
        plot_df["global_vision_vy"],
        label="global vision velocity",
        s=14,
        alpha=0.75,
    )

    ax.set_title("Global vision velocity in vx-vy plane")
    ax.set_xlabel("global_vision_vx")
    ax.set_ylabel("global_vision_vy")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [scatter])
    ax.axis("equal")

    fig.tight_layout()
    fig.savefig(output_dir / "global_vision_velocity_xy_plane.png", dpi=180)

    if not interactive:
        plt.close(fig)


def print_basic_info(df: pd.DataFrame, input_path: Path) -> None:
    duration_s = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    median_dt_ms = df["timestamp"].diff().median()

    print(f"Input file: {input_path}")
    print(f"Samples: {len(df)}")
    print(f"Duration: {duration_s:.3f} s")
    print(f"Median timestamp interval: {median_dt_ms:.3f} timestamp units")
    print()
    print("Velocity statistics:")
    print(
        df[
            [
                "global_vision_vx",
                "global_vision_vy",
                "global_vision_speed",
            ]
        ]
        .describe()
        .to_string()
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=str,
        default="go2Pos_vel_real_2",
        help="Path to input text file.",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=".",
        help="Output directory for figures and parsed CSV.",
    )
    parser.add_argument(
        "--bin",
        type=float,
        default=0,
        help=(
            "Downsample averaging window in seconds. "
            "Use 0 to disable downsampling. Default: 0.05"
        ),
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Only save output files; do not open interactive plot windows.",
    )

    args = parser.parse_args()

    input_path = resolve_input_path(args.input)
    output_dir = Path(args.out)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = parse_three_line_velocity_file(input_path)
    plot_df = downsample_by_time(df, args.bin)

    df.to_csv(output_dir / "global_vision_velocity_parsed.csv", index=False)

    print_basic_info(df, input_path)

    interactive = not args.no_show

    plot_vx_vy(plot_df, output_dir, interactive)
    plot_speed_magnitude(plot_df, output_dir, interactive)
    plot_xy_plane(plot_df, output_dir, interactive)

    print("\nGenerated files:")
    print(output_dir / "global_vision_velocity_vx_vy.png")
    print(output_dir / "global_vision_velocity_magnitude.png")
    print(output_dir / "global_vision_velocity_xy_plane.png")
    print(output_dir / "global_vision_velocity_parsed.csv")

    if interactive:
        print("\nInteractive mode: click legend items to show/hide each data series.")
        plt.show()


if __name__ == "__main__":
    main()
