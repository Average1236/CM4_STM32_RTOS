#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize STM32 velocity log CSV. Plot optflow_body instead of odom.

Default input:
    stm32_velocity_log.csv

Generated outputs:
    stm32_velocity_vx_comparison.png
    stm32_velocity_vy_comparison.png
    stm32_velocity_fused_tracking_error.png
    stm32_velocity_xy_plane.png
    stm32_velocity_error_summary.csv

Usage:
    python visualize_stm32_velocity.py
    python visualize_stm32_velocity.py --csv stm32_velocity_log.csv
    python visualize_stm32_velocity.py --csv stm32_velocity_log.csv --bin 0.2
    python visualize_stm32_velocity.py --no-show
"""

import argparse
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def check_required_columns(df: pd.DataFrame) -> None:
    required_cols = [
        "time_ms",
        "target_vx",
        "target_vy",
        "optflow_kf_vx",
        "optflow_kf_vy",
        "wheel_chassis_vx",
        "wheel_chassis_vy",
        "fused_chassis_vx",
        "fused_chassis_vy",
        "optflow_body_vx",
        "optflow_body_vy",
    ]

    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(
            "CSV is missing required columns: {}\nCurrent columns: {}".format(
                ", ".join(missing),
                ", ".join(df.columns),
            )
        )


def resolve_csv_path(csv_arg: str) -> Path:
    csv_path = Path(csv_arg)
    if csv_path.exists():
        return csv_path

    script_relative_path = Path(__file__).resolve().parent / csv_arg
    if script_relative_path.exists():
        return script_relative_path

    raise FileNotFoundError(
        f"CSV file not found: {csv_arg}\n"
        f"Tried: {csv_path.resolve()} and {script_relative_path}"
    )


def load_and_preprocess(csv_path: Path, bin_s: float) -> tuple[pd.DataFrame, pd.DataFrame]:
    df = pd.read_csv(csv_path)
    check_required_columns(df)

    df["t_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
    df["t_bin"] = (df["t_s"] / bin_s).round() * bin_s
    plot_df = df.groupby("t_bin", as_index=False).mean(numeric_only=True)

    return df, plot_df


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


def save_line_chart(
    plot_df: pd.DataFrame,
    columns: list[str],
    title: str,
    ylabel: str,
    output_path: Path,
    interactive: bool,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 5))

    artists = []
    for col in columns:
        (line,) = ax.plot(plot_df["t_bin"], plot_df[col], label=col)
        artists.append(line)

    ax.set_title(title)
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, artists)

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    if not interactive:
        plt.close(fig)


def plot_vx_comparison(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    save_line_chart(
        plot_df=plot_df,
        columns=[
            "target_vx",
            "optflow_kf_vx",
            "wheel_chassis_vx",
            "fused_chassis_vx",
            "optflow_body_vx",
        ],
        title="X-axis velocity comparison",
        ylabel="vx",
        output_path=output_dir / "stm32_velocity_vx_comparison.png",
        interactive=interactive,
    )


def plot_vy_comparison(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    save_line_chart(
        plot_df=plot_df,
        columns=[
            "target_vy",
            "optflow_kf_vy",
            "wheel_chassis_vy",
            "fused_chassis_vy",
            "optflow_body_vy",
        ],
        title="Y-axis velocity comparison",
        ylabel="vy",
        output_path=output_dir / "stm32_velocity_vy_comparison.png",
        interactive=interactive,
    )


def plot_fused_tracking_error(
    df: pd.DataFrame,
    output_dir: Path,
    interactive: bool,
) -> None:
    df = df.copy()

    df["fused_error_vx"] = df["fused_chassis_vx"] - df["target_vx"]
    df["fused_error_vy"] = df["fused_chassis_vy"] - df["target_vy"]

    err_df = df.groupby("t_bin", as_index=False).mean(numeric_only=True)

    fig, ax = plt.subplots(figsize=(12, 5))
    (line_vx,) = ax.plot(
        err_df["t_bin"],
        err_df["fused_error_vx"],
        label="fused_chassis_vx - target_vx",
    )
    (line_vy,) = ax.plot(
        err_df["t_bin"],
        err_df["fused_error_vy"],
        label="fused_chassis_vy - target_vy",
    )

    ax.axhline(0, linestyle="--", linewidth=1)
    ax.set_title("Fused velocity tracking error")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Error")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [line_vx, line_vy])

    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_fused_tracking_error.png", dpi=180)
    if not interactive:
        plt.close(fig)


def plot_xy_plane(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    sample = plot_df.iloc[::2].copy()

    fig, ax = plt.subplots(figsize=(7, 7))

    target_scatter = ax.scatter(
        sample["target_vx"],
        sample["target_vy"],
        label="target",
        s=18,
        alpha=0.75,
    )
    fused_scatter = ax.scatter(
        sample["fused_chassis_vx"],
        sample["fused_chassis_vy"],
        label="fused",
        s=18,
        alpha=0.75,
    )

    ax.set_title("Velocity command and fused estimate in vx-vy plane")
    ax.set_xlabel("vx")
    ax.set_ylabel("vy")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [target_scatter, fused_scatter])
    ax.axis("equal")

    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_xy_plane.png", dpi=180)
    if not interactive:
        plt.close(fig)


def export_error_summary(df: pd.DataFrame, output_dir: Path) -> pd.DataFrame:
    summary_rows = []

    for axis in ["vx", "vy"]:
        target = df[f"target_{axis}"]

        for col in [
            f"optflow_kf_{axis}",
            f"wheel_chassis_{axis}",
            f"fused_chassis_{axis}",
            f"optflow_body_{axis}",
        ]:
            error = df[col] - target

            summary_rows.append(
                {
                    "axis": axis,
                    "signal": col,
                    "MAE_vs_target": error.abs().mean(),
                    "RMSE_vs_target": np.sqrt((error**2).mean()),
                    "max_abs_error": error.abs().max(),
                }
            )

    summary = pd.DataFrame(summary_rows)
    summary.to_csv(output_dir / "stm32_velocity_error_summary.csv", index=False)
    return summary


def print_basic_info(df: pd.DataFrame, csv_path: Path) -> None:
    duration_s = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    median_dt_ms = df["time_ms"].diff().median()

    print(f"Input CSV: {csv_path}")
    print(f"Rows: {len(df)}")
    print(f"Columns: {len(df.columns)}")
    print(f"Duration: {duration_s:.3f} s")
    print(f"Median sample interval: {median_dt_ms:.3f} ms")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv",
        type=str,
        default="stm32_velocity_log_12.csv",
        help="Path to input CSV file.",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=".",
        help="Output directory for figures and summary CSV.",
    )
    parser.add_argument(
        "--bin",
        type=float,
        default=0.01,
        help="Downsample averaging window in seconds. Default: 0.01",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Only save output files; do not open interactive plot windows.",
    )

    args = parser.parse_args()

    csv_path = resolve_csv_path(args.csv)
    output_dir = Path(args.out)
    output_dir.mkdir(parents=True, exist_ok=True)

    df, plot_df = load_and_preprocess(csv_path, args.bin)

    print_basic_info(df, csv_path)

    interactive = not args.no_show
    plot_vx_comparison(plot_df, output_dir, interactive)
    plot_vy_comparison(plot_df, output_dir, interactive)
    plot_fused_tracking_error(df, output_dir, interactive)
    plot_xy_plane(plot_df, output_dir, interactive)
    summary = export_error_summary(df, output_dir)

    print("\nError summary:")
    print(summary.to_string(index=False))

    print("\nGenerated files:")
    print(output_dir / "stm32_velocity_vx_comparison.png")
    print(output_dir / "stm32_velocity_vy_comparison.png")
    print(output_dir / "stm32_velocity_fused_tracking_error.png")
    print(output_dir / "stm32_velocity_xy_plane.png")
    print(output_dir / "stm32_velocity_error_summary.csv")

    if interactive:
        print("\nInteractive mode: click legend items to show/hide each data series.")
        plt.show()


if __name__ == "__main__":
    main()
