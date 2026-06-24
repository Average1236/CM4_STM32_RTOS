#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize STM32 velocity log CSV. Plot optflow_body instead of odom.

Default input:
    stm32_velocity_log.csv

Generated outputs:
    stm32_velocity_vx_comparison.png
    stm32_velocity_vy_comparison.png
    stm32_velocity_planned_fused_acceleration.png
    stm32_velocity_fused_tracking_error.png
    stm32_velocity_xy_plane.png
    stm32_velocity_vision_source.png, when vision_source exists
    stm32_velocity_yaw_control.png, when chassis_yaw_rad exists
    stm32_velocity_controller_force.png, when controller_f_task_x exists
    stm32_velocity_acceleration_feedforward.png, when acc_ff_x/y exist
    stm32_velocity_xy_acc_dec_limits.png, when XY acceleration-limit columns exist
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
from matplotlib.axes import Axes
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


def add_optional_derived_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    # Raspberry Pi logs raw_vision_vel_x/y in mm/s; all other velocity columns
    # in this script are m/s.
    if {"raw_vision_vel_x", "raw_vision_vel_y"}.issubset(df.columns):
        df["raw_vision_vx"] = df["raw_vision_vel_x"] * 0.001
        df["raw_vision_vy"] = df["raw_vision_vel_y"] * 0.001

    dt_s = (df["time_ms"].diff() * 0.001).where(lambda values: values > 0)
    acceleration_sources = {
        "planned": ("planned_vx", "planned_vy"),
        "fused_chassis": ("fused_chassis_vx", "fused_chassis_vy"),
    }
    for source_name, (vx_column, vy_column) in acceleration_sources.items():
        if {vx_column, vy_column}.issubset(df.columns):
            df[f"{source_name}_acc_x"] = (
                df[vx_column].diff().div(dt_s).replace([np.inf, -np.inf], np.nan)
            )
            df[f"{source_name}_acc_y"] = (
                df[vy_column].diff().div(dt_s).replace([np.inf, -np.inf], np.nan)
            )

    return df


def optional_columns(df: pd.DataFrame, columns: list[str]) -> list[str]:
    return [col for col in columns if col in df.columns]


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
    df = add_optional_derived_columns(df)

    df["t_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
    df["t_bin"] = (df["t_s"] / bin_s).round() * bin_s
    plot_df = df.groupby("t_bin", as_index=False).mean(numeric_only=True)

    return df, plot_df


def add_interactive_legend(ax: Axes, artists: list[Any]) -> None:
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
        columns=optional_columns(
            plot_df,
            [
            "target_vx",
            "planned_vx",
            "optflow_kf_vx",
            "wheel_chassis_vx",
            "fused_chassis_vx",
            "optflow_body_vx",
            "raw_vision_vx",
            ],
        ),
        title="X-axis velocity comparison",
        ylabel="vx",
        output_path=output_dir / "stm32_velocity_vx_comparison.png",
        interactive=interactive,
    )


def plot_vy_comparison(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> None:
    save_line_chart(
        plot_df=plot_df,
        columns=optional_columns(
            plot_df,
            [
            "target_vy",
            "planned_vy",
            "optflow_kf_vy",
            "wheel_chassis_vy",
            "fused_chassis_vy",
            "optflow_body_vy",
            "raw_vision_vy",
            ],
        ),
        title="Y-axis velocity comparison",
        ylabel="vy",
        output_path=output_dir / "stm32_velocity_vy_comparison.png",
        interactive=interactive,
    )


def plot_planned_fused_acceleration(
    plot_df: pd.DataFrame,
    output_dir: Path,
    interactive: bool,
) -> bool:
    required_cols = {
        "planned_acc_x",
        "planned_acc_y",
        "fused_chassis_acc_x",
        "fused_chassis_acc_y",
    }
    if not required_cols.issubset(plot_df.columns):
        return False

    fig, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for ax, axis_name in ((ax_x, "x"), (ax_y, "y")):
        artists = []
        for source_name, color in (
            ("planned", "tab:red"),
            ("fused_chassis", "tab:blue"),
        ):
            column = f"{source_name}_acc_{axis_name}"
            (line,) = ax.plot(
                plot_df["t_bin"],
                plot_df[column],
                label=column,
                color=color,
            )
            artists.append(line)

        ax.axhline(0, linestyle="--", linewidth=1, color="gray")
        ax.set_title(f"{axis_name.upper()}-axis planned and fused acceleration")
        ax.set_ylabel("Acceleration (m/s^2)")
        ax.grid(True, alpha=0.3)
        add_interactive_legend(ax, artists)

    ax_y.set_xlabel("Elapsed time (s)")
    fig.tight_layout()
    fig.savefig(
        output_dir / "stm32_velocity_planned_fused_acceleration.png",
        dpi=180,
    )
    if not interactive:
        plt.close(fig)

    return True

def plot_fused_tracking_error(
    df: pd.DataFrame,
    output_dir: Path,
    interactive: bool,
) -> None:
    df = df.copy()

    df["fused_error_vx"] = df["fused_chassis_vx"] - df["target_vx"]
    df["fused_error_vy"] = df["fused_chassis_vy"] - df["target_vy"]
    if {"planned_vx", "planned_vy"}.issubset(df.columns):
        df["planned_error_vx"] = df["planned_vx"] - df["target_vx"]
        df["planned_error_vy"] = df["planned_vy"] - df["target_vy"]
    if {"raw_vision_vx", "raw_vision_vy"}.issubset(df.columns):
        df["raw_vision_error_vx"] = df["raw_vision_vx"] - df["target_vx"]
        df["raw_vision_error_vy"] = df["raw_vision_vy"] - df["target_vy"]

    err_df = df.groupby("t_bin", as_index=False).mean(numeric_only=True)

    fig, ax = plt.subplots(figsize=(12, 5))
    artists = []
    (line_vx,) = ax.plot(
        err_df["t_bin"],
        err_df["fused_error_vx"],
        label="fused_chassis_vx - target_vx",
    )
    artists.append(line_vx)
    (line_vy,) = ax.plot(
        err_df["t_bin"],
        err_df["fused_error_vy"],
        label="fused_chassis_vy - target_vy",
    )
    artists.append(line_vy)
    if {"planned_error_vx", "planned_error_vy"}.issubset(err_df.columns):
        (line_planned_vx,) = ax.plot(
            err_df["t_bin"],
            err_df["planned_error_vx"],
            label="planned_vx - target_vx",
            alpha=0.75,
        )
        artists.append(line_planned_vx)
        (line_planned_vy,) = ax.plot(
            err_df["t_bin"],
            err_df["planned_error_vy"],
            label="planned_vy - target_vy",
            alpha=0.75,
        )
        artists.append(line_planned_vy)
    if {"raw_vision_error_vx", "raw_vision_error_vy"}.issubset(err_df.columns):
        (line_raw_vx,) = ax.plot(
            err_df["t_bin"],
            err_df["raw_vision_error_vx"],
            label="raw_vision_vx - target_vx",
            alpha=0.75,
        )
        artists.append(line_raw_vx)
        (line_raw_vy,) = ax.plot(
            err_df["t_bin"],
            err_df["raw_vision_error_vy"],
            label="raw_vision_vy - target_vy",
            alpha=0.75,
        )
        artists.append(line_raw_vy)

    ax.axhline(0, linestyle="--", linewidth=1)
    ax.set_title("Velocity tracking error")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Error")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, artists)

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
    artists = [target_scatter, fused_scatter]
    if {"planned_vx", "planned_vy"}.issubset(sample.columns):
        planned_scatter = ax.scatter(
            sample["planned_vx"],
            sample["planned_vy"],
            label="planned",
            s=18,
            alpha=0.65,
        )
        artists.append(planned_scatter)
    if {"raw_vision_vx", "raw_vision_vy"}.issubset(sample.columns):
        raw_vision_scatter = ax.scatter(
            sample["raw_vision_vx"],
            sample["raw_vision_vy"],
            label="raw_vision",
            s=18,
            alpha=0.55,
        )
        artists.append(raw_vision_scatter)

    ax.set_title("Velocity command and estimates in vx-vy plane")
    ax.set_xlabel("vx")
    ax.set_ylabel("vy")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, artists)
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
            f"planned_{axis}",
            f"optflow_kf_{axis}",
            f"wheel_chassis_{axis}",
            f"fused_chassis_{axis}",
            f"optflow_body_{axis}",
            f"raw_vision_{axis}",
        ]:
            if col not in df.columns:
                continue
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


def plot_vision_source(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> bool:
    if "vision_source" not in plot_df.columns:
        return False

    fig, ax = plt.subplots(figsize=(12, 3.5))
    (line,) = ax.step(
        plot_df["t_bin"],
        plot_df["vision_source"],
        where="post",
        label="vision_source",
    )
    ax.set_title("Vision source selector")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("source")
    ax.set_yticks(sorted(plot_df["vision_source"].dropna().unique()))
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, [line])

    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_vision_source.png", dpi=180)
    if not interactive:
        plt.close(fig)

    return True


def _add_twinx_interactive_legend(
    ax1: Axes,
    ax2: Axes,
    artists_ax1: list[Any],
    artists_ax2: list[Any],
) -> None:
    """Combine artists from two twin axes into one interactive legend.

    Passes the original line artists directly as legend handles so that
    pick-event → artist mapping stays 1:1 and toggle works reliably.
    """
    all_artists = artists_ax1 + artists_ax2
    if not all_artists:
        return

    labels = [a.get_label() for a in all_artists]
    # ax2 is the top-most overlapping axes created by twinx(). Matplotlib
    # dispatches pick events only to artists in that top-most axes.
    legend = ax2.legend(all_artists, labels, loc="best")
    legend_map: dict[Any, Any] = {}
    legend_items: dict[Any, list[Any]] = {}

    # The legend stores the handles we just passed; use them for the
    # picker / mapping.  On some mpl versions these are the exact same
    # objects; on others they are proxies — either way the Legend was
    # built from them so the order matches `all_artists`.
    legend_handles = getattr(legend, "legend_handles", None)
    if legend_handles is None:
        legend_handles = legend.legendHandles

    for lh, artist in zip(legend_handles, all_artists):
        lh.set_picker(True)
        lh.set_pickradius(8)
        lh.set_alpha(1.0 if artist.get_visible() else 0.25)
        legend_map[lh] = artist
        legend_items.setdefault(artist, []).append(lh)

    for lt, artist in zip(legend.get_texts(), all_artists):
        lt.set_picker(True)
        lt.set_alpha(1.0 if artist.get_visible() else 0.25)
        legend_map[lt] = artist
        legend_items.setdefault(artist, []).append(lt)

    def on_pick(event: Any) -> None:
        artist = legend_map.get(event.artist)
        if artist is None:
            return
        visible = not artist.get_visible()
        artist.set_visible(visible)
        for legend_item in legend_items[artist]:
            legend_item.set_alpha(1.0 if visible else 0.25)
        ax1.figure.canvas.draw_idle()

    ax1.figure.canvas.mpl_connect("pick_event", on_pick)
    ax2.text(
        0.99, 0.01,
        "Click legend items to show/hide",
        transform=ax2.transAxes,
        ha="right", va="bottom", fontsize=9, alpha=0.65,
    )


def plot_yaw_control(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> bool:
    yaw_cols = optional_columns(plot_df, [
        "chassis_yaw_rad",
        "chassis_omega_z",
        "target_vr",
        "controller_omega_ref",
    ])
    if not yaw_cols:
        return False

    fig, ax1 = plt.subplots(figsize=(12, 5))

    artists_ax1: list[Any] = []
    if "chassis_yaw_rad" in yaw_cols:
        (line,) = ax1.plot(
            plot_df["t_bin"], plot_df["chassis_yaw_rad"],
            label="chassis_yaw_rad", color="tab:blue",
        )
        artists_ax1.append(line)
    ax1.set_ylabel("Yaw (rad)")
    ax1.tick_params(axis="y")

    # Omega_z / omega_ref on right axis (rad/s)
    ax2 = ax1.twinx()
    artists_ax2: list[Any] = []
    if "chassis_omega_z" in yaw_cols:
        (line,) = ax2.plot(
            plot_df["t_bin"], plot_df["chassis_omega_z"],
            label="chassis_omega_z", color="tab:orange",
        )
        artists_ax2.append(line)
    if "target_vr" in yaw_cols:
        (line,) = ax2.plot(
            plot_df["t_bin"], plot_df["target_vr"],
            label="target_vr", color="tab:red", linestyle=":",
        )
        artists_ax2.append(line)
    if "controller_omega_ref" in yaw_cols:
        (line,) = ax2.plot(
            plot_df["t_bin"], plot_df["controller_omega_ref"],
            label="controller_omega_ref", color="tab:green", linestyle="--",
        )
        artists_ax2.append(line)
    ax2.set_ylabel("ωz (rad/s)")
    ax2.tick_params(axis="y")

    ax1.set_title("Yaw control: angle, angular velocity, and rate reference")
    ax1.set_xlabel("Elapsed time (s)")
    ax1.grid(True, alpha=0.3)
    _add_twinx_interactive_legend(ax1, ax2, artists_ax1, artists_ax2)

    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_yaw_control.png", dpi=180)
    if not interactive:
        plt.close(fig)

    return True


def plot_controller_force(plot_df: pd.DataFrame, output_dir: Path, interactive: bool) -> bool:
    force_cols = optional_columns(plot_df, [
        "controller_f_task_x",
        "controller_f_task_y",
        "controller_f_task_psi",
    ])
    if not force_cols:
        return False

    fig, (ax_xy, ax_psi) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    artists_xy = []
    if "controller_f_task_x" in force_cols:
        (line,) = ax_xy.plot(
            plot_df["t_bin"], plot_df["controller_f_task_x"],
            label="F_task_x", color="tab:red",
        )
        artists_xy.append(line)
    if "controller_f_task_y" in force_cols:
        (line,) = ax_xy.plot(
            plot_df["t_bin"], plot_df["controller_f_task_y"],
            label="F_task_y", color="tab:green",
        )
        artists_xy.append(line)
    ax_xy.set_ylabel("Force (N)")
    ax_xy.set_title("ChassisController force output: F_task_x / F_task_y")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axhline(0, linestyle="--", linewidth=1, color="gray")
    add_interactive_legend(ax_xy, artists_xy)

    artists_psi = []
    if "controller_f_task_psi" in force_cols:
        (line,) = ax_psi.plot(
            plot_df["t_bin"], plot_df["controller_f_task_psi"],
            label="F_task_ψ", color="tab:purple",
        )
        artists_psi.append(line)
    ax_psi.set_ylabel("Yaw moment (N·m)")
    ax_psi.set_title("ChassisController force output: F_task_ψ")
    ax_psi.set_xlabel("Elapsed time (s)")
    ax_psi.grid(True, alpha=0.3)
    ax_psi.axhline(0, linestyle="--", linewidth=1, color="gray")
    add_interactive_legend(ax_psi, artists_psi)

    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_controller_force.png", dpi=180)
    if not interactive:
        plt.close(fig)

    return True


def plot_acceleration_feedforward(
    plot_df: pd.DataFrame,
    output_dir: Path,
    interactive: bool,
) -> bool:
    acceleration_cols = optional_columns(plot_df, ["acc_ff_x", "acc_ff_y"])
    if not acceleration_cols:
        return False

    fig, ax = plt.subplots(figsize=(12, 5))
    artists = []
    colors = {"acc_ff_x": "tab:red", "acc_ff_y": "tab:green"}
    for column in acceleration_cols:
        (line,) = ax.plot(
            plot_df["t_bin"],
            plot_df[column],
            label=column,
            color=colors[column],
        )
        artists.append(line)

    ax.axhline(0, linestyle="--", linewidth=1, color="gray")
    ax.set_title("Host acceleration feedforward")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.grid(True, alpha=0.3)
    add_interactive_legend(ax, artists)

    fig.tight_layout()
    fig.savefig(
        output_dir / "stm32_velocity_acceleration_feedforward.png",
        dpi=180,
    )
    if not interactive:
        plt.close(fig)

    return True

def plot_xy_acc_dec_limits(
    plot_df: pd.DataFrame,
    output_dir: Path,
    interactive: bool,
) -> bool:
    required_cols = {
        "xy_max_acc_x",
        "xy_max_acc_y",
        "xy_max_dec_x",
        "xy_max_dec_y",
    }
    if not required_cols.issubset(plot_df.columns):
        return False

    fig, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    for ax, axis_name in ((ax_x, "x"), (ax_y, "y")):
        artists = []
        for limit_name, linestyle in (("acc", "-"), ("dec", "--")):
            column = f"xy_max_{limit_name}_{axis_name}"
            (line,) = ax.plot(
                plot_df["t_bin"],
                plot_df[column],
                label=column,
                linestyle=linestyle,
            )
            artists.append(line)

        feedforward_column = f"acc_ff_{axis_name}"
        if feedforward_column in plot_df.columns:
            (line,) = ax.plot(
                plot_df["t_bin"],
                plot_df[feedforward_column],
                label=feedforward_column,
                color="purple",
                linewidth=1.4,
            )
            artists.append(line)

        ax.set_title(f"{axis_name.upper()}-axis acceleration feedforward and limits")
        ax.set_ylabel("Acceleration (m/s^2)")
        ax.grid(True, alpha=0.3)
        add_interactive_legend(ax, artists)

    ax_y.set_xlabel("Elapsed time (s)")
    fig.tight_layout()
    fig.savefig(output_dir / "stm32_velocity_xy_acc_dec_limits.png", dpi=180)
    if not interactive:
        plt.close(fig)

    return True

def print_basic_info(df: pd.DataFrame, csv_path: Path) -> None:
    duration_s = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    median_dt_ms = df["time_ms"].diff().median()

    print(f"Input CSV: {csv_path}")
    print(f"Rows: {len(df)}")
    print(f"Columns: {len(df.columns)}")
    print(f"Duration: {duration_s:.3f} s")
    print(f"Median sample interval: {median_dt_ms:.3f} ms")
    if "vision_source" in df.columns:
        counts = df["vision_source"].value_counts(dropna=False).sort_index()
        print("vision_source counts:")
        for source, count in counts.items():
            print(f"  {source}: {count}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv",
        type=str,
        default="stm32_velocity_log_filt.csv",
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
    has_planned_fused_acceleration_plot = plot_planned_fused_acceleration(
        plot_df, output_dir, interactive
    )
    plot_fused_tracking_error(df, output_dir, interactive)
    plot_xy_plane(plot_df, output_dir, interactive)
    has_vision_source_plot = plot_vision_source(plot_df, output_dir, interactive)
    has_yaw_control_plot = plot_yaw_control(plot_df, output_dir, interactive)
    has_controller_force_plot = plot_controller_force(plot_df, output_dir, interactive)
    has_acceleration_feedforward_plot = plot_acceleration_feedforward(
        plot_df, output_dir, interactive
    )
    has_xy_acc_dec_limits_plot = plot_xy_acc_dec_limits(plot_df, output_dir, interactive)
    summary = export_error_summary(df, output_dir)

    print("\nError summary:")
    print(summary.to_string(index=False))

    print("\nGenerated files:")
    print(output_dir / "stm32_velocity_vx_comparison.png")
    print(output_dir / "stm32_velocity_vy_comparison.png")
    if has_planned_fused_acceleration_plot:
        print(output_dir / "stm32_velocity_planned_fused_acceleration.png")
    print(output_dir / "stm32_velocity_fused_tracking_error.png")
    print(output_dir / "stm32_velocity_xy_plane.png")
    if has_vision_source_plot:
        print(output_dir / "stm32_velocity_vision_source.png")
    if has_yaw_control_plot:
        print(output_dir / "stm32_velocity_yaw_control.png")
    if has_controller_force_plot:
        print(output_dir / "stm32_velocity_controller_force.png")
    if has_acceleration_feedforward_plot:
        print(output_dir / "stm32_velocity_acceleration_feedforward.png")
    if has_xy_acc_dec_limits_plot:
        print(output_dir / "stm32_velocity_xy_acc_dec_limits.png")
    print(output_dir / "stm32_velocity_error_summary.csv")

    if interactive:
        print("\nInteractive mode: click legend items to show/hide each data series.")
        plt.show()


if __name__ == "__main__":
    main()
