#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import os
import shutil
import time

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Polygon, Rectangle
from matplotlib.widgets import Button
from PIL import Image


DEFAULT_MAP_YAML = "/home/robot/bobac3_ws/src/bobac3_navigation/maps/national_2026.yaml"
DEFAULT_MISSION_CONFIG = (
    "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"
)
SCORE_BOX_SIZE_METERS = 0.50
DEFAULT_SPEED_VALUES = {
    "max_vel_x": 0.10,
    "max_vel_y": 0.08,
    "max_trans_vel": 0.10,
    "max_vel_theta": 0.60,
}


def backup_file(path):
    if not os.path.exists(path):
        return ""
    backup = "%s.bak_%s" % (path, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(path, backup)
    return backup


def load_map(map_yaml):
    with open(map_yaml, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    image_path = data["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)
    image = np.array(Image.open(image_path).convert("L"))
    resolution = float(data["resolution"])
    origin = data["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    height, width = image.shape[:2]
    return image, [origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution]


def load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError("Invalid mission config: %s" % config_path)
    return data


def save_config(config_path, data):
    backup = backup_file(config_path)
    with open(config_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    return backup


def speed_values_from_args(args):
    return {
        "max_vel_x": float(args.max_vel_x),
        "max_vel_y": float(args.max_vel_y),
        "max_trans_vel": float(args.max_trans_vel),
        "max_vel_theta": float(args.max_vel_theta),
    }


def polygon_area(points):
    area = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        area += float(point[0]) * float(next_point[1]) - float(next_point[0]) * float(point[1])
    return abs(area) / 2.0


def validate_points(points):
    if len(points) != 4:
        raise RuntimeError("Slow area needs exactly 4 vertices.")
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    area = polygon_area(points)
    if width < 0.10 or height < 0.10 or area < 0.02:
        raise RuntimeError(
            "Slow area is too small/thin: width=%.3fm height=%.3fm area=%.3fm^2. "
            "Mark a real four-vertex rectangle." % (width, height, area)
        )


def format_points(points):
    return json.dumps(
        [{"x": round(point[0], 4), "y": round(point[1], 4)} for point in points],
        ensure_ascii=False,
    )


def load_existing_area(config_path, zone_id):
    if not os.path.exists(config_path):
        return []
    data = load_config(config_path)
    for zone in data.get("slow_zones", {}).get("zones", []):
        if zone.get("id") == zone_id:
            return [(float(point["x"]), float(point["y"])) for point in zone.get("points", [])]
    return []


def upsert_slow_area(config_path, zone_id, points, speed_values, replace_all=True):
    validate_points(points)
    data = load_config(config_path)
    slow = data.setdefault("slow_zones", {})
    slow["enabled"] = True
    slow["default_multiplier"] = float(slow.get("default_multiplier", 0.45))
    slow["speed_params"] = slow.get(
        "speed_params",
        ["max_vel_x", "max_vel_y", "max_trans_vel", "max_vel_theta"],
    )
    slow["speed_values"] = dict(speed_values)
    slow["note"] = (
        "Slow area limits DWA speed to fixed emergency-stop-friendly values "
        "while the robot is inside the polygon."
    )
    zones = [] if replace_all else [zone for zone in slow.get("zones", []) if zone.get("id") != zone_id]
    zones.append(
        {
            "id": zone_id,
            "type": "polygon",
            "frame_id": "map",
            "enabled": True,
            "speed_values": dict(speed_values),
            "points": [
                {"x": float(point[0]), "y": float(point[1]), "z": 0.0}
                for point in points
            ],
        }
    )
    slow["zones"] = zones
    backup = save_config(config_path, data)
    return backup


def delete_slow_areas(config_path):
    data = load_config(config_path)
    slow = data.setdefault("slow_zones", {})
    removed = len(slow.get("zones", []))
    slow["zones"] = []
    backup = save_config(config_path, data)
    return backup, removed


def plot_waypoints(ax, mission_config):
    if not os.path.exists(mission_config):
        return
    data = load_config(mission_config)
    half = SCORE_BOX_SIZE_METERS / 2.0
    for name, point in data.get("waypoints", {}).items():
        x = float(point["x"])
        y = float(point["y"])
        ax.add_patch(
            Rectangle(
                (x - half, y - half),
                SCORE_BOX_SIZE_METERS,
                SCORE_BOX_SIZE_METERS,
                fill=False,
                edgecolor="#00a000",
                linewidth=1.3,
                alpha=0.8,
                zorder=5,
            )
        )
        ax.plot([x], [y], marker="o", color="#1f77b4", markersize=4, zorder=6)
        ax.text(x + 0.03, y + 0.03, name, color="#1f77b4", fontsize=8, zorder=6)


def draw_area(ax, points, label, color, alpha, zorder):
    if not points:
        return []
    artists = []
    if len(points) >= 3:
        patch = Polygon(points, closed=True, facecolor=color, edgecolor=color, alpha=alpha, linewidth=2.2, zorder=zorder)
        ax.add_patch(patch)
        artists.append(patch)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    line = ax.plot(xs, ys, marker="o", color=color, linewidth=1.5, zorder=zorder + 1)[0]
    artists.append(line)
    if len(points) >= 1:
        text = ax.text(
            xs[0],
            ys[0] + 0.03,
            label,
            color=color,
            fontsize=9,
            zorder=zorder + 2,
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none", "pad": 1.0},
        )
        artists.append(text)
    return artists


def plot_existing_slow_areas(ax, mission_config, current_zone_id):
    artists = []
    if not os.path.exists(mission_config):
        return artists
    data = load_config(mission_config)
    for zone in data.get("slow_zones", {}).get("zones", []):
        if zone.get("id") == current_zone_id:
            continue
        points = [(float(point["x"]), float(point["y"])) for point in zone.get("points", [])]
        artists.extend(draw_area(ax, points, zone.get("id", "slow_area"), "#0066ff", 0.18, 7))
    return artists


def interactive_select(args):
    image, extent = load_map(args.map_yaml)
    points = load_existing_area(args.mission_config, args.zone_id)
    speed_values = speed_values_from_args(args)

    fig, ax = plt.subplots(figsize=(10, 9))
    plt.subplots_adjust(bottom=0.2)
    ax.imshow(np.flipud(image), cmap="gray", origin="lower", extent=extent, alpha=args.map_alpha)
    ax.set_title("Slow area: click 4 vertices. Wheel/+/− zoom, middle-drag/arrows pan.")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="yellow", alpha=0.25)
    plot_waypoints(ax, args.mission_config)

    dynamic_artists = []
    pan_state = {"active": False, "x": 0.0, "y": 0.0, "xlim": (extent[0], extent[1]), "ylim": (extent[2], extent[3])}
    status = ax.text(0.01, 0.01, "", transform=ax.transAxes, color="black", fontsize=10, bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"}, zorder=20)

    def clear_dynamic_artists():
        for artist in dynamic_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        dynamic_artists[:] = []

    def refresh():
        clear_dynamic_artists()
        dynamic_artists.extend(plot_existing_slow_areas(ax, args.mission_config, args.zone_id))
        dynamic_artists.extend(draw_area(ax, points, args.zone_id, "#ff8c00", 0.30, 13))
        status.set_text("zone=%s vertices=%d speed=%s\n%s" % (args.zone_id, len(points), speed_values, points))
        fig.canvas.draw_idle()

    def undo():
        if points:
            points.pop()
            refresh()

    def clear():
        points[:] = []
        refresh()

    def fit_view():
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        fig.canvas.draw_idle()

    def zoom_at(x, y, scale):
        if x is None or y is None:
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            x = (xlim[0] + xlim[1]) / 2.0
            y = (ylim[0] + ylim[1]) / 2.0
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        new_width = (xlim[1] - xlim[0]) * float(scale)
        new_height = (ylim[1] - ylim[0]) * float(scale)
        rel_x = (float(x) - xlim[0]) / (xlim[1] - xlim[0])
        rel_y = (float(y) - ylim[0]) / (ylim[1] - ylim[0])
        ax.set_xlim(float(x) - new_width * rel_x, float(x) + new_width * (1.0 - rel_x))
        ax.set_ylim(float(y) - new_height * rel_y, float(y) + new_height * (1.0 - rel_y))
        fig.canvas.draw_idle()

    def pan_by_fraction(dx_fraction, dy_fraction):
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        dx = (xlim[1] - xlim[0]) * float(dx_fraction)
        dy = (ylim[1] - ylim[0]) * float(dy_fraction)
        ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        fig.canvas.draw_idle()

    def save_and_close():
        try:
            backup = upsert_slow_area(args.mission_config, args.zone_id, points, speed_values, replace_all=not args.keep_existing)
        except Exception as exc:
            print("SLOW_AREA_SAVE_FAILED %s" % exc)
            return
        print("SLOW_AREA_SAVED")
        print("mission_config=%s" % args.mission_config)
        print("zone_id=%s" % args.zone_id)
        print("speed_values=%s" % speed_values)
        if backup:
            print("mission_config_backup=%s" % backup)
        print(format_points(points))
        plt.close(fig)

    def on_press(event):
        if event.inaxes != ax:
            return
        if event.button == 1:
            if len(points) >= 4:
                points[:] = []
            points.append((float(event.xdata), float(event.ydata)))
            refresh()
        elif event.button == 3:
            undo()
        elif event.button == 2:
            pan_state["active"] = True
            pan_state["x"] = float(event.xdata)
            pan_state["y"] = float(event.ydata)
            pan_state["xlim"] = ax.get_xlim()
            pan_state["ylim"] = ax.get_ylim()

    def on_release(event):
        if event.button == 2:
            pan_state["active"] = False

    def on_motion(event):
        if not pan_state["active"] or event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        dx = pan_state["x"] - float(event.xdata)
        dy = pan_state["y"] - float(event.ydata)
        xlim = pan_state["xlim"]
        ylim = pan_state["ylim"]
        ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        fig.canvas.draw_idle()

    def on_scroll(event):
        if event.inaxes != ax:
            return
        zoom_at(event.xdata, event.ydata, 1.0 / 1.25 if event.button == "up" else 1.25)

    def on_key(event):
        if event.key in ("enter", "return"):
            save_and_close()
        elif event.key in ("backspace", "delete", "u", "ctrl+z"):
            undo()
        elif event.key == "c":
            clear()
        elif event.key == "home":
            fit_view()
        elif event.key in ("+", "="):
            zoom_at(None, None, 1.0 / 1.25)
        elif event.key in ("-", "_"):
            zoom_at(None, None, 1.25)
        elif event.key == "left":
            pan_by_fraction(-0.15, 0.0)
        elif event.key == "right":
            pan_by_fraction(0.15, 0.0)
        elif event.key == "up":
            pan_by_fraction(0.0, 0.15)
        elif event.key == "down":
            pan_by_fraction(0.0, -0.15)
        elif event.key == "escape":
            print("SLOW_AREA_CANCELLED")
            plt.close(fig)

    buttons = []
    for label, left, callback in [
        ("Undo", 0.05, undo),
        ("Clear", 0.17, clear),
        ("Save", 0.29, save_and_close),
        ("Fit", 0.41, fit_view),
        ("Zoom+", 0.53, lambda: zoom_at(None, None, 1.0 / 1.25)),
        ("Zoom-", 0.65, lambda: zoom_at(None, None, 1.25)),
    ]:
        button_ax = fig.add_axes([left, 0.055, 0.095, 0.06])
        button = Button(button_ax, label)
        button.on_clicked(lambda _event, func=callback: func())
        buttons.append(button)
    fig._slow_area_buttons = buttons
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fit_view()
    refresh()
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(description="Mark one four-vertex slow area in national_home_waypoints.yaml.")
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    parser.add_argument("--mission-config", default=DEFAULT_MISSION_CONFIG)
    parser.add_argument("--zone-id", default="slow_area")
    parser.add_argument("--map-alpha", type=float, default=0.45)
    parser.add_argument("--max-vel-x", type=float, default=DEFAULT_SPEED_VALUES["max_vel_x"])
    parser.add_argument("--max-vel-y", type=float, default=DEFAULT_SPEED_VALUES["max_vel_y"])
    parser.add_argument("--max-trans-vel", type=float, default=DEFAULT_SPEED_VALUES["max_trans_vel"])
    parser.add_argument("--max-vel-theta", type=float, default=DEFAULT_SPEED_VALUES["max_vel_theta"])
    parser.add_argument("--keep-existing", action="store_true", help="Keep other slow zones instead of replacing all of them.")
    parser.add_argument("--points", type=float, nargs=8, metavar=("X1", "Y1", "X2", "Y2", "X3", "Y3", "X4", "Y4"))
    parser.add_argument("--delete", action="store_true", help="Delete all slow areas from the mission config.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.delete:
        backup, removed = delete_slow_areas(args.mission_config)
        print("SLOW_AREAS_DELETED removed=%d" % removed)
        if backup:
            print("mission_config_backup=%s" % backup)
        return
    speed_values = speed_values_from_args(args)
    if args.points:
        points = [(args.points[i], args.points[i + 1]) for i in range(0, 8, 2)]
        backup = upsert_slow_area(args.mission_config, args.zone_id, points, speed_values, replace_all=not args.keep_existing)
        print("SLOW_AREA_SAVED")
        print("mission_config=%s" % args.mission_config)
        print("zone_id=%s" % args.zone_id)
        print("speed_values=%s" % speed_values)
        if backup:
            print("mission_config_backup=%s" % backup)
        print(format_points(points))
        return
    interactive_select(args)


if __name__ == "__main__":
    main()
