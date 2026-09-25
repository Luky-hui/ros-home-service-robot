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
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button
from PIL import Image


DEFAULT_MAP_YAML = "/home/robot/bobac3_ws/src/bobac3_navigation/maps/national_2026.yaml"
DEFAULT_MISSION_CONFIG = (
    "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"
)
SCORE_BOX_SIZE_METERS = 0.50
DEFAULT_SIM_MAP_IMAGE = "/home/robot/.gazebo/models/rei_2026raicom/service_group_map/meshes/map.png"
# service_floor.dae mesh bounds in Gazebo/map coordinates.
DEFAULT_SIM_MAP_EXTENT = [-0.5974152, 2.602585, -2.859911, 0.3400891]



def segment_length(start, end):
    return math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))


def segment_midpoint(start, end):
    return ((float(start[0]) + float(end[0])) / 2.0, (float(start[1]) + float(end[1])) / 2.0)


def add_length_label(ax, start, end, color, artists):
    length = segment_length(start, end)
    mid_x, mid_y = segment_midpoint(start, end)
    label = ax.text(
        mid_x,
        mid_y,
        "%.2fm" % length,
        color=color,
        fontsize=8,
        ha="center",
        va="center",
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none", "pad": 1.5},
        zorder=8,
    )
    artists.append(label)
    return label


def load_map(map_yaml):
    with open(map_yaml, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    image_path = data["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)
    image = np.array(Image.open(image_path).convert("L"))
    resolution = float(data["resolution"])
    origin = data["origin"]
    return data, image, image_path, resolution, float(origin[0]), float(origin[1])


def load_json(path):
    if not os.path.exists(path):
        return {"vws": []}
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def backup_file(path):
    if os.path.exists(path):
        backup = "%s.bak_%s" % (path, time.strftime("%Y%m%d_%H%M%S"))
        shutil.copy2(path, backup)
        return backup
    return ""


def polygon_to_walls(points, zone_id):
    walls = []
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        walls.append(
            {
                "id": "%s_edge_%d" % (zone_id, index + 1),
                "points": [
                    {"x": float(start[0]), "y": float(start[1]), "z": 0.0},
                    {"x": float(end[0]), "y": float(end[1]), "z": 0.0},
                ],
            }
        )
    return walls


def save_virtual_walls(json_path, points, zone_id):
    data = load_json(json_path)
    walls = [wall for wall in data.get("vws", []) if not str(wall.get("id", "")).startswith(zone_id)]
    walls.extend(polygon_to_walls(points, zone_id))
    data["vws"] = walls
    backup = backup_file(json_path)
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return backup


def save_mission_config(config_path, points, zone_id, json_path):
    with open(config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    forbidden = data.setdefault("forbidden_zones", {})
    forbidden["virtual_wall_file"] = json_path
    forbidden["map_topic"] = forbidden.get("map_topic", "/virtual_wall_map")
    forbidden["note"] = "Forbidden zone is enforced by national_2026.json vws line segments."
    zones = [zone for zone in forbidden.get("zones", []) if zone.get("id") != zone_id]
    zones.append(
        {
            "id": zone_id,
            "type": "polygon",
            "frame_id": "map",
            "points": [
                {"x": float(point[0]), "y": float(point[1]), "z": 0.0}
                for point in points
            ],
        }
    )
    forbidden["zones"] = zones
    backup = backup_file(config_path)
    with open(config_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    return backup


def plot_existing_walls(ax, json_path, artists, skip_zone_id=""):
    data = load_json(json_path)
    for wall in data.get("vws", []):
        if skip_zone_id and str(wall.get("id", "")).startswith(skip_zone_id):
            continue
        pts = wall.get("points", [])
        if len(pts) >= 2:
            xs = [float(point["x"]) for point in pts]
            ys = [float(point["y"]) for point in pts]
            line = ax.plot(xs, ys, color="red", linewidth=2, alpha=0.8)[0]
            artists.append(line)
            add_length_label(ax, (xs[0], ys[0]), (xs[1], ys[1]), "red", artists)


def plot_sim_map_background(ax, image_path, extent, alpha, artists):
    if not image_path or not os.path.exists(image_path):
        return None
    image = np.array(Image.open(image_path).convert("RGBA"))
    artist = ax.imshow(
        image,
        origin="lower",
        extent=[float(value) for value in extent],
        alpha=float(alpha),
        zorder=1,
    )
    artists.append(artist)
    return artist


def default_sim_overlay_file(map_yaml):
    base, _ext = os.path.splitext(map_yaml)
    return "%s_sim_overlay.json" % base


def load_sim_overlay(path):
    overlay = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "scale": 1.0}
    if not path or not os.path.exists(path):
        return overlay
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    for key in overlay:
        if key in data:
            overlay[key] = float(data[key])
    return overlay


def save_sim_overlay(path, overlay):
    if not path:
        return ""
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)
    backup = backup_file(path)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(
            {
                "x": float(overlay["x"]),
                "y": float(overlay["y"]),
                "yaw_deg": float(overlay["yaw_deg"]),
                "scale": float(overlay["scale"]),
            },
            stream,
            ensure_ascii=False,
            indent=2,
        )
        stream.write("\n")
    return backup


def load_existing_zone_points(config_path, zone_id):
    if not os.path.exists(config_path):
        return []
    with open(config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    for zone in data.get("forbidden_zones", {}).get("zones", []):
        if zone.get("id") == zone_id:
            return [
                (float(point["x"]), float(point["y"]))
                for point in zone.get("points", [])
                if "x" in point and "y" in point
            ]
    return []


def nearest_point_index(points, point, max_distance):
    if not points:
        return None
    distances = [
        math.hypot(float(existing[0]) - float(point[0]), float(existing[1]) - float(point[1]))
        for existing in points
    ]
    index = int(np.argmin(distances))
    if distances[index] <= max_distance:
        return index
    return None


def plot_waypoints(ax, mission_config, artists):
    if not os.path.exists(mission_config):
        return
    with open(mission_config, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    centers = {}
    for name, point in data.get("waypoints", {}).items():
        x = float(point["x"])
        y = float(point["y"])
        key = (round(x, 4), round(y, 4))
        centers.setdefault(key, []).append(name)
    half = SCORE_BOX_SIZE_METERS / 2.0
    for (x, y), names in centers.items():
        rectangle = Rectangle(
            (x - half, y - half),
            SCORE_BOX_SIZE_METERS,
            SCORE_BOX_SIZE_METERS,
            fill=False,
            edgecolor="#00a000",
            linewidth=1.8,
            linestyle="-",
            alpha=0.95,
        )
        ax.add_patch(rectangle)
        center_marker = ax.plot(
            [x],
            [y],
            marker="o",
            color="#1f77b4",
            markersize=5,
            linestyle="None",
            zorder=5,
        )[0]
        text = ax.text(
            x + 0.03,
            y + 0.03,
            "/".join(names),
            color="#1f77b4",
            fontsize=9,
            zorder=6,
        )
        size_label = ax.text(
            x,
            y + half + 0.04,
            "0.50m x 0.50m",
            color="#00a000",
            fontsize=8,
            ha="center",
            va="bottom",
            bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "none", "pad": 1.2},
            zorder=6,
        )
        artists.extend([rectangle, center_marker, text, size_label])


def interactive_select(args):
    map_data, image, image_path, resolution, origin_x, origin_y = load_map(args.map_yaml)
    height, width = image.shape[:2]
    x_max = origin_x + width * resolution
    y_max = origin_y + height * resolution
    json_path = args.virtual_wall_file or args.map_yaml.replace(".yaml", ".json")
    sim_overlay_file = args.sim_map_overlay_file or default_sim_overlay_file(args.map_yaml)
    sim_overlay = load_sim_overlay(sim_overlay_file)
    center_x = (origin_x + x_max) / 2.0
    center_y = (origin_y + y_max) / 2.0
    sim_extent = [float(value) for value in args.sim_map_extent]
    sim_center_x = (sim_extent[0] + sim_extent[1]) / 2.0
    sim_center_y = (sim_extent[2] + sim_extent[3]) / 2.0

    points = load_existing_zone_points(args.mission_config, args.zone_id)
    display_image = np.flipud(image)
    fig, ax = plt.subplots(figsize=(10, 10))
    plt.subplots_adjust(bottom=0.29)
    map_artists = []
    sim_map_artists = []
    if bool(args.use_sim_map_background):
        plot_sim_map_background(
            ax,
            args.sim_map_image,
            sim_extent,
            args.sim_map_alpha,
            sim_map_artists,
        )
    image_artist = ax.imshow(
        display_image,
        cmap="gray",
        origin="lower",
        extent=[origin_x, x_max, origin_y, y_max],
        alpha=float(args.map_alpha),
        zorder=2,
    )
    map_artists.append(image_artist)
    ax.set_title(
        "Left: add point | Replace Mode: overwrite nearby point | Right/Undo: remove | B: edit color map"
    )
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="yellow", alpha=0.25)
    plot_existing_walls(ax, json_path, map_artists, skip_zone_id=args.zone_id)
    plot_waypoints(ax, args.mission_config, map_artists)
    selected_line, = ax.plot([], [], color="orange", marker="o", linewidth=2)
    map_artists.append(selected_line)
    dynamic_label_artists = []
    full_xlim = (origin_x, x_max)
    full_ylim = (origin_y, y_max)
    view_angle = {"degrees": 0.0}
    pan_state = {
        "active": False,
        "x": 0.0,
        "y": 0.0,
        "xlim": full_xlim,
        "ylim": full_ylim,
    }
    sim_edit_state = {"enabled": False}
    point_replace_state = {"enabled": False}
    sim_drag_state = {
        "active": False,
        "start_map": (0.0, 0.0),
        "start_x": 0.0,
        "start_y": 0.0,
    }
    status = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        color="black",
        fontsize=10,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
    )

    def current_rotation():
        return Affine2D().rotate_deg_around(center_x, center_y, view_angle["degrees"])

    def view_transform():
        return current_rotation() + ax.transData

    def sim_overlay_transform():
        return (
            Affine2D()
            .translate(-sim_center_x, -sim_center_y)
            .scale(float(sim_overlay["scale"]))
            .rotate_deg(float(sim_overlay["yaw_deg"]))
            .translate(
                sim_center_x + float(sim_overlay["x"]),
                sim_center_y + float(sim_overlay["y"]),
            )
        )

    def map_from_display_event(event):
        x, y = view_transform().inverted().transform((event.x, event.y))
        return float(x), float(y)

    def rotated_map_bounds():
        corners = np.array(
            [
                [origin_x, origin_y],
                [origin_x, y_max],
                [x_max, origin_y],
                [x_max, y_max],
            ]
        )
        rotated = current_rotation().transform(corners)
        padding = 0.25
        return (
            (float(rotated[:, 0].min()) - padding, float(rotated[:, 0].max()) + padding),
            (float(rotated[:, 1].min()) - padding, float(rotated[:, 1].max()) + padding),
        )

    def apply_view_transform(redraw=True):
        transform = view_transform()
        sim_transform = sim_overlay_transform() + transform
        for artist in sim_map_artists:
            artist.set_transform(sim_transform)
        for artist in map_artists:
            artist.set_transform(transform)
        if redraw:
            fig.canvas.draw_idle()

    def refresh():
        for artist in list(dynamic_label_artists):
            try:
                artist.remove()
            except ValueError:
                pass
            if artist in map_artists:
                map_artists.remove(artist)
        dynamic_label_artists.clear()
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        if len(points) >= 3:
            xs = xs + [points[0][0]]
            ys = ys + [points[0][1]]
        selected_line.set_data(xs, ys)
        if len(points) >= 2:
            edge_pairs = list(zip(points[:-1], points[1:]))
            if len(points) >= 3:
                edge_pairs.append((points[-1], points[0]))
            for start, end in edge_pairs:
                label = add_length_label(ax, start, end, "darkorange", map_artists)
                dynamic_label_artists.append(label)
                label.set_transform(view_transform())
        mode = "BG-EDIT" if sim_edit_state["enabled"] else "POINT-EDIT"
        replace_text = "ON" if point_replace_state["enabled"] else "OFF"
        status.set_text(
            "mode=%s  replace=%s  points=%d  view=%.1f deg  bg=(x %.3f, y %.3f, yaw %.1f deg)  map=%s\n%s"
            % (
                mode,
                replace_text,
                len(points),
                view_angle["degrees"],
                sim_overlay["x"],
                sim_overlay["y"],
                sim_overlay["yaw_deg"],
                os.path.basename(args.map_yaml),
                format_points(points),
            )
        )
        fig.canvas.draw_idle()

    def undo():
        if points:
            points.pop()
            refresh()

    def clear():
        points.clear()
        refresh()

    def fit_view():
        xlim, ylim = rotated_map_bounds()
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        fig.canvas.draw_idle()

    def rotate_view(delta_degrees):
        view_angle["degrees"] = (view_angle["degrees"] + delta_degrees) % 360.0
        if view_angle["degrees"] > 180.0:
            view_angle["degrees"] -= 360.0
        apply_view_transform(redraw=False)
        fit_view()
        refresh()

    def reset_rotation():
        view_angle["degrees"] = 0.0
        apply_view_transform(redraw=False)
        fit_view()
        refresh()

    def toggle_sim_edit():
        if not sim_map_artists:
            print("SIM_MAP_BACKGROUND_NOT_AVAILABLE")
            return
        sim_edit_state["enabled"] = not sim_edit_state["enabled"]
        refresh()

    def toggle_point_replace():
        point_replace_state["enabled"] = not point_replace_state["enabled"]
        refresh()

    def normalize_sim_yaw():
        sim_overlay["yaw_deg"] = (float(sim_overlay["yaw_deg"]) + 180.0) % 360.0 - 180.0

    def move_sim_map(dx, dy):
        sim_overlay["x"] = float(sim_overlay["x"]) + float(dx)
        sim_overlay["y"] = float(sim_overlay["y"]) + float(dy)
        apply_view_transform(redraw=False)
        refresh()

    def rotate_sim_map(delta_degrees):
        sim_overlay["yaw_deg"] = float(sim_overlay["yaw_deg"]) + float(delta_degrees)
        normalize_sim_yaw()
        apply_view_transform(redraw=False)
        refresh()

    def reset_sim_map():
        sim_overlay["x"] = 0.0
        sim_overlay["y"] = 0.0
        sim_overlay["yaw_deg"] = 0.0
        sim_overlay["scale"] = 1.0
        apply_view_transform(redraw=False)
        refresh()

    def save_sim_map_only():
        backup = save_sim_overlay(sim_overlay_file, sim_overlay)
        print("SIM_MAP_OVERLAY_SAVED")
        print("sim_overlay_file=%s" % sim_overlay_file)
        if backup:
            print("sim_overlay_backup=%s" % backup)

    def zoom_at(x, y, scale):
        if x is None or y is None:
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            x = (xlim[0] + xlim[1]) / 2.0
            y = (ylim[0] + ylim[1]) / 2.0
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        new_width = (xlim[1] - xlim[0]) * scale
        new_height = (ylim[1] - ylim[0]) * scale
        rel_x = (x - xlim[0]) / (xlim[1] - xlim[0])
        rel_y = (y - ylim[0]) / (ylim[1] - ylim[0])
        ax.set_xlim(x - new_width * rel_x, x + new_width * (1.0 - rel_x))
        ax.set_ylim(y - new_height * rel_y, y + new_height * (1.0 - rel_y))
        fig.canvas.draw_idle()

    def pan_by_fraction(dx_fraction, dy_fraction):
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        dx = (xlim[1] - xlim[0]) * dx_fraction
        dy = (ylim[1] - ylim[0]) * dy_fraction
        ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        fig.canvas.draw_idle()

    def save_and_close():
        if len(points) < 3:
            print("Need at least 3 points before saving.")
            return
        wall_backup = save_virtual_walls(json_path, points, args.zone_id)
        config_backup = save_mission_config(
            args.mission_config, points, args.zone_id, json_path
        )
        overlay_backup = save_sim_overlay(sim_overlay_file, sim_overlay)
        print("FORBIDDEN_ZONE_SAVED")
        print("map_yaml=%s" % args.map_yaml)
        print("map_image=%s" % image_path)
        print("virtual_wall_file=%s" % json_path)
        print("mission_config=%s" % args.mission_config)
        print("sim_overlay_file=%s" % sim_overlay_file)
        if wall_backup:
            print("virtual_wall_backup=%s" % wall_backup)
        if config_backup:
            print("mission_config_backup=%s" % config_backup)
        if overlay_backup:
            print("sim_overlay_backup=%s" % overlay_backup)
        print(format_points(points))
        plt.close(fig)

    def on_press(event):
        if event.inaxes != ax:
            return
        if event.button == 1:
            selected_point = map_from_display_event(event)
            if sim_edit_state["enabled"] and sim_map_artists:
                sim_drag_state["active"] = True
                sim_drag_state["start_map"] = selected_point
                sim_drag_state["start_x"] = float(sim_overlay["x"])
                sim_drag_state["start_y"] = float(sim_overlay["y"])
                return
            replace_index = None
            if point_replace_state["enabled"]:
                replace_index = nearest_point_index(
                    points,
                    selected_point,
                    float(args.point_replace_radius),
                )
            if replace_index is None:
                points.append(selected_point)
            else:
                points[replace_index] = selected_point
            refresh()
        elif event.button == 3 and points and not sim_edit_state["enabled"]:
            undo()
        elif event.button == 2:
            if event.xdata is None or event.ydata is None:
                return
            pan_state["active"] = True
            pan_state["x"] = float(event.xdata)
            pan_state["y"] = float(event.ydata)
            pan_state["xlim"] = ax.get_xlim()
            pan_state["ylim"] = ax.get_ylim()

    def on_release(event):
        if event.button == 1:
            sim_drag_state["active"] = False
        if event.button == 2:
            pan_state["active"] = False

    def on_motion(event):
        if event.inaxes != ax:
            return
        if sim_drag_state["active"]:
            current_point = map_from_display_event(event)
            start_point = sim_drag_state["start_map"]
            sim_overlay["x"] = sim_drag_state["start_x"] + current_point[0] - start_point[0]
            sim_overlay["y"] = sim_drag_state["start_y"] + current_point[1] - start_point[1]
            apply_view_transform(redraw=True)
            return
        if not pan_state["active"]:
            return
        if event.xdata is None or event.ydata is None:
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
        if event.button == "up":
            zoom_at(event.xdata, event.ydata, 1.0 / 1.25)
        elif event.button == "down":
            zoom_at(event.xdata, event.ydata, 1.25)

    def on_key(event):
        if event.key == "b":
            toggle_sim_edit()
        elif event.key == "p":
            toggle_point_replace()
        elif event.key in ("enter", "return"):
            save_and_close()
        elif event.key in ("backspace", "delete", "u", "ctrl+z"):
            if not sim_edit_state["enabled"]:
                undo()
        elif event.key == "c":
            if not sim_edit_state["enabled"]:
                clear()
        elif event.key == "escape":
            print("FORBIDDEN_ZONE_CANCELLED")
            plt.close(fig)
        elif sim_edit_state["enabled"] and event.key == "s":
            save_sim_map_only()
        elif event.key in ("+", "="):
            zoom_at(None, None, 1.0 / 1.25)
        elif event.key in ("-", "_"):
            zoom_at(None, None, 1.25)
        elif event.key == "home":
            fit_view()
        elif event.key == "[":
            if sim_edit_state["enabled"]:
                rotate_sim_map(-float(args.sim_map_rotation_step))
            else:
                rotate_view(-5.0)
        elif event.key == "]":
            if sim_edit_state["enabled"]:
                rotate_sim_map(float(args.sim_map_rotation_step))
            else:
                rotate_view(5.0)
        elif event.key == "r":
            if sim_edit_state["enabled"]:
                reset_sim_map()
            else:
                reset_rotation()
        elif event.key == "left":
            if sim_edit_state["enabled"]:
                move_sim_map(-float(args.sim_map_move_step), 0.0)
            else:
                pan_by_fraction(-0.12, 0.0)
        elif event.key == "right":
            if sim_edit_state["enabled"]:
                move_sim_map(float(args.sim_map_move_step), 0.0)
            else:
                pan_by_fraction(0.12, 0.0)
        elif event.key == "up":
            if sim_edit_state["enabled"]:
                move_sim_map(0.0, float(args.sim_map_move_step))
            else:
                pan_by_fraction(0.0, 0.12)
        elif event.key == "down":
            if sim_edit_state["enabled"]:
                move_sim_map(0.0, -float(args.sim_map_move_step))
            else:
                pan_by_fraction(0.0, -0.12)

    button_specs = [
        ("Undo", 0.04, 0.04, undo),
        ("Clear", 0.16, 0.04, clear),
        ("Save", 0.28, 0.04, save_and_close),
        ("Fit", 0.40, 0.04, fit_view),
        ("Rot -5", 0.52, 0.04, lambda: rotate_view(-5.0)),
        ("Rot +5", 0.64, 0.04, lambda: rotate_view(5.0)),
        ("Reset", 0.76, 0.04, reset_rotation),
        ("BG Mode", 0.04, 0.115, toggle_sim_edit),
        ("BG Save", 0.16, 0.115, save_sim_map_only),
        ("BG -1", 0.28, 0.115, lambda: rotate_sim_map(-float(args.sim_map_rotation_step))),
        ("BG +1", 0.40, 0.115, lambda: rotate_sim_map(float(args.sim_map_rotation_step))),
        ("BG Reset", 0.52, 0.115, reset_sim_map),
        ("Replace", 0.64, 0.115, toggle_point_replace),
    ]
    buttons = []
    for label, left, bottom, callback in button_specs:
        button_ax = fig.add_axes([left, bottom, 0.1, 0.055])
        button = Button(button_ax, label)
        button.on_clicked(lambda _event, func=callback: func())
        buttons.append(button)

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig._forbidden_zone_buttons = buttons
    apply_view_transform(redraw=False)
    refresh()
    plt.show()


def format_points(points):
    if not points:
        return "[]"
    return json.dumps(
        [{"x": round(point[0], 4), "y": round(point[1], 4)} for point in points],
        ensure_ascii=False,
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw the national forbidden zone on the saved map."
    )
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    parser.add_argument("--mission-config", default=DEFAULT_MISSION_CONFIG)
    parser.add_argument("--virtual-wall-file", default="")
    parser.add_argument("--zone-id", default="corridor_forbidden_zone")
    parser.add_argument("--map-alpha", type=float, default=0.35)
    parser.add_argument("--sim-map-alpha", type=float, default=0.72)
    parser.add_argument("--sim-map-image", default=DEFAULT_SIM_MAP_IMAGE)
    parser.add_argument("--sim-map-overlay-file", default="")
    parser.add_argument("--sim-map-move-step", type=float, default=0.02)
    parser.add_argument("--sim-map-rotation-step", type=float, default=1.0)
    parser.add_argument(
        "--sim-map-extent",
        type=float,
        nargs=4,
        default=DEFAULT_SIM_MAP_EXTENT,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
    )
    sim_map_group = parser.add_mutually_exclusive_group()
    sim_map_group.add_argument(
        "--use-sim-map-background",
        dest="use_sim_map_background",
        action="store_true",
        default=True,
    )
    sim_map_group.add_argument(
        "--no-use-sim-map-background",
        dest="use_sim_map_background",
        action="store_false",
    )
    parser.add_argument("--point-replace-radius", type=float, default=0.08)
    return parser.parse_args()


if __name__ == "__main__":
    interactive_select(parse_args())
