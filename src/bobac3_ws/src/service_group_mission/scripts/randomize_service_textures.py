#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import random

import yaml
from PIL import Image


MODEL_ROOT = "/home/robot/.gazebo/models/rei_2025raicom"
DEFAULT_ROUTE = ["jilin", "guangzhou", "beijing", "shanghai", "shenzhen"]
DEFAULT_EVENTS = {
    "jilin": "fire",
    "guangzhou": "normal",
    "beijing": "extinguisher_missing",
    "shanghai": "normal",
    "shenzhen": "normal",
}
EVENT_FILE = "/tmp/raicom_service_events.yaml"


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def resolve_events(route, event_mode, random_seed):
    if event_mode == "fixed":
        return {venue: DEFAULT_EVENTS.get(venue, "normal") for venue in route}

    if event_mode == "random":
        rng = random.Random(str(random_seed)) if random_seed != "" else random.Random()
        events = {venue: "normal" for venue in route}
        if not route:
            return events
        fire_venue = rng.choice(route)
        missing_choices = [venue for venue in route if venue != fire_venue] or list(route)
        missing_venue = rng.choice(missing_choices)
        events[fire_venue] = "fire"
        events[missing_venue] = "extinguisher_missing"
        for venue in route:
            if venue in (fire_venue, missing_venue):
                continue
            events[venue] = rng.choice(["normal", "normal", "fire", "extinguisher_missing"])
        return events

    raise ValueError("Unsupported event_mode: %s" % event_mode)


def transparent_black(image):
    image = image.convert("RGBA")
    pixels = image.load()
    width, height = image.size
    for y in range(height):
        for x in range(width):
            r, g, b, a = pixels[x, y]
            if r < 18 and g < 18 and b < 18:
                pixels[x, y] = (r, g, b, 0)
    return image


def load_object_image(model_name):
    path = os.path.join(MODEL_ROOT, model_name, "meshes", "%s.png" % model_name)
    return trim_transparent(transparent_black(Image.open(path)))


def trim_transparent(image):
    bbox = image.getbbox()
    if not bbox:
        return image
    return image.crop(bbox)


def resize_by_width(image, width):
    ratio = float(width) / float(image.size[0])
    height = max(1, int(round(image.size[1] * ratio)))
    return image.resize((int(width), height), Image.LANCZOS)


def paste_object(base, obj, x_ratio, y_ratio, width_ratio):
    width, height = base.size
    obj = resize_by_width(obj, max(1, int(width * width_ratio)))
    x = int(width * x_ratio)
    y = int(height * y_ratio)
    base.alpha_composite(obj, (x, y))


def texture_paths(venue):
    mesh_dir = os.path.join(MODEL_ROOT, venue, "meshes")
    texture_path = os.path.join(mesh_dir, "%s.png" % venue)
    backup_path = texture_path + ".bak_codex_20260622_235943"
    if not os.path.exists(backup_path):
        backup_path = texture_path
    return texture_path, backup_path


def write_textures(route, events):
    fire_image = load_object_image("fire_hazard")
    extinguisher_image = load_object_image("fire_extinguisher")
    for venue in route:
        texture_path, backup_path = texture_paths(venue)
        base = Image.open(backup_path).convert("RGBA")
        event = events.get(venue, "normal")

        if event == "fire":
            paste_object(base, fire_image, 0.18, 0.58, 0.22)
            paste_object(base, extinguisher_image, 0.58, 0.45, 0.18)
        elif event == "normal":
            paste_object(base, extinguisher_image, 0.58, 0.45, 0.18)
        elif event == "extinguisher_missing":
            pass
        else:
            raise ValueError("Unsupported service event: %s" % event)

        base.convert("RGB").save(texture_path)


def save_events(events):
    with open(EVENT_FILE, "w", encoding="utf-8") as stream:
        yaml.safe_dump(events, stream, allow_unicode=True, sort_keys=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-file", default="/home/robot/bobac3_ws/src/service_group_mission/config/service_group_waypoints.yaml")
    parser.add_argument("--event-mode", default="random")
    parser.add_argument("--random-seed", default="")
    args = parser.parse_args()

    config = load_yaml(args.config_file)
    route = list(config["missions"]["patrol"].get("route", DEFAULT_ROUTE))
    events = resolve_events(route, args.event_mode, args.random_seed)
    write_textures(route, events)
    save_events(events)
    print(json.dumps(events, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
