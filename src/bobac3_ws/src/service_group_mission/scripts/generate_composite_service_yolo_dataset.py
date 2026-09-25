#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import random
import shutil

import cv2
import numpy as np


BACKGROUND_DIR = "/tmp/venue_baselines"
FIRE_PNG = "/home/robot/.gazebo/models/rei_2025raicom/fire_hazard/meshes/fire_hazard.png"
EXTINGUISHER_PNG = "/home/robot/.gazebo/models/rei_2025raicom/fire_extinguisher/meshes/fire_extinguisher.png"


def load_rgba(path):
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError("image unreadable: %s" % path)
    if image.shape[2] == 4:
        bgr = image[:, :, :3]
        alpha = image[:, :, 3]
    else:
        bgr = image[:, :, :3]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        alpha = np.where(gray > 18, 255, 0).astype(np.uint8)
    return bgr, alpha


def overlay_object(background, object_bgr, object_alpha, rng, height_range, x_bias):
    image_h, image_w = background.shape[:2]
    raw_h, raw_w = object_bgr.shape[:2]
    target_h = rng.randint(height_range[0], height_range[1])
    target_w = max(8, int(float(raw_w) * float(target_h) / float(raw_h)))
    resized_bgr = cv2.resize(object_bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
    resized_alpha = cv2.resize(object_alpha, (target_w, target_h), interpolation=cv2.INTER_AREA)

    if x_bias == "left":
        x_min = int(image_w * 0.03)
        x_max = int(image_w * 0.38)
    elif x_bias == "right":
        x_min = int(image_w * 0.48)
        x_max = int(image_w * 0.92)
    else:
        x_min = int(image_w * 0.18)
        x_max = int(image_w * 0.78)
    x_max = max(x_min, min(image_w - target_w - 2, x_max))
    x_min = min(x_min, x_max)
    y_min = int(image_h * 0.50)
    y_max = max(y_min, image_h - target_h - 2)
    x_value = rng.randint(x_min, x_max)
    y_value = rng.randint(y_min, y_max)

    gain = rng.uniform(0.75, 1.12)
    resized_bgr = np.clip(resized_bgr.astype(np.float32) * gain, 0, 255).astype(np.uint8)

    roi = background[y_value:y_value + target_h, x_value:x_value + target_w]
    alpha = (resized_alpha.astype(np.float32) / 255.0)[:, :, None]
    mixed = (resized_bgr.astype(np.float32) * alpha + roi.astype(np.float32) * (1.0 - alpha)).astype(np.uint8)
    background[y_value:y_value + target_h, x_value:x_value + target_w] = mixed

    visible = resized_alpha > 20
    ys, xs = np.where(visible)
    if len(xs) == 0 or len(ys) == 0:
        return None
    x1 = x_value + int(xs.min())
    y1 = y_value + int(ys.min())
    x2 = x_value + int(xs.max())
    y2 = y_value + int(ys.max())
    return x1, y1, x2, y2


def yolo_line(class_id, bbox, shape):
    image_h, image_w = shape[:2]
    x1, y1, x2, y2 = bbox
    cx = ((x1 + x2) / 2.0) / float(image_w)
    cy = ((y1 + y2) / 2.0) / float(image_h)
    bw = (x2 - x1) / float(image_w)
    bh = (y2 - y1) / float(image_h)
    return "%d %.6f %.6f %.6f %.6f" % (class_id, cx, cy, bw, bh)


def split_name(index):
    return "train" if index % 10 < 8 else "val"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/home/robot/ros_workspace/datasets/raicom_service_yolo")
    parser.add_argument("--images", type=int, default=600)
    parser.add_argument("--seed", type=int, default=20260702)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()
    rng = random.Random(args.seed)

    backgrounds = []
    for name in sorted(os.listdir(BACKGROUND_DIR)):
        if name.lower().endswith((".jpg", ".png")):
            path = os.path.join(BACKGROUND_DIR, name)
            image = cv2.imread(path, cv2.IMREAD_COLOR)
            if image is not None:
                backgrounds.append((name, image))
    if not backgrounds:
        raise RuntimeError("no backgrounds in %s" % BACKGROUND_DIR)
    fire_bgr, fire_alpha = load_rgba(FIRE_PNG)
    ext_bgr, ext_alpha = load_rgba(EXTINGUISHER_PNG)

    if args.clean and os.path.isdir(args.out):
        shutil.rmtree(args.out)
    for split in ["train", "val"]:
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)

    patterns = ["fire", "extinguisher", "normal", "empty", "fire", "extinguisher"]
    for index in range(args.images):
        bg_name, bg = rng.choice(backgrounds)
        image = bg.copy()
        labels = []
        pattern = patterns[index % len(patterns)]
        bias = rng.choice(["left", "center", "right"])
        if pattern == "fire":
            bbox = overlay_object(image, fire_bgr, fire_alpha, rng, (42, 130), bias)
            if bbox:
                labels.append(yolo_line(0, bbox, image.shape))
        elif pattern in ("extinguisher", "normal"):
            bbox = overlay_object(image, ext_bgr, ext_alpha, rng, (58, 150), bias)
            if bbox:
                labels.append(yolo_line(1, bbox, image.shape))
        elif pattern == "empty":
            pass

        split = split_name(index)
        image_name = "%05d_%s_%s.jpg" % (index, os.path.splitext(bg_name)[0], pattern)
        label_name = image_name.replace(".jpg", ".txt")
        cv2.imwrite(os.path.join(args.out, "images", split, image_name), image)
        with open(os.path.join(args.out, "labels", split, label_name), "w", encoding="utf-8") as stream:
            stream.write("\n".join(labels))
            if labels:
                stream.write("\n")

    with open(os.path.join(args.out, "data.yaml"), "w", encoding="utf-8") as stream:
        stream.write("path: %s\n" % args.out)
        stream.write("train: images/train\n")
        stream.write("val: images/val\n")
        stream.write("names:\n")
        stream.write("  0: fire\n")
        stream.write("  1: fire_extinguisher\n")
    print("DATASET", args.out, "IMAGES", args.images)


if __name__ == "__main__":
    main()
