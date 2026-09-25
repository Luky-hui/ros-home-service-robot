#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import shutil

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="/home/robot/ros_workspace/datasets/raicom_service_yolo/data.yaml")
    parser.add_argument("--weights", default="/home/robot/yolov8n.pt")
    parser.add_argument("--project", default="/home/robot/ros_workspace/runs/raicom_service_yolo")
    parser.add_argument("--name", default="train")
    parser.add_argument("--epochs", type=int, default=45)
    parser.add_argument("--imgsz", type=int, default=480)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--out", default="/home/robot/ros_workspace/models/raicom_service_yolo.pt")
    args = parser.parse_args()

    model = YOLO(args.weights)
    result = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device="cpu",
        workers=0,
        project=args.project,
        name=args.name,
        exist_ok=True,
        patience=10,
        verbose=True,
    )
    best_path = os.path.join(args.project, args.name, "weights", "best.pt")
    if not os.path.isfile(best_path):
        raise RuntimeError("best.pt not found: %s" % best_path)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    shutil.copy2(best_path, args.out)
    print("MODEL", args.out)
    print("RESULT", result)


if __name__ == "__main__":
    main()
