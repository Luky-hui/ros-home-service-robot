#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time

import rospy
import yaml

from raicom_vision.srv import DetectObjects, DetectObjectsRequest


DEFAULT_CONFIG = "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"


def load_task(config_file, mode, topic_override, confidence_override):
    with open(config_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if mode == "assistant":
        section = data["assistant"]
        labels = list(section["food_labels"])
    elif mode == "find":
        section = data["find_object"]
        labels = list(section["labels"])
    else:
        raise RuntimeError("Unsupported mode: %s" % mode)
    topic = topic_override or section["image_topic"]
    confidence = float(confidence_override) if confidence_override else float(section["confidence"])
    return labels, topic, confidence


def parse_args():
    parser = argparse.ArgumentParser(description="Live RAICOM national vision probe.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=("assistant", "find"), default="assistant")
    parser.add_argument("--topic", default="")
    parser.add_argument("--confidence", default="")
    parser.add_argument("--rate", type=float, default=1.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("watch_national_vision", anonymous=True)
    labels, topic, confidence = load_task(
        args.config, args.mode, args.topic, args.confidence
    )
    rospy.wait_for_service("/raicom_vision/detect", timeout=20.0)
    client = rospy.ServiceProxy("/raicom_vision/detect", DetectObjects)
    print(
        "WATCH_NATIONAL_VISION mode=%s topic=%s confidence=%.3f labels=%s"
        % (args.mode, topic, confidence, json.dumps(labels, ensure_ascii=False))
    )
    delay = 1.0 / max(args.rate, 0.1)
    while not rospy.is_shutdown():
        request = DetectObjectsRequest()
        request.image_topic = topic
        request.labels = labels
        request.confidence = confidence
        started = time.time()
        try:
            response = client(request)
            message = {
                "success": bool(response.success),
                "message": response.message,
                "labels": list(response.labels),
                "confidences": [round(float(value), 4) for value in response.confidences],
                "topic": topic,
                "seconds": round(time.time() - started, 3),
            }
        except Exception as exc:
            message = {
                "success": False,
                "message": str(exc),
                "labels": [],
                "confidences": [],
                "topic": topic,
                "seconds": round(time.time() - started, 3),
            }
        print(json.dumps(message, ensure_ascii=False, sort_keys=True), flush=True)
        rospy.sleep(delay)


if __name__ == "__main__":
    main()
