#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import subprocess
import sys

import numpy as np
import yaml


def command_output(arguments):
    result = subprocess.run(
        arguments,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def main():
    report = {"checks": {}, "software_ready": True}

    report["checks"]["camera"] = {
        "ready": os.path.exists("/dev/video0"),
        "path": "/dev/video0",
    }
    report["software_ready"] = report["software_ready"] and report["checks"]["camera"]["ready"]

    face_database = "/home/robot/.ros/face_encodeing.npz"
    face_check = {"ready": os.path.isfile(face_database), "path": face_database}
    if face_check["ready"]:
        data = np.load(face_database, allow_pickle=True)
        face_check["labels"] = sorted(set(data["labels"].tolist()))
        face_check["encoding_count"] = int(data["encoding"].shape[0])
        face_check["operator_label_present"] = "operator" in face_check["labels"]
        face_check["ready"] = face_check["operator_label_present"]
    report["checks"]["face_database"] = face_check
    report["software_ready"] = report["software_ready"] and face_check["ready"]

    model_paths = [
        "/home/robot/ros_workspace/models/raicom_world.pt",
        "/home/robot/ros_workspace/models/sensevoice/model.int8.onnx",
        "/home/robot/ros_workspace/models/melo_tts/model.int8.onnx",
    ]
    model_check = {
        path: {"ready": os.path.isfile(path), "bytes": os.path.getsize(path) if os.path.isfile(path) else 0}
        for path in model_paths
    }
    report["checks"]["models"] = model_check
    report["software_ready"] = report["software_ready"] and all(
        item["ready"] for item in model_check.values()
    )

    package_names = [
        "local_voice_bridge",
        "raicom_vision",
        "service_group_mission",
        "home_service_mission",
        "raicom_supervisor",
    ]
    package_check = {}
    for package_name in package_names:
        code, output, error = command_output(["rospack", "find", package_name])
        package_check[package_name] = {
            "ready": code == 0,
            "path": output,
            "error": error,
        }
    report["checks"]["packages"] = package_check
    report["software_ready"] = report["software_ready"] and all(
        item["ready"] for item in package_check.values()
    )

    home_config_path = (
        "/home/robot/bobac3_ws/src/home_service_mission/config/home_waypoints.yaml"
    )
    with open(home_config_path, "r", encoding="utf-8") as stream:
        home_config = yaml.safe_load(stream)
    missing_points = [
        name
        for name, point in home_config["waypoints"].items()
        if point.get("x") is None or point.get("y") is None or point.get("yaw") is None
    ]
    report["checks"]["home_waypoints"] = {
        "ready": not missing_points,
        "missing_points": missing_points,
        "path": home_config_path,
    }

    audio_code, audio_info, audio_error = command_output(["pactl", "info"])
    audio_values = {}
    for line in audio_info.splitlines():
        if ": " in line:
            key, value = line.split(": ", 1)
            audio_values[key] = value
    default_source = audio_values.get("Default Source", "")
    default_sink = audio_values.get("Default Sink", "")
    expected_source = "alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback"
    expected_sink = "alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo"
    report["checks"]["audio"] = {
        "default_source": default_source,
        "expected_source": expected_source,
        "default_sink": default_sink,
        "expected_sink": expected_sink,
        "error": audio_error,
        "ready": (
            audio_code == 0
            and default_source == expected_source
            and default_sink == expected_sink
        ),
    }
    report["software_ready"] = (
        report["software_ready"] and report["checks"]["audio"]["ready"]
    )

    service_code, services, service_error = command_output(["rosservice", "list"])
    topic_code, topics, topic_error = command_output(["rostopic", "list"])
    service_names = set(services.splitlines())
    topic_names = set(topics.splitlines())
    required_services = ["/voice_tts", "/voice_aiui", "/raicom_vision/detect", "/raicom/emergency_stop/set"]
    required_topics = ["/head_camera/image_raw", "/face_detection", "/local_voice/intent"]
    report["checks"]["live_ros"] = {
        "ready": service_code == 0 and topic_code == 0,
        "services": {name: name in service_names for name in required_services},
        "topics": {name: name in topic_names for name in required_topics},
        "service_error": service_error,
        "topic_error": topic_error,
    }
    report["software_ready"] = (
        report["software_ready"] and report["checks"]["live_ros"]["ready"]
    )
    report["competition_ready"] = (
        report["software_ready"] and report["checks"]["home_waypoints"]["ready"]
    )

    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["software_ready"] else 2


if __name__ == "__main__":
    sys.exit(main())
