#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import atexit
import os
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from PIL import Image as PilImage
from PIL import ImageDraw, ImageFont
from sensor_msgs.msg import Image

from raicom_vision.srv import DetectObjects, DetectObjectsRequest


DEFAULT_HOME_CONFIG = "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"
DEFAULT_VISION_CONFIG = "/home/robot/bobac3_ws/src/raicom_vision/config/vision_raicom16.yaml"
DEFAULT_SERVICE = "/raicom_vision/detect"
DEFAULT_ANNOTATED_TOPIC = "/raicom_vision/annotated"
WINDOW_NAME = "RAICOM Task2 Real Camera Vision Test"


def u(*codes):
    return "".join(chr(code) for code in codes)


TEXT_TITLE = u(0x4efb, 0x52a1, 0x4e8c, 0x771f, 0x5b9e, 0x6444, 0x50cf, 0x5934, 0x8bc6, 0x522b, 0x6d4b, 0x8bd5)
TEXT_CURRENT = u(0x5f53, 0x524d, 0x5e27)
TEXT_BEST = u(0x7d2f, 0x8ba1, 0x6700, 0x9ad8)
TEXT_NONE = u(0x6682, 0x65e0)
TEXT_WAIT_IMAGE = u(0x7b49, 0x5f85, 0x6444, 0x50cf, 0x5934, 0x56fe, 0x50cf)
TEXT_WAIT_SERVICE = u(0x7b49, 0x5f85, 0x89c6, 0x89c9, 0x670d, 0x52a1)
TEXT_RUNNING = u(0x8bc6, 0x522b, 0x4e2d)
TEXT_EXIT = u(0x6309, 0x20, 0x71, 0x20, 0x6216, 0x20, 0x45, 0x73, 0x63, 0x20, 0x9000, 0x51fa)


def load_font(size):
    paths = [
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in paths:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def run_quiet(args, timeout=3.0):
    try:
        return subprocess.check_output(
            args,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            universal_newlines=True,
        )
    except Exception:
        return ""


def topic_exists(topic):
    topics = run_quiet(["rostopic", "list"], timeout=3.0).splitlines()
    return topic in topics


def service_exists(service):
    services = run_quiet(["rosservice", "list"], timeout=3.0).splitlines()
    return service in services


class ProcessManager:
    def __init__(self):
        self.processes = []
        atexit.register(self.cleanup)

    def start(self, name, args, log_path):
        log_file = open(log_path, "a")
        log_file.write("\n--- start %s: %s ---\n" % (name, " ".join(args)))
        log_file.flush()
        process = subprocess.Popen(
            args,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        self.processes.append((name, process, log_file))
        return process

    def cleanup(self):
        for name, process, log_file in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                except Exception:
                    pass
        deadline = time.time() + 3.0
        for name, process, log_file in reversed(self.processes):
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
            try:
                log_file.close()
            except Exception:
                pass


class RealtimeVisionTester:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.raw_image = None
        self.annotated_image = None
        self.current_result = []
        self.best_result = {}
        self.message = TEXT_WAIT_SERVICE
        self.inference_seconds = 0.0
        self.stop_requested = False
        self.font_title = load_font(22)
        self.font_body = load_font(18)
        self.font_small = load_font(16)
        self.labels, self.image_topic, self.confidence = self.load_config()
        self.client = None
        self.raw_sub = rospy.Subscriber(self.image_topic, Image, self.raw_callback, queue_size=1)
        self.annotated_sub = rospy.Subscriber(
            args.annotated_topic,
            Image,
            self.annotated_callback,
            queue_size=1,
        )
        self.worker = threading.Thread(target=self.detect_loop)
        self.worker.daemon = True
        self.worker.start()

    def load_config(self):
        with open(self.args.config, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        assistant = data["assistant"]
        labels = list(assistant["food_labels"])
        topic = self.args.topic or assistant["image_topic"]
        confidence = self.args.confidence
        if confidence is None:
            confidence = float(assistant["confidence"])
        return labels, topic, float(confidence)

    def raw_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Raw image conversion failed: %s", exc)
            return
        with self.lock:
            self.raw_image = image

    def annotated_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Annotated image conversion failed: %s", exc)
            return
        with self.lock:
            self.annotated_image = image

    def detect_loop(self):
        try:
            rospy.wait_for_service(self.args.service, timeout=float(self.args.service_wait))
            self.client = rospy.ServiceProxy(self.args.service, DetectObjects)
        except Exception as exc:
            with self.lock:
                self.message = "service unavailable: %s" % exc
            return
        interval = 1.0 / max(float(self.args.rate), 0.1)
        while not rospy.is_shutdown() and not self.stop_requested:
            request = DetectObjectsRequest()
            request.image_topic = self.image_topic
            request.labels = self.labels
            request.confidence = float(self.confidence)
            started = time.time()
            try:
                response = self.client(request)
                merged = {}
                for label, score in zip(response.labels, response.confidences):
                    score = float(score)
                    if label not in merged or score > merged[label]:
                        merged[label] = score
                current = sorted(merged.items(), key=lambda item: item[1], reverse=True)
                message = response.message
            except Exception as exc:
                current = []
                message = "detect failed: %s" % exc
            seconds = time.time() - started
            with self.lock:
                self.current_result = current
                for label, score in current:
                    if label not in self.best_result or score > self.best_result[label]:
                        self.best_result[label] = score
                self.message = message
                self.inference_seconds = seconds
            rospy.sleep(interval)

    def make_placeholder(self):
        canvas = np.zeros((420, 640, 3), dtype=np.uint8)
        canvas[:] = (30, 30, 30)
        return canvas

    def draw_text_panel(self, frame):
        _height, width = frame.shape[:2]
        panel_height = 220
        panel = np.zeros((panel_height, width, 3), dtype=np.uint8)
        panel[:] = (246, 246, 246)
        with self.lock:
            current = list(self.current_result)
            best = sorted(self.best_result.items(), key=lambda item: item[1], reverse=True)
            message = self.message
            seconds = self.inference_seconds
        rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
        image = PilImage.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        draw.text((14, 8), TEXT_TITLE + "  " + TEXT_RUNNING, font=self.font_title, fill=(20, 20, 20))
        draw.text(
            (14, 40),
            "topic=%s  conf=%.2f  rate=%.2fHz  %.2fs" % (
                self.image_topic,
                self.confidence,
                float(self.args.rate),
                seconds,
            ),
            font=self.font_small,
            fill=(70, 70, 70),
        )
        draw.text((14, 66), TEXT_CURRENT + "：" + self.format_items(current), font=self.font_body, fill=(0, 80, 160))
        draw.text((14, 100), TEXT_BEST + "：" + self.format_items(best), font=self.font_body, fill=(0, 120, 70))
        draw.text((14, 136), message[:100], font=self.font_small, fill=(90, 90, 90))
        draw.text((14, 168), TEXT_EXIT, font=self.font_small, fill=(90, 90, 90))
        panel = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        return np.vstack([frame, panel])

    @staticmethod
    def format_items(items):
        if not items:
            return TEXT_NONE
        return "  ".join("%s %.2f" % (label, score) for label, score in items[:14])

    def run(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, int(self.args.width), int(self.args.height))
        while not rospy.is_shutdown():
            with self.lock:
                frame = None
                if self.annotated_image is not None:
                    frame = self.annotated_image.copy()
                elif self.raw_image is not None:
                    frame = self.raw_image.copy()
                    self.message = self.message or TEXT_WAIT_IMAGE
            if frame is None:
                frame = self.make_placeholder()
            frame = self.draw_text_panel(frame)
            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                self.stop_requested = True
                break
        cv2.destroyWindow(WINDOW_NAME)


def parse_args():
    parser = argparse.ArgumentParser(description="Open the real head camera and test task2 vegetable recognition live.")
    parser.add_argument("--config", default=DEFAULT_HOME_CONFIG)
    parser.add_argument("--vision-config", default=DEFAULT_VISION_CONFIG)
    parser.add_argument("--topic", default="")
    parser.add_argument("--confidence", type=float, default=None)
    parser.add_argument("--service", default=DEFAULT_SERVICE)
    parser.add_argument("--annotated-topic", default=DEFAULT_ANNOTATED_TOPIC)
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=980)
    parser.add_argument("--height", type=int, default=760)
    parser.add_argument("--service-wait", type=float, default=80.0)
    parser.add_argument("--start-camera", action="store_true", default=True)
    parser.add_argument("--no-start-camera", dest="start_camera", action="store_false")
    parser.add_argument("--restart-vision", action="store_true", default=True)
    parser.add_argument("--no-restart-vision", dest="restart_vision", action="store_false")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    manager = ProcessManager()
    rospy.init_node("national_assistant_vision_camera_test", anonymous=False)
    with open(args.config, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    image_topic = args.topic or data["assistant"]["image_topic"]

    if args.start_camera and not topic_exists(image_topic):
        manager.start(
            "head_camera",
            ["roslaunch", "ar_pose", "usbcam_head.launch"],
            "/tmp/raicom_task2_camera_test_head_camera.log",
        )
        deadline = time.time() + 20.0
        while time.time() < deadline and not topic_exists(image_topic):
            time.sleep(0.5)

    if args.restart_vision:
        subprocess.call(["rosnode", "kill", "/raicom_vision"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.8)
    if args.restart_vision or not service_exists(args.service):
        manager.start(
            "raicom_vision",
            [
                "roslaunch",
                "raicom_vision",
                "vision.launch",
                "config_file:=" + args.vision_config,
            ],
            "/tmp/raicom_task2_camera_test_vision.log",
        )

    tester = RealtimeVisionTester(args)
    tester.run()


if __name__ == "__main__":
    main()
