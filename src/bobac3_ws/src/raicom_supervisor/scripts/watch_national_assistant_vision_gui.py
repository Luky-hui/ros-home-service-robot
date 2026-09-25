#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import ctypes
import json
import os
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
from std_msgs.msg import String

from raicom_vision.srv import DetectObjects, DetectObjectsRequest


DEFAULT_CONFIG = "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"
DEFAULT_DETECTION_SERVICE = "/raicom_vision/detect"
DEFAULT_ANNOTATED_TOPIC = "/raicom_vision/annotated"
DEFAULT_STATUS_TOPIC = "/home_service_mission/status"
WINDOW_NAME = "RAICOM Task2 Vision Monitor"


def u(*codes):
    return "".join(chr(code) for code in codes)


TEXT_TITLE = u(0x4efb, 0x52a1, 0x4e8c, 0x5b9e, 0x65f6, 0x8bc6, 0x522b)
TEXT_TITLE_FIND = u(0x4efb, 0x52a1, 0x4e09, 0x5b9e, 0x65f6, 0x8bc6, 0x522b)
TEXT_DETECTED_PREFIX = u(0x8bc6, 0x522b, 0x5230, 0xff1a)
TEXT_DETECTED_NONE = u(0x8bc6, 0x522b, 0x5230, 0xff1a, 0x6682, 0x65e0)
TEXT_WAITING = u(0x7b49, 0x5f85, 0x5230, 0x8fbe, 0x53a8, 0x623f, 0x540e, 0x5f00, 0x59cb, 0x8bc6, 0x522b)
TEXT_DELAY = u(0x5230, 0x8fbe, 0x53a8, 0x623f, 0xff0c, 0x7b49, 0x5f85, 0x5f00, 0x59cb, 0x8bc6, 0x522b)
TEXT_RUNNING = u(0x8bc6, 0x522b, 0x4e2d)
TEXT_STOPPED = u(0x8bc6, 0x522b, 0x5df2, 0x505c, 0x6b62)


def minimize_x11_window(window_name):
    try:
        x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        display = x11.XOpenDisplay(None)
        if not display:
            return False
        root = x11.XDefaultRootWindow(ctypes.c_void_p(display))
        screen = x11.XDefaultScreen(ctypes.c_void_p(display))

        def fetch_name(window):
            name_ptr = ctypes.c_char_p()
            ok = x11.XFetchName(ctypes.c_void_p(display), ctypes.c_ulong(window), ctypes.byref(name_ptr))
            if ok == 0 or not name_ptr.value:
                return ""
            return name_ptr.value.decode("utf-8", "replace")

        def children(window):
            root_return = ctypes.c_ulong()
            parent_return = ctypes.c_ulong()
            children_return = ctypes.POINTER(ctypes.c_ulong)()
            nchildren = ctypes.c_uint()
            ok = x11.XQueryTree(
                ctypes.c_void_p(display),
                ctypes.c_ulong(window),
                ctypes.byref(root_return),
                ctypes.byref(parent_return),
                ctypes.byref(children_return),
                ctypes.byref(nchildren),
            )
            if ok == 0:
                return []
            result = [children_return[index] for index in range(nchildren.value)]
            if children_return:
                x11.XFree(children_return)
            return result

        stack = [root]
        target = 0
        while stack:
            window = stack.pop()
            if fetch_name(window) == window_name:
                target = window
                break
            stack.extend(children(window))
        if not target:
            x11.XCloseDisplay(ctypes.c_void_p(display))
            return False
        x11.XIconifyWindow(ctypes.c_void_p(display), ctypes.c_ulong(target), ctypes.c_int(screen))
        x11.XFlush(ctypes.c_void_p(display))
        x11.XCloseDisplay(ctypes.c_void_p(display))
        return True
    except Exception:
        return False


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


def parse_labels(value):
    return [label.strip() for label in value.split(",") if label.strip()]


def load_monitor_config(path, mode, topic_override, confidence_override, labels_override):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if labels_override:
        labels = parse_labels(labels_override)
    elif mode == "find":
        find_object = data["find_object"]
        labels = list(find_object.get("labels") or find_object.get("trigger_phrases", {}).keys())
        if not labels:
            labels = ["cell phone", "backpack"]
    else:
        labels = list(data["assistant"]["food_labels"])
    section = data["find_object"] if mode == "find" else data["assistant"]
    topic = topic_override or section["image_topic"]
    confidence = (
        float(confidence_override)
        if confidence_override is not None
        else float(section["confidence"])
    )
    return labels, topic, confidence


def normalize_image_file_path(path):
    value = path or ""
    if value.startswith("file://"):
        value = value[len("file://"):]
    return value if value and os.path.isfile(value) else ""


class AssistantVisionMonitor:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_image = None
        self.last_raw = None
        self.last_result = []
        self.cumulative_result = {}
        self.last_message = TEXT_WAITING
        self.last_seconds = 0.0
        self.detect_enabled = bool(args.always_on)
        self.last_find_trigger = ""
        self.stop_requested = False
        self.font_title = load_font(22)
        self.font_body = load_font(18)
        self.title = TEXT_TITLE_FIND if args.mode == "find" else TEXT_TITLE
        self.labels, self.image_topic, self.confidence = load_monitor_config(
            args.config, args.mode, args.topic or "", args.confidence, args.labels
        )
        self.image_path = normalize_image_file_path(args.image_path) or normalize_image_file_path(self.image_topic)
        if self.image_path:
            image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
            if image is not None:
                self.last_raw = image
            else:
                self.last_message = "image file unreadable: %s" % self.image_path
        self.status_sub = rospy.Subscriber(
            args.status_topic, String, self.status_callback, queue_size=10
        )
        self.annotated_sub = rospy.Subscriber(
            args.annotated_topic, Image, self.annotated_callback, queue_size=1
        )
        self.raw_sub = None
        if not self.image_path:
            self.raw_sub = rospy.Subscriber(
                self.image_topic, Image, self.raw_callback, queue_size=1
            )
        self.client = None
        self.worker = threading.Thread(target=self.detect_loop)
        self.worker.daemon = True
        self.worker.start()

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        state = data.get("state", "")
        mission = data.get("mission", "")
        if self.args.always_on:
            with self.lock:
                self.detect_enabled = True
                if self.last_message in (TEXT_WAITING, TEXT_STOPPED):
                    self.last_message = TEXT_RUNNING
            return
        if self.args.mode == "find":
            with self.lock:
                waypoint = data.get("waypoint", "")
                trigger_key = "%s:%s" % (state, waypoint)
                if (
                    mission == "find_object"
                    and state in ("yaw_aligned", "arrived")
                    and trigger_key != self.last_find_trigger
                ):
                    self.last_find_trigger = trigger_key
                    self.detect_enabled = True
                    self.last_message = TEXT_RUNNING
                    self.last_result = []
                    self.cumulative_result = {}
                    self.last_seconds = 0.0
                elif mission == "find_object":
                    self.detect_enabled = False
                    if state in ("target_found", "target_not_found", "completed"):
                        self.last_message = TEXT_STOPPED
            return
        if state == "assistant_vision_started" or (
            mission == "assistant" and state == "assistant_vision_started"
        ):
            with self.lock:
                self.detect_enabled = True
                self.last_message = TEXT_RUNNING
                self.last_result = []
                self.cumulative_result = {}
                self.last_seconds = 0.0
        elif state == "assistant_vision_delay":
            with self.lock:
                self.detect_enabled = False
                self.last_result = []
                self.cumulative_result = {}
                self.last_seconds = 0.0
                self.last_message = TEXT_DELAY
        elif state == "assistant_vision_stopped" or state in ("vision_result", "vision_no_food"):
            with self.lock:
                self.detect_enabled = False
                self.last_message = TEXT_STOPPED

    def annotated_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Annotated image conversion failed: %s", exc)
            return
        with self.lock:
            self.last_image = image

    def raw_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.lock:
            self.last_raw = image

    def detect_loop(self):
        try:
            rospy.wait_for_service(self.args.service, timeout=60.0)
            self.client = rospy.ServiceProxy(self.args.service, DetectObjects)
        except Exception as exc:
            with self.lock:
                self.last_message = "service wait failed: %s" % exc
            return
        delay = 1.0 / max(float(self.args.rate), 0.1)
        while not rospy.is_shutdown() and not self.stop_requested:
            with self.lock:
                enabled = self.detect_enabled
            if not enabled:
                rospy.sleep(0.1)
                continue
            request = DetectObjectsRequest()
            request.image_topic = self.image_path or self.image_topic
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
                result = sorted(merged.items(), key=lambda item: item[1], reverse=True)
                message = response.message
            except Exception as exc:
                result = []
                message = "detect failed: %s" % exc
            with self.lock:
                if self.detect_enabled:
                    for label, score in result:
                        if (
                            label not in self.cumulative_result
                            or score > self.cumulative_result[label]
                        ):
                            self.cumulative_result[label] = score
                    self.last_result = sorted(
                        self.cumulative_result.items(),
                        key=lambda item: item[1],
                        reverse=True,
                    )
                    self.last_message = message
                    self.last_seconds = time.time() - started
                    if self.args.single_shot:
                        self.detect_enabled = False
            rospy.sleep(delay)

    def make_placeholder(self):
        canvas = np.zeros((420, 640, 3), dtype=np.uint8)
        canvas[:] = (30, 30, 30)
        return canvas

    def draw_text_panel(self, frame):
        _height, width = frame.shape[:2]
        panel_height = 180
        panel = np.zeros((panel_height, width, 3), dtype=np.uint8)
        panel[:] = (245, 245, 245)
        with self.lock:
            result = list(self.last_result)
            message = self.last_message
            seconds = self.last_seconds
            enabled = self.detect_enabled
        rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
        image = PilImage.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        title = self.title + "  " + (TEXT_RUNNING if enabled else TEXT_STOPPED)
        draw.text((14, 10), title, font=self.font_title, fill=(20, 20, 20))
        source = self.image_path or self.image_topic
        status = "source=%s  conf=%.2f  %.2fs" % (source, self.confidence, seconds)
        draw.text((14, 42), status, font=self.font_body, fill=(70, 70, 70))
        if result:
            items = ["%s %.2f" % (label, score) for label, score in result[:14]]
            top_text = TEXT_DETECTED_PREFIX + "  ".join(items[:7])
            second_text = "  ".join(items[7:])
        else:
            top_text = TEXT_DETECTED_NONE
            second_text = ""
        draw.text((14, 74), top_text, font=self.font_body, fill=(0, 85, 170))
        draw.text((14, 104), second_text, font=self.font_body, fill=(0, 85, 170))
        draw.text((14, 138), message[:90], font=self.font_body, fill=(90, 90, 90))
        panel = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        return np.vstack([frame, panel])

    def run(self):
        cv2.namedWindow(self.args.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.args.window_name, int(self.args.width), int(self.args.height))
        minimized_once = False
        while not rospy.is_shutdown():
            with self.lock:
                frame = None
                if self.last_image is not None:
                    frame = self.last_image.copy()
                elif self.last_raw is not None:
                    frame = self.last_raw.copy()
            if frame is None:
                frame = self.make_placeholder()
            frame = self.draw_text_panel(frame)
            cv2.imshow(self.args.window_name, frame)
            key = cv2.waitKey(30) & 0xFF
            if self.args.start_minimized and not minimized_once:
                minimize_x11_window(self.args.window_name)
                minimized_once = True
            if key in (27, ord("q")):
                self.stop_requested = True
                break
        cv2.destroyWindow(self.args.window_name)


def parse_args():
    parser = argparse.ArgumentParser(description="RAICOM live vision GUI monitor.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=("assistant", "find"), default="assistant")
    parser.add_argument("--topic", default="")
    parser.add_argument("--image-path", default="")
    parser.add_argument("--labels", default="")
    parser.add_argument("--confidence", type=float, default=None)
    parser.add_argument("--service", default=DEFAULT_DETECTION_SERVICE)
    parser.add_argument("--annotated-topic", default=DEFAULT_ANNOTATED_TOPIC)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--window-name", default=WINDOW_NAME)
    parser.add_argument("--node-name", default="national_assistant_vision_monitor")
    parser.add_argument("--always-on", action="store_true", default=False)
    parser.add_argument("--single-shot", action="store_true", default=False)
    parser.add_argument("--start-minimized", dest="start_minimized", action="store_true", default=True)
    parser.add_argument("--show-on-start", dest="start_minimized", action="store_false")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node(args.node_name)
    monitor = AssistantVisionMonitor(args)
    monitor.run()


if __name__ == "__main__":
    main()
