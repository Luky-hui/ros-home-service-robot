#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from raicom_vision.srv import DetectObjects, DetectObjectsResponse


class RaicomVisionNode:
    def __init__(self):
        rospy.init_node("raicom_vision")
        config_file = rospy.get_param("~config_file", "")
        if not config_file or not os.path.isfile(config_file):
            raise RuntimeError("Vision config file not found: %s" % config_file)
        with open(config_file, "r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        model_path = self.config["model_path"]
        if not os.path.isfile(model_path):
            raise RuntimeError("Vision model file not found: %s" % model_path)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.model = YOLO(model_path)
        self.extra_models = []
        self.class_names = list(self.config["classes"])
        actual_names = [self.model.names[index] for index in sorted(self.model.names)]
        if actual_names != self.class_names:
            raise RuntimeError(
                "Model classes do not match vision.yaml: model=%s config=%s"
                % (actual_names, self.class_names)
            )
        self.class_ids = {name: index for index, name in enumerate(self.class_names)}
        for extra_model_path in self.config.get("extra_model_paths", []):
            if not os.path.isfile(extra_model_path):
                raise RuntimeError("Extra vision model file not found: %s" % extra_model_path)
            extra_model = YOLO(extra_model_path)
            extra_names = {
                int(index): str(name)
                for index, name in dict(extra_model.names).items()
            }
            self.extra_models.append(
                {
                    "path": extra_model_path,
                    "model": extra_model,
                    "names": extra_names,
                    "class_ids": {name: index for index, name in extra_names.items()},
                }
            )
        self.minimum_area_ratio = float(self.config.get("minimum_area_ratio", 0.0))
        self.label_aliases = dict(self.config.get("label_aliases", {}))
        invalid_aliases = [
            alias
            for alias, model_label in self.label_aliases.items()
            if model_label not in self.class_ids
        ]
        if invalid_aliases:
            raise RuntimeError("Invalid label aliases in vision.yaml: %s" % invalid_aliases)
        self.template_match = self.config.get("template_match", {})
        self.template_labels = []
        self.template_images = {}
        self.template_threshold = float(self.template_match.get("threshold", 0.55))
        self.template_scales = [
            float(value) for value in self.template_match.get("scales", [1.0])
        ]
        if self.template_match.get("enabled", False):
            self.load_templates()
        self.debug_image_directory = self.config.get("debug_image_directory", "")
        if self.debug_image_directory:
            os.makedirs(self.debug_image_directory, exist_ok=True)

        self.annotated_pub = rospy.Publisher(
            self.config["annotated_topic"], Image, queue_size=1
        )
        self.detections_pub = rospy.Publisher(
            self.config["detections_topic"], String, queue_size=10
        )
        self.service = rospy.Service(
            self.config["service_name"], DetectObjects, self.detect
        )
        rospy.loginfo(
            "RAICOM vision ready: model=%s extra_models=%d classes=%d templates=%d service=%s",
            model_path,
            len(self.extra_models),
            len(self.class_names),
            len(self.template_labels),
            self.config["service_name"],
        )

    def load_templates(self):
        directory = self.template_match.get("directory", "")
        if not directory or not os.path.isdir(directory):
            rospy.logwarn("Template directory unavailable: %s", directory)
            return
        for label in self.template_match.get("labels", []):
            path = os.path.join(directory, label + ".png")
            if not os.path.isfile(path):
                rospy.logwarn("Template image missing for %s: %s", label, path)
                continue
            image = cv2.imread(path, cv2.IMREAD_COLOR)
            if image is None:
                rospy.logwarn("Template image unreadable for %s: %s", label, path)
                continue
            image = self.crop_template(image)
            if image.size == 0:
                rospy.logwarn("Template image empty after crop for %s: %s", label, path)
                continue
            self.template_labels.append(label)
            self.template_images[label] = image

    def resolve_yolo_label(self, label):
        return self.label_aliases.get(label, label)

    def extra_model_class_selection(self, model_info, requested_labels):
        class_ids = []
        output_labels = {}
        for label in requested_labels:
            model_label = self.resolve_yolo_label(label)
            candidates = [label, model_label]
            for candidate in candidates:
                if candidate not in model_info["class_ids"]:
                    continue
                if candidate not in output_labels:
                    class_ids.append(model_info["class_ids"][candidate])
                output_labels[candidate] = label
                break
        return class_ids, output_labels

    def load_image_from_source(self, image_source):
        image_path = image_source
        if image_path.startswith("file://"):
            image_path = image_path[len("file://"):]
        if os.path.isfile(image_path):
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("Image file unreadable: %s" % image_path)
            return image, None

        image_timeout = float(self.config["image_timeout"])
        image_message = None
        for _ in range(6):
            image_message = rospy.wait_for_message(
                image_source,
                Image,
                timeout=image_timeout,
            )
            rospy.sleep(0.05)
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        return image, image_message.header

    def detect(self, request):
        response = DetectObjectsResponse()
        requested_labels = list(request.labels) or self.class_names
        yolo_labels = []
        yolo_output_labels = {}
        template_labels = []
        unknown_labels = [
            label
            for label in requested_labels
            if self.resolve_yolo_label(label) not in self.class_ids
            and label not in self.template_images
        ]
        if unknown_labels:
            response.success = False
            response.message = "Unknown labels: " + ", ".join(unknown_labels)
            return response

        for label in requested_labels:
            model_label = self.resolve_yolo_label(label)
            if model_label in self.class_ids:
                if model_label not in yolo_labels:
                    yolo_labels.append(model_label)
                yolo_output_labels.setdefault(model_label, label)
            if label in self.template_images:
                template_labels.append(label)

        image_source = request.image_topic or self.config["default_image_topic"]
        confidence = float(request.confidence)
        if confidence <= 0.0:
            confidence = float(self.config["default_confidence"])

        try:
            image, source_header = self.load_image_from_source(image_source)
            if self.debug_image_directory:
                debug_name = "detect_%d.png" % int(time.time() * 1000)
                cv2.imwrite(os.path.join(self.debug_image_directory, debug_name), image)
        except Exception as exc:
            response.success = False
            response.message = "Image unavailable on %s: %s" % (image_source, exc)
            return response

        started = time.time()
        detections = []
        detected_labels = set()
        annotated = image.copy()

        if yolo_labels:
            class_ids = [self.class_ids[label] for label in yolo_labels]
            try:
                with self.lock:
                    result = self.model.predict(
                        source=image,
                        classes=class_ids,
                        conf=confidence,
                        iou=float(self.config["iou"]),
                        imgsz=int(self.config["image_size"]),
                        device="cpu",
                        verbose=False,
                    )[0]
            except Exception as exc:
                response.success = False
                response.message = "Inference failed: %s" % exc
                return response

            for box in result.boxes:
                class_id = int(box.cls[0].item())
                label = result.names[class_id]
                output_label = yolo_output_labels.get(label, label)
                score = float(box.conf[0].item())
                coordinates = [float(value) for value in box.xyxy[0].tolist()]
                area_ratio = self.detection_area_ratio(coordinates, image)
                if self.minimum_area_ratio > 0.0 and area_ratio < self.minimum_area_ratio:
                    continue
                self.append_detection(
                    response, detections, output_label, score, coordinates, area_ratio
                )
                detected_labels.add(output_label)
            annotated = result.plot()

        for model_info in self.extra_models:
            extra_class_ids, extra_output_labels = self.extra_model_class_selection(
                model_info,
                requested_labels,
            )
            if not extra_class_ids:
                continue
            try:
                with self.lock:
                    result = model_info["model"].predict(
                        source=image,
                        classes=extra_class_ids,
                        conf=confidence,
                        iou=float(self.config["iou"]),
                        imgsz=int(self.config["image_size"]),
                        device="cpu",
                        verbose=False,
                    )[0]
            except Exception as exc:
                response.success = False
                response.message = "Extra model inference failed: %s" % exc
                return response

            for box in result.boxes:
                class_id = int(box.cls[0].item())
                label = str(result.names[class_id])
                output_label = extra_output_labels.get(label)
                if not output_label:
                    continue
                score = float(box.conf[0].item())
                coordinates = [float(value) for value in box.xyxy[0].tolist()]
                area_ratio = self.detection_area_ratio(coordinates, image)
                if self.minimum_area_ratio > 0.0 and area_ratio < self.minimum_area_ratio:
                    continue
                self.append_detection(
                    response, detections, output_label, score, coordinates, area_ratio
                )
                detected_labels.add(output_label)
                x1, y1, x2, y2 = [int(round(value)) for value in coordinates]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (180, 120, 40), 2)
                cv2.putText(
                    annotated,
                    self.resolve_yolo_label(output_label),
                    (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (180, 120, 40),
                    2,
                    cv2.LINE_AA,
                )

        for label, score, coordinates in self.detect_templates(
            image, template_labels, confidence
        ):
            self.append_detection(response, detections, label, score, coordinates)
            detected_labels.add(label)
            x1, y1, x2, y2 = [int(round(value)) for value in coordinates]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (40, 180, 40), 2)
            cv2.putText(
                annotated,
                label,
                (x1, max(0, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (40, 180, 40),
                2,
                cv2.LINE_AA,
            )

        # Use the trained YOLO model as the only final detector.
        # Color/shape fallback is intentionally disabled because it can classify venue textures as fire/extinguisher.

        response.success = True
        response.message = "detections=%d inference_seconds=%.3f" % (
            len(detections),
            time.time() - started,
        )
        report = {
            "image_topic": image_source,
            "requested_labels": requested_labels,
            "detections": detections,
            "message": response.message,
        }
        self.detections_pub.publish(
            String(json.dumps(report, ensure_ascii=False, sort_keys=True))
        )

        if "fire" in response.labels:
            for label, xmin, ymin, xmax, ymax in zip(
                response.labels,
                response.xmin,
                response.ymin,
                response.xmax,
                response.ymax,
            ):
                if label == "fire":
                    cv2.rectangle(
                        annotated,
                        (int(xmin), int(ymin)),
                        (int(xmax), int(ymax)),
                        (0, 140, 255),
                        2,
                    )
                    cv2.putText(
                        annotated,
                        "fire",
                        (int(xmin), max(0, int(ymin) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 140, 255),
                        2,
                        cv2.LINE_AA,
                    )
        annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        if source_header is not None:
            annotated_message.header = source_header
        else:
            annotated_message.header.stamp = rospy.Time.now()
            annotated_message.header.frame_id = "raicom_vision_file"
        self.annotated_pub.publish(annotated_message)
        return response

    def detect_templates(self, image, labels, confidence):
        if not labels:
            return []
        threshold = max(self.template_threshold, float(confidence))
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = []
        for label in labels:
            template = self.template_images[label]
            best = self.match_template(gray_image, template)
            if best is None:
                continue
            score, x1, y1, x2, y2 = best
            if score >= threshold:
                detections.append((label, score, [x1, y1, x2, y2]))
        detections.sort(key=lambda item: item[1], reverse=True)
        return detections

    def match_template(self, gray_image, template):
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        image_height, image_width = gray_image.shape[:2]
        template_height, template_width = template_gray.shape[:2]
        best = None
        for scale in self.template_scales:
            scaled_width = int(round(template_width * scale))
            scaled_height = int(round(template_height * scale))
            if scaled_width < 8 or scaled_height < 8:
                continue
            if scaled_width > image_width or scaled_height > image_height:
                continue
            scaled = cv2.resize(
                template_gray,
                (scaled_width, scaled_height),
                interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC,
            )
            result = cv2.matchTemplate(gray_image, scaled, cv2.TM_CCOEFF_NORMED)
            _, max_value, _, max_location = cv2.minMaxLoc(result)
            x1, y1 = max_location
            x2 = x1 + scaled_width
            y2 = y1 + scaled_height
            if best is None or max_value > best[0]:
                best = (float(max_value), float(x1), float(y1), float(x2), float(y2))
        return best

    @staticmethod
    def crop_template(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        non_white = np.logical_or(saturation > 18, value < 235).astype(np.uint8)
        kernel = np.ones((5, 5), dtype=np.uint8)
        non_white = cv2.morphologyEx(non_white, cv2.MORPH_CLOSE, kernel)
        coords = cv2.findNonZero(non_white)
        if coords is None:
            return image
        x, y, width, height = cv2.boundingRect(coords)
        pad = 4
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(image.shape[1], x + width + pad)
        y2 = min(image.shape[0], y + height + pad)
        return image[y1:y2, x1:x2].copy()

    @staticmethod
    def official_fire_mask(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(
            hsv,
            np.array([5, 80, 80], dtype=np.uint8),
            np.array([35, 255, 255], dtype=np.uint8),
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        return orange_mask

    @staticmethod
    def is_valid_fire_detection(image, coordinates):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = [int(round(value)) for value in coordinates]
        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(x1 + 1, min(width, x2))
        y2 = max(y1 + 1, min(height, y2))
        patch = image[y1:y2, x1:x2]
        mask = RaicomVisionNode.official_fire_mask(patch)
        patch_area = max(1, int((x2 - x1) * (y2 - y1)))
        orange_ratio = float(cv2.countNonZero(mask)) / float(patch_area)
        if orange_ratio < 0.08:
            return False
        component_count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for index in range(1, component_count):
            _, _, component_width, component_height, component_area = stats[index]
            if component_area < 450:
                continue
            if component_width < 18 or component_height < 25:
                continue
            aspect = float(component_height) / float(max(1, component_width))
            if aspect < 0.55 or aspect > 3.8:
                continue
            return True
        return False

    @staticmethod
    def detect_official_fire(image):
        mask = RaicomVisionNode.official_fire_mask(image)
        component_count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        for index in range(1, component_count):
            x, y, component_width, component_height, component_area = stats[index]
            if component_area < 700:
                continue
            if component_width < 22 or component_height < 30:
                continue
            if y > 360:
                continue
            aspect = float(component_height) / float(max(1, component_width))
            if aspect < 0.55 or aspect > 3.8:
                continue
            density = float(component_area) / float(max(1, component_width * component_height))
            score_value = component_area * density
            if best is None or score_value > best[0]:
                score = min(0.96, 0.55 + float(component_area) / 12000.0)
                best = (score_value, score, [float(x), float(y), float(x + component_width), float(y + component_height)])
        if best is None:
            return None
        return best[1], best[2]

    @staticmethod
    def official_extinguisher_mask(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_mask_a = cv2.inRange(
            hsv,
            np.array([0, 70, 45], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        red_mask_b = cv2.inRange(
            hsv,
            np.array([170, 70, 45], dtype=np.uint8),
            np.array([180, 255, 255], dtype=np.uint8),
        )
        red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)
        kernel = np.ones((3, 3), dtype=np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        return red_mask

    @staticmethod
    def detect_official_extinguisher(image):
        height, width = image.shape[:2]
        mask = RaicomVisionNode.official_extinguisher_mask(image)
        component_count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        for index in range(1, component_count):
            x, y, component_width, component_height, component_area = stats[index]
            if component_area < 250 or component_area > 6500:
                continue
            if component_width < 10 or component_width > 90:
                continue
            if component_height < 35 or component_height > 150:
                continue
            if y > int(height * 0.68):
                continue
            aspect = float(component_height) / float(max(1, component_width))
            if aspect < 1.15 or aspect > 6.5:
                continue
            density = float(component_area) / float(max(1, component_width * component_height))
            if density < 0.16:
                continue
            score_value = component_area * aspect * density
            if best is None or score_value > best[0]:
                score = min(0.97, 0.60 + float(component_area) / 9000.0)
                best = (score_value, score, [float(x), float(y), float(x + component_width), float(y + component_height)])
        if best is None:
            return None
        return best[1], best[2]

    @staticmethod
    def detection_area_ratio(coordinates, image):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = coordinates
        box_area = max(0.0, float(x2) - float(x1)) * max(0.0, float(y2) - float(y1))
        image_area = max(1.0, float(width) * float(height))
        return box_area / image_area

    @staticmethod
    def append_detection(response, detections, label, score, coordinates, area_ratio=None):
        response.labels.append(label)
        response.confidences.append(float(score))
        response.xmin.append(float(coordinates[0]))
        response.ymin.append(float(coordinates[1]))
        response.xmax.append(float(coordinates[2]))
        response.ymax.append(float(coordinates[3]))
        detections.append(
            {
                "label": label,
                "confidence": round(float(score), 5),
                "xmin": round(float(coordinates[0]), 2),
                "ymin": round(float(coordinates[1]), 2),
                "xmax": round(float(coordinates[2]), 2),
                "ymax": round(float(coordinates[3]), 2),
            }
        )
        if area_ratio is not None:
            detections[-1]["area_ratio"] = round(float(area_ratio), 6)

    @staticmethod
    def is_valid_extinguisher_detection(image, coordinates):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = [int(round(value)) for value in coordinates]
        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(x1 + 1, min(width, x2))
        y2 = max(y1 + 1, min(height, y2))
        patch = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        red_mask_a = cv2.inRange(
            patch,
            np.array([0, 70, 45], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        red_mask_b = cv2.inRange(
            patch,
            np.array([170, 70, 45], dtype=np.uint8),
            np.array([180, 255, 255], dtype=np.uint8),
        )
        red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)
        patch_area = max(1, int((x2 - x1) * (y2 - y1)))
        if float(cv2.countNonZero(red_mask)) / float(patch_area) < 0.06:
            return False
        component_count, _, stats, _ = cv2.connectedComponentsWithStats(red_mask, 8)
        for index in range(1, component_count):
            _, _, component_width, component_height, component_area = stats[index]
            if component_area < 250:
                continue
            if component_width < 10 or component_height < 45:
                continue
            if float(component_height) / float(max(1, component_width)) < 1.35:
                continue
            return True
        return False

if __name__ == "__main__":
    try:
        RaicomVisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("RAICOM vision failed: %s", exc)
        raise
