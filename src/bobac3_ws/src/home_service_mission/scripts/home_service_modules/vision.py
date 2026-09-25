#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rospy

from raicom_vision.srv import DetectObjects, DetectObjectsRequest


class VisionClient:
    def __init__(self, status, check_stopped):
        self.default_service_name = rospy.get_param(
            "~vision_service",
            "/raicom_vision/detect",
        )
        self.find_service_name = rospy.get_param(
            "~find_vision_service",
            self.default_service_name,
        )
        self.client = rospy.ServiceProxy(self.default_service_name, DetectObjects)
        self.find_client = rospy.ServiceProxy(self.find_service_name, DetectObjects)
        self.status = status
        self.check_stopped = check_stopped

    def client_for_labels(self, labels):
        find_labels = {"cell phone", "backpack", "phone", "bag", "手机", "书包"}
        if any(label in find_labels for label in labels):
            return self.find_client, self.find_service_name
        return self.client, self.default_service_name

    def detect_labels(
        self,
        labels,
        image_topic,
        confidence,
        sample_count,
        sample_interval,
        fallback_labels=None,
        fallback_enabled=False,
        max_labels=None,
        minimum_consecutive_count=1,
    ):
        fallback_labels = fallback_labels or []
        best_scores = {}
        current_streaks = {label: 0 for label in labels}
        accepted_labels = set()
        minimum_consecutive_count = max(1, int(minimum_consecutive_count))
        successful_samples = 0
        client, service_name = self.client_for_labels(labels)
        self.status.publish("vision_service_selected", service=service_name)
        for index in range(sample_count):
            self.check_stopped()
            request = DetectObjectsRequest()
            request.image_topic = image_topic
            request.labels = labels
            request.confidence = confidence
            try:
                response = client(request)
            except Exception as exc:
                rospy.logwarn("Vision request failed: %s", exc)
                continue
            if not response.success:
                rospy.logwarn("Vision request failed: %s", response.message)
                continue
            successful_samples += 1
            frame_scores = {}
            for label, score in zip(response.labels, response.confidences):
                if label in labels:
                    frame_scores[label] = max(frame_scores.get(label, 0.0), float(score))
            for label in labels:
                if label in frame_scores:
                    current_streaks[label] = current_streaks.get(label, 0) + 1
                    if current_streaks[label] >= minimum_consecutive_count:
                        accepted_labels.add(label)
                        best_scores[label] = max(best_scores.get(label, 0.0), frame_scores[label])
                else:
                    current_streaks[label] = 0
            if index + 1 < sample_count:
                time.sleep(sample_interval)

        if successful_samples == 0:
            if fallback_enabled:
                ordered_fallback = [label for label in fallback_labels if label in labels]
                self.status.publish("vision_fallback", labels=ordered_fallback)
                return ordered_fallback
            raise RuntimeError("No successful vision samples")

        filtered_scores = {
            label: score for label, score in best_scores.items() if label in accepted_labels
        }
        ordered = sorted(filtered_scores, key=lambda label: (-filtered_scores[label], labels.index(label)))
        if max_labels is not None and int(max_labels) > 0:
            ordered = ordered[: int(max_labels)]
        self.status.publish(
            "vision_result",
            labels=ordered,
            scores=filtered_scores,
            minimum_consecutive_count=minimum_consecutive_count,
        )
        return ordered
