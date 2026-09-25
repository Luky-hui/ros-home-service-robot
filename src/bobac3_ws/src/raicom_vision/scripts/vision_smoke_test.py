#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from raicom_vision.srv import DetectObjects, DetectObjectsRequest


def main():
    rospy.init_node("raicom_vision_smoke_test", anonymous=True)
    service_name = "/raicom_vision/detect"
    rospy.wait_for_service(service_name, timeout=15.0)
    client = rospy.ServiceProxy(service_name, DetectObjects)
    request = DetectObjectsRequest()
    request.image_topic = rospy.get_param("~image_topic", "/head_camera/image_raw")
    request.labels = rospy.get_param(
        "~labels",
        ["fire", "fire extinguisher"],
    )
    request.confidence = float(rospy.get_param("~confidence", 0.10))
    response = client(request)
    if not response.success:
        raise RuntimeError(response.message)
    print(
        "RAICOM_VISION_SMOKE_TEST_OK %s labels=%s confidences=%s"
        % (response.message, list(response.labels), list(response.confidences))
    )


if __name__ == "__main__":
    main()
