#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

import rospy
from face_rec.msg import face_data, face_results


def make_face_message(frame_id, xmin, xmax, ymin, ymax):
    message = face_results()
    face = face_data()
    face.header.stamp = rospy.Time.now()
    face.header.frame_id = frame_id
    face.xmin = float(xmin)
    face.xmax = float(xmax)
    face.ymin = float(ymin)
    face.ymax = float(ymax)
    message.face_data.append(face)
    return message


def main():
    parser = argparse.ArgumentParser(
        description="Publish a manual face_rec/face_results trigger once for national service missions."
    )
    parser.add_argument("--topic", default="/face_detection")
    parser.add_argument("--frame-id", default="manual_face_trigger")
    parser.add_argument("--count", type=int, default=15)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--xmin", type=float, default=220.0)
    parser.add_argument("--xmax", type=float, default=420.0)
    parser.add_argument("--ymin", type=float, default=80.0)
    parser.add_argument("--ymax", type=float, default=320.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("manual_face_trigger", anonymous=True)
    publisher = rospy.Publisher(args.topic, face_results, queue_size=10)
    rate = rospy.Rate(args.rate)

    rospy.sleep(0.5)
    publish_count = max(1, int(args.count))
    for _ in range(publish_count):
        if rospy.is_shutdown():
            break
        publisher.publish(
            make_face_message(
                args.frame_id,
                args.xmin,
                args.xmax,
                args.ymin,
                args.ymax,
            )
        )
        rate.sleep()

    print(
        "MANUAL_FACE_TRIGGER_PUBLISHED topic=%s count=%d frame_id=%s"
        % (args.topic, publish_count, args.frame_id)
    )


if __name__ == "__main__":
    main()
