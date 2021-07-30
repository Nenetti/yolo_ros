#! /usr/bin/env python

import rospy

from yolov5_object_detector import YoloObjectDetector

if __name__ == "__main__":
    rospy.init_node("yolov5_ros")
    YoloObjectDetector()
    rospy.spin()
