#! /usr/bin/env python

import sys
import time
from pathlib import Path

import actionlib
import cv2
import cv_bridge
import numpy as np
import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import torch
import yolo_ros_msgs.msg as yolo_ros_msgs
from torch import nn

path = Path(__file__).parent.parent
sys.path.append(str(path))
from yolov5.models.yolo import Detect, Model
from yolov5.utils.general import non_max_suppression
from yolov5.utils.plots import colors


class YoloObjectDetector:

    def __init__(self):

        self._bridge = cv_bridge.CvBridge()

        self._is_activated = rospy.get_param("~is_autostart")
        self._frame_rate = rospy.get_param("~frame_rate")

        self._is_publish_bounding_box_image = rospy.get_param("~is_publish_bounding_box_image")

        self._is_publish_bounding_box = rospy.get_param("~is_publish_bounding_box")

        self._score_threshold = rospy.get_param("~score_threshold")
        self._iou_threshold = rospy.get_param("~iou_threshold")

        self._frame_time = 1.0 / self._frame_rate
        self._before_time = 0

        config_path = rospy.get_param("~config")
        weight_path = rospy.get_param("~weights_path")
        self._device = rospy.get_param("~device")

        self._model, self._classes = self._init_model(config_path, weight_path, self._device)

        # Subscriber
        rospy.Subscriber("/input/activation", std_msgs.Bool, callback=self._activation_callback)
        rospy.Subscriber("/input/image/compressed", sensor_msgs.CompressedImage, self._image_callback, queue_size=10)

        # Publisher
        self._detection_image_publisher = rospy.Publisher("output/image/compressed", sensor_msgs.CompressedImage, queue_size=1)
        self._bounding_boxes_publisher = rospy.Publisher("output/bounding_boxes", yolo_ros_msgs.BoundingBoxes, queue_size=1)

        # ActionServer
        self._server = actionlib.SimpleActionServer("check_for_objects", yolo_ros_msgs.CheckForObjectsAction, self.action_call_back, auto_start=False)
        self._server.start()

    @staticmethod
    def _init_model(config, weight_path, device):
        """

        Args:
            config:
            weight_path:
            device:

        Returns:

        """
        rospy.loginfo("Loading model ...")

        model = Model(cfg=config)
        load_pt = torch.load(weight_path)

        model.load_state_dict(load_pt["model"].state_dict())
        model.names = load_pt["model"].names

        for module in model.modules():
            if type(module) in [nn.Hardswish, nn.LeakyReLU, nn.ReLU, nn.ReLU6, nn.SiLU, Detect, Model]:
                module.inplace = True

        model.eval()
        model.requires_grad_(False)
        model = model.to(device)
        classes = model.names

        rospy.loginfo("Ready...")
        return model, classes

    # ==================================================================================================
    #
    #   Callback
    #
    # ==================================================================================================
    def _activation_callback(self, msg):
        """
        Args:
            msg (std_msgs.Bool):
        """
        is_activate = msg.data

        if is_activate != self._is_activated:
            self._is_activated = is_activate
        else:
            rospy.loginfo(f"Activation flag is already {self._is_activated}")

    def _image_callback(self, msg):
        """
        Args:
            msg (sensor_msgs.CompressedImage):
        """
        if not self._is_activated or self._model is None:
            return

        now_time = time.time()
        if (now_time - self._before_time) < self._frame_time:
            return

        self._before_time = now_time
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
        bounding_boxes = self._predict_image(image)

        self._publish_detection_image(image, bounding_boxes, self._classes)
        self._publish_bounding_boxes(bounding_boxes, self._classes)

        elapsed_time = time.time() - now_time
        rospy.logdebug("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    def action_call_back(self, goal):
        """

        Args:
            goal (yolo_ros_msgs.CheckForObjectsGoal):

        Returns:

        """
        rospy.logdebug("Receive ActionGoal")
        image = self._bridge.compressed_imgmsg_to_cv2(goal.image, desired_encoding="rgb8")
        bounding_boxes = self._predict_image(image)
        if not self._server.is_preempt_requested():
            result = yolo_ros_msgs.CheckForObjectsResult()
            result.bounding_boxes = self._generate_bounding_box_msg(bounding_boxes, self._classes)
            result.id = goal.id
            self._server.set_succeeded(result)
            rospy.logdebug("Return ActionResult")

    # ==================================================================================================
    #
    #   Instance Method (Public)
    #
    # ==================================================================================================
    def _predict_image(self, image):
        """

        Args:
            image (np.ndarray):

        Returns:

        """
        batch_image = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0).type(torch.float32).to(self._device) / 255.0
        result = self._model(batch_image)[0]
        result = non_max_suppression(result, conf_thres=self._score_threshold, iou_thres=self._iou_threshold)
        bounding_boxes = []
        for i, detections in enumerate(result):
            detections = detections.detach().cpu().numpy()
            for detection in reversed(detections):
                (x0, y0, x1, y1), confidence, class_id = detection[:4].astype(np.int32), detection[4], int(detection[5])
                bounding_boxes.append((x0, y0, x1, y1, confidence, class_id))

        return bounding_boxes

    def _publish_detection_image(self, image, bounding_boxes, classes):
        """
        Args:
            image:
            bounding_boxes:
            classes:
        """
        if not self._is_publish_bounding_box_image:
            return

        bounding_box_image = self._generate_result_images(image, bounding_boxes, classes)
        bounding_box_image_msg = self._bridge.cv2_to_compressed_imgmsg(bounding_box_image)
        bounding_box_image_msg.header.stamp = rospy.Time.now()
        self._detection_image_publisher.publish(bounding_box_image_msg)

    def _publish_bounding_boxes(self, bounding_boxes, classes):
        """
        Args:
            bounding_boxes:
            classes:

        """
        if not self._is_publish_bounding_box:
            return

        bounding_boxes_msg = self._generate_bounding_box_msg(bounding_boxes, classes)
        self._bounding_boxes_publisher.publish(bounding_boxes_msg)

    # ==================================================================================================
    #
    #   Instance Method (Private)
    #
    # ==================================================================================================
    @staticmethod
    def _generate_result_images(image, bounding_boxes, classes):
        """
        Args:
            image:
            bounding_boxes:
            classes:

        Returns:

        """
        output_image = image.copy()
        text_config = {'fontFace': cv2.FONT_HERSHEY_DUPLEX, 'fontScale': 0.6, 'thickness': 1}
        for i, bounding_box in enumerate(bounding_boxes):
            x0, y0, x1, y1, confidence, class_id = bounding_box
            color = colors(class_id, bgr=True)
            cv2.rectangle(output_image, (x0, y0), (x1, y1), color, thickness=2)
            label_name = classes[class_id]
            label_str = f"{label_name} {confidence * 100:.1f}"
            size, baseline = cv2.getTextSize(text=label_str, **text_config)
            cv2.rectangle(output_image, (x0, y0), (x0 + size[0], y0 + size[1]), (255, 255, 255), cv2.FILLED)
            cv2.putText(output_image, org=(x0, y0 + size[1]), color=(255, 0, 0), text=label_str, **text_config)

        return cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _generate_bounding_box_msg(bounding_boxes, classes):
        """

        Args:
            bounding_boxes:
            classes:

        Returns:

        """
        msg = yolo_ros_msgs.BoundingBoxes()
        msg.header.stamp = rospy.Time.now()
        for i, bounding_box in enumerate(bounding_boxes):
            x0, y0, x1, y1, confidence, class_id = bounding_box

            bounding_box_msg = yolo_ros_msgs.BoundingBox()
            bounding_box_msg.xmin = x0
            bounding_box_msg.ymin = y0
            bounding_box_msg.xmax = x1
            bounding_box_msg.ymax = y1
            bounding_box_msg.probability = confidence
            bounding_box_msg.Class = classes[class_id]
            bounding_box_msg.id = i
            msg.bounding_boxes.append(bounding_box_msg)

        return msg
