# YOLO v5 ROS

<!--
# ==================================================================================================
#
#   Overview
#
# ==================================================================================================
--->
# Overview <a id="Overview"></a>

The `yolov5_ros` package provides real-time object detection.

This package contains submodule of https://github.com/ultralytics/yolov5.

**Content:**

* [Overview](#Overview)
* [Dependencies](#Dependencies)
* [Weights](#Weights)
* [Example](#Example)
* [Nodes](#Nodes)


<!--
# ==================================================================================================
#
#   Dependencies
#
# ==================================================================================================
--->
## Dependencies <a id="Dependencies"></a>

Code wrapped from [ultralytics/yolov5](https://github.com/ultralytics/yolov5) at:

* URL: `https://github.com/ultralytics/yolov5`
* Branch: `master`
* Commit: [`8d3c3ef45ce1d530aa3751f6187f18cfd9c40791`](https://github.com/ultralytics/yolov5/tree/8d3c3ef45ce1d530aa3751f6187f18cfd9c40791)

Original `README.md`: https://github.com/ultralytics/yolov5/blob/8d3c3ef45ce1d530aa3751f6187f18cfd9c40791/README.md
Original `LICENSE`: https://github.com/ultralytics/yolov5/blob/8d3c3ef45ce1d530aa3751f6187f18cfd9c40791/LICENSE

<!--
# ==================================================================================================
#
#   Weights
#
# ==================================================================================================
--->
## Weights

The weight from YOLO v5 can be found here

    roscd yolov5_ros/weights
    wget https://github.com/ultralytics/yolov5/releases/download/v5.0/yolov5m.pt

<!--
# ==================================================================================================
#
#   Example
#
# ==================================================================================================
--->
## Example <a id="Example"></a>

Start the server.

    roslaunch yolov5_ros yolov5_ros.launch

<!--
# ==================================================================================================
#
#   Nodes
#
# ==================================================================================================
--->
## Node <a id="Node"></a>

This is the main YOLO ROS: Real-Time Object Detection for ROS node. It uses the camera measurements to detect pre-learned objects in the frames.

<!--
# ==================================================================================================
#   Subscribed Topics
# ==================================================================================================
--->
### Subscribed Topics

* **`/input/activation`** ([std_msgs/Bool])  
  The camera measurements.

  
* **`/input/image/compressed`** ([sensor_msgs/CompressedImage])  
  The camera measurements.

<!--
# ==================================================================================================
#   Published Topics
# ==================================================================================================
--->
### Published Topics

* **`output/image/compressed`** ([sensor_msgs/CompressedImage])  
  Publishes an array of bounding boxes that gives information of the position and size of the bounding box in pixel coordinates.


* **`output/bounding_boxes`** ([yolo_ros_msgs/BoundingBoxes])  
  Publishes an image of the detection image including the bounding boxes.

<!--
# ==================================================================================================
#   Actions
# ==================================================================================================
--->
### Actions

* **`check_for_objects`** ([yolo_ros_msgs/CheckForObjectsAction])  
  Sends an action with an image and the result is an array of bounding boxes.

<!--
# ==================================================================================================
#   Parameters
# ==================================================================================================
--->
### Parameters

* **`~is_autostart`** (bool, default: true)  
  Prepare

  
* **`~frame_rate`** (int, default: 60)
  Prepare

  
* **`~is_publish_bounding_box_image`** (bool, default: true)  
  Prepare


* **`~is_publish_bounding_box`** (bool, default: true)  
  Prepare


* **`~score_threshold`** (float, default: 0.5)  
  Prepare


* **`~iou_threshold`** (float, default: 0.45)  
  Prepare


* **`~config`** (str, default: "$(find yolov5_ros)/yolov5/models/yolov5m.yaml")  
  Prepare


* **`~weights_path`** (str, default: "$(find yolov5_ros)/weights/yolov5m.pt")  
  Prepare


* **`~device`** (str, default: "cuda")  
  Prepare
  