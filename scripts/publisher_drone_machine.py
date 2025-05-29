#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import os
import cv2
import torch
import random
import time
from geometry_msgs.msg import Point
from drone_pkg.msg import DroneStatusDroneMachine, Detection


def allocate_drone_id():
    # get or initialize a global counter
    if not rospy.has_param("/next_drone_index"):
        rospy.set_param("/next_drone_index", 1)
    idx = rospy.get_param("/next_drone_index")
    rospy.set_param("/next_drone_index", idx + 1)
    return idx


def main():
    # initialize ROS node
    rospy.init_node('publisher_drone_machine', anonymous=True)

    # allocate a unique drone ID
    drone_id = allocate_drone_id()

    # parameters
    image_dir  = rospy.get_param('~image_dir', '/path/to/images')
    model_name = rospy.get_param('~model_name', 'yolov5s')
    model_path = rospy.get_param('~model_path', None)
    conf_thres = rospy.get_param('~conf_thres', 0.5)
    rate_hz    = rospy.get_param('~rate', 1.0)



    # load YOLO model
    if model_path:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
    else:
        model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
    model.conf = conf_thres

    # gather image files
    if not os.path.isdir(image_dir):
        rospy.logfatal("Image directory does not exist: {}".format(image_dir))
        return
    image_files = sorted([f for f in os.listdir(image_dir)
                          if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
    if not image_files:
        rospy.logfatal("No images found in directory: {}".format(image_dir))
        return

    # publisher setup
    pub = rospy.Publisher("/drone/{}/status".format(drone_id), DroneStatusDroneMachine, queue_size=1)
    rate = rospy.Rate(rate_hz)
    idx = 0

    rospy.loginfo("Starting YOLO publisher for drone {} with {} images at {} Hz".format(drone_id,len(image_files),rate_hz))
    prev_pub_time = None

    while not rospy.is_shutdown():
        start_time = time.time()
        img_path = os.path.join(image_dir, image_files[idx])

        # load image
        img = cv2.imread(img_path)
        if img is None:
            rospy.logerr("Failed to load image: {}".format(img_path))
            idx = (idx + 1) % len(image_files)
            rate.sleep()
            continue

        # run inference
        try:
            inf_start = time.time()
            results = model(img[:, :, ::-1])  # BGR -> RGB
            df = results.pandas().xyxy[0]
            inf_end = time.time()
        except Exception as e:
            rospy.logerr("YOLO inference error on {}: {}".format(img_path,e))
            idx = (idx + 1) % len(image_files)
            rate.sleep()
            continue

        end_time = time.time()

        # build message
        msg = DroneStatusDroneMachine()
        msg.header.stamp = rospy.Time.now()
        msg.drone_id = drone_id
        msg.position = Point(
            random.uniform(-10, 10),
            random.uniform(-10, 10),
            random.uniform(0, 5),
        )
        msg.detections = []

        for _, row in df.iterrows():
            det = Detection()
            det.x1 = int(row.xmin)
            det.y1 = int(row.ymin)
            det.x2 = int(row.xmax)
            det.y2 = int(row.ymax)
            det.class_id = int(row['class'])
            msg.detections.append(det)

        msg.emergency = bool(len(msg.detections))

        # publish
        pub.publish(msg)
        rospy.loginfo("Published {} detections from {}".format(len(msg.detections),image_files[idx]))

        # measure publish rate
        now = time.time()
        if prev_pub_time is not None:
            fps = 1.0 / (now - prev_pub_time) if (now - prev_pub_time) > 0 else float('inf')
            rospy.loginfo("[drone {}] Publish rate: {:.2f} fps, Processing Time {:.1f} ms".format(drone_id,fps,((end_time - start_time)*1000)))
        prev_pub_time = now

        # advance index and sleep
        idx = (idx + 1) % len(image_files)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

