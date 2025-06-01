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
from drone_package.msg import DroneStatusDroneMachine  #, Detection
from ultralytics import YOLO


class PublisherDroneMachine:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('publisher_drone_machine', anonymous=True)

        # Allocate drone ID
        self.drone_id = self.allocate_drone_id()

        # Load parameters
        self.image_dir = rospy.get_param('~image_dir', '/path/to/images')
        self.model_name = rospy.get_param('~model_name', 'yolov5s')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)
        self.rate_hz = rospy.get_param('~rate', 1.0)
        self.compression_ratio = rospy.get_param("~compression_ratio", 1)

        # Load model
        self.model = YOLO(self.model_path if self.model_path else self.model_name)

        # Prepare images
        if not os.path.isdir(self.image_dir):
            rospy.logfatal("Image directory does not exist: {}".format(self.image_dir))
            raise FileNotFoundError(self.image_dir)

        self.image_files = sorted([f for f in os.listdir(self.image_dir)
                                   if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        if not self.image_files:
            rospy.logfatal("No images found in directory: {}".format(self.image_dir))
            raise FileNotFoundError("No images in {}".format(self.image_dir))

        # Setup publisher
        topic_name = f"/drone/{self.drone_id}/status"
        self.publisher = rospy.Publisher(topic_name, DroneStatusDroneMachine, queue_size=10)
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo(f"Starting YOLO publisher for drone {self.drone_id} with {len(self.image_files)} images at {self.rate_hz} Hz")

        self.idx = 0
        self.prev_pub_time = None

    def allocate_drone_id(self):
        if not rospy.has_param("/next_drone_index"):
            rospy.set_param("/next_drone_index", 1)
        idx = rospy.get_param("/next_drone_index")
        rospy.set_param("/next_drone_index", idx + 1)
        return idx

    def load_image(self, path):
        img = cv2.imread(path)
        if img is not None and self.compression_ratio > 1:
            h, w = img.shape[:2]
            img = cv2.resize(img, (w // self.compression_ratio, h // self.compression_ratio), interpolation=cv2.INTER_AREA)
        return img

    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            img_path = os.path.join(self.image_dir, self.image_files[self.idx])
            img = self.load_image(img_path)

            if img is None:
                rospy.logerr(f"Failed to load image: {img_path}")
                self.advance_index()
                self.rate.sleep()
                continue

            try:
                results = self.model(img[:, :, ::-1])  # BGR -> RGB
            except Exception as e:
                rospy.logerr(f"YOLO inference error on {img_path}: {e}")
                self.advance_index()
                self.rate.sleep()
                continue

            # Build and publish message
            msg = DroneStatusDroneMachine()
            msg.header.stamp = rospy.Time.now()
            msg.drone_id = self.drone_id
            msg.position = Point(
                random.uniform(-10, 10),
                random.uniform(-10, 10),
                random.uniform(0, 5),
            )
            msg.detections = len(results[0].boxes)

            self.publisher.publish(msg)
            rospy.loginfo(f"Published {msg.detections} detections from {self.image_files[self.idx]}")

            # Print FPS info
            now = time.time()
            if self.prev_pub_time:
                fps = 1.0 / (now - self.prev_pub_time) if (now - self.prev_pub_time) > 0 else float('inf')
                proc_time = (now - start_time) * 1000
                rospy.loginfo(f"[drone {self.drone_id}] Publish rate: {fps:.2f} fps, Processing Time: {proc_time:.1f} ms")
            self.prev_pub_time = now

            self.advance_index()
            self.rate.sleep()

    def advance_index(self):
        self.idx = (self.idx + 1) % len(self.image_files)


if __name__ == '__main__':
    try:
        drone_publisher = PublisherDroneMachine()
        drone_publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"DronePublisher failed to start: {e}")
