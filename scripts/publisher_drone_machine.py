#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import os
import cv2
import random
import time
from geometry_msgs.msg import Point
from drone_package.msg import DroneStatusDroneMachine,WayPoint
from ultralytics import YOLO


class PublisherDroneMachine:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('publisher_drone_machine', anonymous=True)

        #Time attributes
        self.processing_time = None
        self.msg_publisher_time = None
        self.waypoint_tx_delay= None


        # Drone Features
        self.drone_id = self.allocate_drone_id()
        self.position = Point(x=random.uniform(-10,10),y=random.uniform(-10,10),z=random.uniform(0,5))
        self.action = "go"

        # Load parameters
        self.image_dir = rospy.get_param('~image_dir', '/path/to/images')
        self.model_name = rospy.get_param('~model_name', 'yolov5s')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)
        self.rate_hz = rospy.get_param('~rate', 1.0)
        self.compression_ratio = rospy.get_param("~compression_ratio", 1)

        # Setup publisher
        topic_name = "/drone_status_drone_machine/status"
        self.pub = rospy.Publisher(topic_name, DroneStatusDroneMachine, queue_size=10)
        self.rate = rospy.Rate(self.rate_hz)

        
        # Load model
        self.model = YOLO(self.model_path if self.model_path else self.model_name)
        self.model.conf = self.conf_thres


        # Prepare images
        if not os.path.isdir(self.image_dir):
            rospy.logfatal("Image directory does not exist: {}".format(self.image_dir))
            raise FileNotFoundError(self.image_dir)

        self.image_files = sorted([f for f in os.listdir(self.image_dir)
                                   if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        if not self.image_files:
            rospy.logfatal("No images found in directory: {}".format(self.image_dir))
            raise FileNotFoundError("No images in {}".format(self.image_dir))

        

        rospy.loginfo("Starting YOLO publisher for drone {} with {} images at {} Hz".format(self.drone_id,len(self.image_files),self.rate_hz))

        self.image_idx = 0
        self.prev_pub_time = None


        #Waypoint Subscriber
        waypoint_sub =  rospy.Subscriber("/drone_waypoint_drone_machine/waypoint", WayPoint, self.waypoint_callback)


    def allocate_drone_id(self):
        if not rospy.has_param("/drone_machine_drone_index"):
            rospy.set_param("/drone_machine_drone_index", 1)
        idx = rospy.get_param("/drone_machine_drone_index")
        rospy.set_param("/drone_machine_drone_index", idx + 1)
        return idx


    def load_image(self, path):
        img = cv2.imread(path)
        if img is not None and self.compression_ratio > 1:
            h, w = img.shape[:2]
            img = cv2.resize(img, (w // self.compression_ratio, h // self.compression_ratio), interpolation=cv2.INTER_AREA)
        return img
    
    def advance_index(self):
        self.image_idx = (self.image_idx + 1) % len(self.image_files)


    def start(self):
        while not rospy.is_shutdown():
            msg_publisher_time_start = time.time()
            start_time = time.time()
            img_path = os.path.join(self.image_dir, self.image_files[self.image_idx])
            img = self.load_image(img_path)

            if img is None:
                rospy.logerr("Failed to load image: {}".format(img_path))
                self.advance_index()
                self.rate.sleep()
                continue
            

            proc_start = time.time()
            try:
                results = self.model(img[:, :, ::-1])  # BGR -> RGB
            except Exception as e:
                rospy.logerr("YOLO inference error on {}: {}".format(img_path,e))
                self.advance_index()
                self.rate.sleep()
                continue
            proc_end = time.time()
            self.processing_time = (proc_end- proc_start)


            # Build and publish message
            msg = DroneStatusDroneMachine()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = str(self.image_idx)
            msg.drone_id = self.drone_id
            msg.position = self.position
            msg.detections = len(results[0].boxes)

            self.pub.publish(msg)

            rospy.loginfo("Published {} detections from {}".format(msg.detections,self.image_files[self.image_idx]))

            # Print FPS info
            now = time.time()
            if self.prev_pub_time:
                fps = 1.0 / (now - self.prev_pub_time) if (now - self.prev_pub_time) > 0 else float('inf')
                self.msg_publisher_time = ((now - start_time) * 1000 )- self.processing_time  # substract yolo model time
                rospy.loginfo("[drone {}] Publish rate: {:.2f} fps, Processing Time: {:.1f} ms".format(self.drone_id,fps,self.msg_publisher_time))
            self.prev_pub_time = now

            self.advance_index()
            self.rate.sleep()


    def waypoint_callback(self, msg):
        # capture arrival time
        recv_time = time.time()
        # transmission delay (ms)
        self.waypoint_tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000
        if msg.drone_id == self.drone_id:
            rospy.loginfo(
                "[{}] Received WayPoint for this drone: (x={:.2f}, y={:.2f}, z={:.2f}) | Action: {}".format(
                    self.drone_id, msg.position.x, msg.position.y, msg.position.z, msg.action))     
            self.position = msg.position
            self.action = msg.action


if __name__ == '__main__':
    try:
        drone_publisher = PublisherDroneMachine()
        drone_publisher.start()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("DronePublisher failed to start: {}".format(e))
