#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import random
import cv2
import os
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from drone_package.msg import DroneStatusMainMachine, WayPoint


class PublisherMainMachine:
    def __init__(self):

        # fixed node name so that /publisher_idx is shared
        rospy.init_node('publisher_main_machine', anonymous=True)

        #Time attributes
        self.msg_publisher_time = None
        self.waypoint_tx_delay= None

        # Drone Features
        self.drone_id = self.allocate_drone_id()
        self.position = Point(x=random.uniform(-10,10),y=random.uniform(-10,10),z=random.uniform(0,5))
        self.action = "go"

        #rospy parameters
        self.compression_ratio = rospy.get_param("~compression_ratio", 1)
        self.image_dir = rospy.get_param('~image_dir', 'images')
        self.rate_hz = rospy.get_param('~rate', 1)

        # choose topic for DroneStatus
        topic = "/drone_status_main_machine/status"
        self.pub = rospy.Publisher(topic, DroneStatusMainMachine, queue_size=10)
        rospy.loginfo("[{}] Publishing DroneStatus on {}".format(self.drone_id,topic))

        # image source (camera or folder)
        self.bridge = CvBridge()
        self.image_idx = 0
        
        if not os.path.isdir(self.image_dir):
            rospy.logerr("Bad image_dir: {}".format(self.image_dir))
            rospy.signal_shutdown("image_dir")
        self.image_files = sorted([
            f for f in os.listdir(self.image_dir)
            if f.lower().endswith(('.png','.jpg','.jpeg'))
        ])
        if not self.image_files:
            rospy.logerr("No images found")
            rospy.signal_shutdown("images")

        # publish rate
        self.rate = rospy.Rate(self.rate_hz)
        #rospy.loginfo("Publishing at {} Hz".format(self.rate_hz))

        # metric: track last publish time
        self.prev_pub_time = None

        #Waypoint subscriber
        waypoint_sub =  rospy.Subscriber("/drone_waypoint_main_machine/waypoint", WayPoint, self.waypoint_callback)

    def allocate_drone_id(self):
        if not rospy.has_param("/main_machine_drone_index"):
            rospy.set_param("/main_machine_drone_index", 1)
        idx = rospy.get_param("/main_machine_drone_index")
        rospy.set_param("/main_machine_drone_index", idx + 1)
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
            path = os.path.join(self.image_dir, self.image_files[self.image_idx])
            img = self.load_image(path)
            


            # build Image msg
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = str(self.image_idx)

            # build DroneStatus
            status = DroneStatusMainMachine(
                header=img_msg.header,  # header msg
                drone_id=self.drone_id,  # drone index
                position=self.position, # drone position
                image=img_msg   # image 
            )

            # publish and measure rate
            self.pub.publish(status)
            msg_publisher_time_end = time.time()
            self.msg_publisher_time = (msg_publisher_time_end- msg_publisher_time_start) * 1000
            
            now = time.time()
            if self.prev_pub_time is not None:
                interval = now - self.prev_pub_time
                fps = 1.0 / interval if interval > 0 else float('inf')
                rospy.loginfo("[{}] Video rate: {:.2f} fps".format(self.drone_id,fps))
            self.prev_pub_time = now

            self.advance_index()  # overpass files for loop  !! IMPORTANT FOR LOOP
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
        node = PublisherMainMachine()
        node.start()
    except rospy.ROSInterruptException:
        pass

