#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import re
import rospy
import cv2
import time
from threading import Lock
from drone_package.msg import DroneStatusDroneMachine
from cv_bridge import CvBridge

class SubscriberDroneMachine:
    def __init__(self):
        rospy.init_node('subscriber_drone_machine')
        self.bridge = CvBridge()
        self.status_subs = {}
        self.last_status = {}   # drone_id -> last receipt time
        self.lock = Lock()


        # Periodically scan for new drone topics
        rospy.Timer(rospy.Duration(2.0), self.scan_topics)



    def scan_topics(self, event):
        # List all published topics
        for topic, ttype in rospy.get_published_topics():
            # Subscribe to status topics
            m = re.match(r"/drone/([^/]+)/status", topic)
            if m and topic not in self.status_subs:
                drone_id = m.group(1)
                sub = rospy.Subscriber(topic, DroneStatusDroneMachine, self.status_cb, callback_args=drone_id)
                with self.lock:
                    self.status_subs[topic] = sub
                rospy.loginfo("Subscribed to status of {}".format(drone_id))

    def status_cb(self, msg, drone_id):
        recv_time = time.time()
        # transmission delay ms
        tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000
        # rate fps
        if drone_id in self.last_status:
            dt = recv_time - self.last_status[drone_id]
            fps = 1.0 / dt if dt > 0 else float('inf')
            rospy.loginfo("[{}] Rate: {:.2f} fps; Transmission Delay: {:.1f} ms".format(drone_id,fps,tx_delay))
        self.last_status[drone_id] = recv_time

        pos = msg.position
        rospy.loginfo("[%s] pos=(%.1f,%.1f,%.1f) detections=%d" % (
            drone_id,
            pos.x, pos.y, pos.z,
            msg.detections
        ))


if __name__ == '__main__':
    monitor = SubscriberDroneMachine()
    rospy.spin()

