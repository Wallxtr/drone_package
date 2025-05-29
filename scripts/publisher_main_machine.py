#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import random
import os
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from drone_pkg.msg import DroneStatusMainMachine  #  adjust to your package

class DroneStatusPublisher:
    def __init__(self):
        # fixed node name so that /publisher_idx is shared
        rospy.init_node('drone_status_publisher', anonymous=True)

        # retrieve and bump the global index
        idx = rospy.get_param('/publisher_idx', 0)
        rospy.set_param('/publisher_idx', idx + 1)
        self.drone_id = idx

        # choose topic for DroneStatus
        topic = f'/drone/status/{self.drone_id}'
        self.pub = rospy.Publisher(topic, DroneStatusMainMachine, queue_size=10)
        rospy.loginfo("[{}] Publishing DroneStatus on {}".format(self.drone_id,topic))

        # image source (camera or folder)
        self.bridge = CvBridge()
        self.use_camera = rospy.get_param('~use_camera', False)
        if self.use_camera:
            import cv2
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                rospy.logerr("Camera init failed")
                rospy.signal_shutdown("camera")
        else:
            self.image_dir = rospy.get_param('~image_dir', 'images')
            if not os.path.isdir(self.image_dir):
                rospy.logerr("Bad image_dir: {}".format(self.image_dir))
                rospy.signal_shutdown("image_dir")
            self.files = sorted([
                f for f in os.listdir(self.image_dir)
                if f.lower().endswith(('.png','.jpg','.jpeg'))
            ])
            if not self.files:
                rospy.logerr("No images found")
                rospy.signal_shutdown("images")

        # publish rate
        rate_hz = rospy.get_param('~rate', 1)
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("Publishing at {} Hz".format(rate_hz))

        # metric: track last publish time
        self.prev_pub_time = None

    def start(self):
        import cv2
        idx = 0
        while not rospy.is_shutdown():
            # grab next frame
            if self.use_camera:
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logwarn("camera grab failed")
                    continue
                img = frame
            else:
                path = os.path.join(self.image_dir, self.files[idx])
                img = cv2.imread(path)
                if img is None:
                    rospy.logwarn("load failed: {}".format(path))
                idx = (idx + 1) % len(self.files)

            # build Image msg
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = rospy.Time.now()

            # build DroneStatus
            status = DroneStatusMainMachine(
                header=img_msg.header,
                drone_id=self.drone_id,
                position=Point(
                    x=random.uniform(-10,10),
                    y=random.uniform(-10,10),
                    z=random.uniform(0,5)
                ),
                image=img_msg
            )

            # publish and measure rate
            self.pub.publish(status)
            now = time.time()
            if self.prev_pub_time is not None:
                interval = now - self.prev_pub_time
                fps = 1.0 / interval if interval > 0 else float('inf')
                rospy.loginfo("[{}] Video rate: {:.2f} fps".format(self.drone_id,fps))
            self.prev_pub_time = now

            self.rate.sleep()

    def __del__(self):
        if self.use_camera and hasattr(self, 'cap'):
            self.cap.release()


if __name__ == '__main__':
    try:
        node = DroneStatusPublisher()
        node.start()
    except rospy.ROSInterruptException:
        pass

