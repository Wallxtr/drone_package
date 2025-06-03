#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import time
import random
from cv_bridge import CvBridge
from drone_package.msg import DroneStatusMainMachine, WayPoint
from geometry_msgs.msg import Point
from ultralytics import YOLO


class SubscriberMainMachine:
    def __init__(self):
        rospy.init_node('subscriber_main_machine', anonymous=True)

        #Time attributes
        self.processing_time = None
        self.waypoint_publisher_time = None
        self.tx_delay = None


        # YOLO model params
        self.model_name = rospy.get_param('~model_name', 'yolov8n.pt')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)


        # Load model
        self.model = YOLO(self.model_path if self.model_path else self.model_name)

        self.model.conf = self.conf_thres

        # bridge initialize
        self.bridge   = CvBridge()


        # metrics bookkeeping
        self.last_arrival = {}           # topic -> timestamp of last frame

        # Set up publisher for WayPoint.msg
        self.pub = rospy.Publisher("/drone_waypoint_main_machine/waypoint", WayPoint, queue_size=10)

        sub = rospy.Subscriber("/drone_status_main_machine/status", DroneStatusMainMachine,self._callback)

        rospy.loginfo("SubscriberMainMachine initialized; scanning for /drone_status_main_machine/status")

        



    def _callback(self, msg):
        # capture arrival time
        recv_time = time.time()
        # transmission delay (ms)
        self.tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000


        # start processing timer
        proc_start = time.time()
        # 1) log header, drone_id, position
        hdr = msg.header
        pos = msg.position
        # 2) convert ROS Image to CV2
        try:
            img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(" CV convert error: {}".format(e))
            return
        # 3) run YOLO
        try:
            results = self.model(img[:, :, ::-1],verbose=False)
            df = results[0].boxes
            number_detections = len(df)
        except Exception as e:
            rospy.logerr(" Detection error: {}".format(e))
        rospy.loginfo(
            " seq=%d stamp=%.3f frame_id=%s | drone_id=%d | pos=(%.2f, %.2f, %.2f) | detection=%d",
            hdr.seq,
            hdr.stamp.to_sec(),
            hdr.frame_id,
            msg.drone_id,
            pos.x, pos.y, pos.z,
            number_detections
        )
        # end processing timer
        proc_end = time.time()
        self.processing_time = (proc_end - proc_start) * 1000
        rospy.loginfo(" Processing time: {:.1f} ms; Transmission delay: {:.1f} ms".format(self.processing_time,self.tx_delay))


        
        # Publish WayPoint.msg
        waypoint_publisher_time_start = time.time()
        wp_msg = WayPoint()
        wp_msg.header.stamp = rospy.Time.now()
        wp_msg.header.frame_id = msg.header.frame_id
        wp_msg.header.seq = msg.header.seq
        
        wp_msg.drone_id = msg.drone_id
        if (number_detections > 5):
            wp_msg.action = "hover"
            wp_msg.position = msg.position
        elif (number_detections > 0):
            wp_msg.action = "wander"
            wp_msg.position = Point(msg.position.x + random.uniform(-1,1),msg.position.y + random.uniform(-1,1),msg.position.z)
        else:
            wp_msg.action = "go"
            wp_msg.position = Point(x=random.uniform(-10,10),y=random.uniform(-10,10),z=random.uniform(0,5))
        self.pub.publish(wp_msg)


        waypoint_publisher_time_end = time.time()
        self.waypoint_publisher_time = (waypoint_publisher_time_end - waypoint_publisher_time_start) * 1000
        
        rospy.loginfo("Published WayPoint for drone %d with label: %s", wp_msg.drone_id, wp_msg.action)

        

        




    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SubscriberMainMachine()
        node.spin()
    except rospy.ROSInterruptException:
        pass

