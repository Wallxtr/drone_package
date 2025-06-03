#!/usr/bin/env python
# test result information for status msg --> msg_type, time,seq , drone_id , frame_id , position, num_detections  ,status_transmission_delay, processing_time
# test result information for waypoint msg --> msg_type, time, seq, drone_id, frame_id, waypoint_publisher_time
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
from datetime import datetime
import os
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
        
        # YOLO model params
        self.model_name = rospy.get_param('~model_name', 'yolov8n.pt')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)
        
        # Create log directory
        log_dir = os.path.expanduser("~/logs/subscriber_main_machine")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.status_log_path = os.path.join(log_dir, f"status_log_{timestamp}.txt")
        self.waypoint_log_path = os.path.join(log_dir, f"waypoint_log_{timestamp}.txt")

        self.status_log_file = open(self.status_log_path, 'a')
        self.waypoint_log_file = open(self.waypoint_log_path, 'a')

        #Time attributes
        self.processing_time = None
        self.waypoint_publisher_time = None
        self.tx_delay = None





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

        # test result information for status msg --> msg_type, time,seq , drone_id , frame_id , position, num_detections  ,status_transmission_delay, processing_time
        pos_str = "({:.1f},{:.1f},{:.1f})".format(msg.position.x,msg.position.y,msg.position.z)
        status_msg = "{},{},{},{},{},{},{},{:.1f},{:.1f}".format("status", msg.header.stamp, msg.header.seq, msg.drone_id, msg.header.frame_id, pos_str, number_detections,self.tx_delay,self.processing_time)
        # Save status message to file
        self.status_log_file.write(status_msg + "\n")
        self.status_log_file.flush()

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

        # test result information for waypoint msg --> msg_type, time, seq, drone_id, frame_id, waypoint_publisher_time
        waypoint_msg = "{},{},{},{},{},{:.1f}".format("waypoint",wp_msg.header.stamp , wp_msg.header.seq , wp_msg.drone_id , wp_msg.header.frame_id , self.waypoint_publisher_time )
        # Save waypoint message to file
        self.waypoint_log_file.write(waypoint_msg + "\n")
        self.waypoint_log_file.flush()
        rospy.loginfo("Published WayPoint for drone %d with label: %s", wp_msg.drone_id, wp_msg.action)

    def spin(self):
        rospy.spin()
    
    def __del__(self):
        if hasattr(self, 'status_log_file'):
            self.status_log_file.close()
        if hasattr(self, 'waypoint_log_file'):
            self.waypoint_log_file.close()

if __name__ == '__main__':
    try:
        node = SubscriberMainMachine()
        node.spin()
    except rospy.ROSInterruptException:
        pass

