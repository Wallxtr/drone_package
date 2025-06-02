#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import time
from cv_bridge import CvBridge
from drone_package.msg import DroneStatusMainMachine
from ultralytics import YOLO


class SubscriberMainMachine:
    def __init__(self):
        rospy.init_node('subscriber_main_machine', anonymous=True)


        # YOLO model params
        self.model_name = rospy.get_param('~model_name', 'yolov5s')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)


        # Load model
        self.model = YOLO(self.model_path if self.model_path else self.model_name)

        self.model.conf = self.conf_thres

        # bridge initialize
        self.bridge   = CvBridge()


        # metrics bookkeeping
        self.last_arrival = {}           # topic -> timestamp of last frame

        sub = rospy.Subscriber("/drone_status_main_machine/status", DroneStatusMainMachine,self._callback)

        rospy.loginfo("SubscriberMainMachine initialized; scanning for /drone_status_main_machine/status")




    def _callback(self, msg):
        # capture arrival time
        recv_time = time.time()
        # transmission delay (ms)
        tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000
        # video rate (fps)
        """
        
        if topic in self.last_arrival:
            dt = recv_time - self.last_arrival[topic]
            fps = 1.0 / dt if dt > 0 else float('inf')
            rospy.loginfo("[{}] Video rate: {:.2f} fps".format(topic,fps))
        self.last_arrival[topic] = recv_time
        """
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
        processing_time = (proc_end - proc_start) * 1000
        rospy.loginfo(" Processing time: {:.1f} ms; Transmission delay: {:.1f} ms".format(processing_time,tx_delay))




    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SubscriberMainMachine()
        node.spin()
    except rospy.ROSInterruptException:
        pass

