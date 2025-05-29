#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import os
import cv2
import torch
import time
from cv_bridge import CvBridge
from drone_pkg.msg import DroneStatusMainMachine

class DynamicDroneYoloSubscriber:
    def __init__(self):
        rospy.init_node('dynamic_drone_yolo_subscriber', anonymous=True)

        # where to save annotated images
        self.save_dir = rospy.get_param('~save_dir', 'output_images')
        os.makedirs(self.save_dir, exist_ok=True)

        # YOLO model params
        self.model_name = rospy.get_param('~model_name', 'yolov5s')
        self.model_path = rospy.get_param('~model_path', None)
        self.conf_thres = rospy.get_param('~conf_thres', 0.5)

        # load model
        if self.model_path:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom',
                                       path=self.model_path, force_reload=True)
        else:
            self.model = torch.hub.load('ultralytics/yolov5',
                                       self.model_name, pretrained=True)
        self.model.conf = self.conf_thres

        # bridge + bookkeeping
        self.bridge   = CvBridge()
        self.subs     = {}    
        self.counters = {}    

        # metrics bookkeeping
        self.last_arrival = {}           # topic -> timestamp of last frame


        # timer to scan for new DroneStatus topics
        scan_hz = rospy.get_param('~scan_rate', 1.0)
        self.timer = rospy.Timer(rospy.Duration(1.0/scan_hz),
                                 self._scan_for_topics)

        rospy.loginfo("DynamicDroneYoloSubscriber initialized; scanning for /drone/status/*")


    def _scan_for_topics(self, event):
        try:
            for topic, ttype in rospy.get_published_topics():
                if topic.startswith('/drone/status/') and ttype == 'drone_pkg/DroneStatusMainMachine':
                    if topic not in self.subs:
                        rospy.loginfo("Subscribing to {}".format(topic))
                        sub = rospy.Subscriber(topic, DroneStatusMainMachine,
                                               self._callback,
                                               callback_args=topic)
                        self.subs[topic]     = sub
                        self.counters[topic] = 0
        except Exception as e:
            rospy.logerr("Scan error: {}".format(e))

    def _callback(self, msg, topic):
        # capture arrival time
        recv_time = time.time()
        # transmission delay (ms)
        tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000
        # video rate (fps)
        if topic in self.last_arrival:
            dt = recv_time - self.last_arrival[topic]
            fps = 1.0 / dt if dt > 0 else float('inf')
            rospy.loginfo("[{}] Video rate: {:.2f} fps".format(topic,fps))
        self.last_arrival[topic] = recv_time

        # start processing timer
        proc_start = time.time()

        # 1) log header, drone_id, position
        hdr = msg.header
        pos = msg.position
        rospy.loginfo(
            "[%s] seq=%d stamp=%.3f frame_id=%s | drone_id=%d | pos=(%.2f, %.2f, %.2f)",
            topic,
            hdr.seq,
            hdr.stamp.to_sec(),
            hdr.frame_id,
            msg.drone_id,
            pos.x, pos.y, pos.z
        )

        # 2) convert ROS Image to CV2
        try:
            img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("[{}] CV convert error: {}".format(topic,e))
            return

        # 3) run YOLO and draw boxes
        try:
            results = self.model(img[:, :, ::-1])
            df = results.pandas().xyxy[0]
            for _, row in df.iterrows():
                x1, y1 = int(row.xmin), int(row.ymin)
                x2, y2 = int(row.xmax), int(row.ymax)
                conf   = row.confidence
                label  = row.name
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(img, "{} {:.2f}".format(label,conf),
                            (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0,255,0), 2)
        except Exception as e:
            rospy.logerr("[{}] Detection error: {}".format(topic,e))

        # end processing timer
        proc_end = time.time()
        processing_time = (proc_end - proc_start) * 1000
        rospy.loginfo("[{}] Processing time: {:.1f} ms; Transmission delay: {:.1f} ms".format(topic,processing_time,tx_delay))

        # 4) save annotated image
        idx   = self.counters[topic]
        name  = topic.strip('/').replace('/', '_') + "_det_{:04d}.png".format(idx)
        path  = os.path.join(self.save_dir, name)
        cv2.imwrite(path, img)
        rospy.loginfo("[{}] Saved annotated image -> {}".format(topic,path))
        self.counters[topic] += 1



    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DynamicDroneYoloSubscriber()
        node.spin()
    except rospy.ROSInterruptException:
        pass

