#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
import time
from drone_package.msg import DroneStatusDroneMachine

class SubscriberDroneMachine:
    def __init__(self):
        rospy.init_node('subscriber_drone_machine')
        self.last_status = {}   # drone_id -> last receipt time

        topic = "/drone_status_drone_machine/status"
        sub = rospy.Subscriber(topic, DroneStatusDroneMachine, self.status_cb,)



    def status_cb(self, msg):
        drone_id = msg.drone_id
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

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SubscriberDroneMachine()
        node.spin()
    except rospy.ROSInterruptException:
        pass
