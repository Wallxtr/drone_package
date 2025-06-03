#!/usr/bin/env python
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import random
import rospy
import time
from geometry_msgs.msg import Point
from drone_package.msg import DroneStatusDroneMachine,WayPoint

class SubscriberDroneMachine:
    def __init__(self):
        rospy.init_node('subscriber_drone_machine')

        #Time attributes
        self.waypoint_publisher_time = None
        self.tx_delay = None

        self.pub = rospy.Publisher("/drone_waypoint_drone_machine/waypoint", WayPoint, queue_size=10)
        topic = "/drone_status_drone_machine/status"
        sub = rospy.Subscriber(topic, DroneStatusDroneMachine, self.status_cb,)



    def status_cb(self, msg):
        drone_id = msg.drone_id
        recv_time = time.time()
        # transmission delay ms
        self.tx_delay = (recv_time - msg.header.stamp.to_sec()) * 1000

        pos = msg.position
        rospy.loginfo("[%s] pos=(%.1f,%.1f,%.1f) detections=%d" % (
            drone_id,
            pos.x, pos.y, pos.z,
            msg.detections
        ))

        # Publish WayPoint.msg
        waypoint_publisher_time_start = time.time()
        wp_msg = WayPoint()
        wp_msg.header.stamp = rospy.Time.now()
        wp_msg.header.frame_id = msg.header.frame_id
        wp_msg.header.seq = msg.header.seq

        wp_msg.drone_id = msg.drone_id
        if (msg.detections > 5):
            wp_msg.action = "hover"
            wp_msg.position = msg.position
        elif (msg.detections > 0):
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
        node = SubscriberDroneMachine()
        node.spin()
    except rospy.ROSInterruptException:
        pass
