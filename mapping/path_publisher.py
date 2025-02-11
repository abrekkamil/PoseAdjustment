#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class path_publisher_node:
    def __init__(self):
        rospy.init_node("custom_mapping_node")
        self.path_pub = rospy.Publisher("/robot_path", Path, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path = Path()
        self.path.header.frame_id = "map"

    def odom_callback(self, odom_msg):
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        self.path.poses.append(pose)
        orientation = odom_msg.pose.pose.orientation


   
        self.path_pub.publish(self.path)

if __name__ == "__main__":
    path_publisher_node()
    rospy.spin()
