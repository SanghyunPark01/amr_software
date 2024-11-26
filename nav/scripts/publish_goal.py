#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse

import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('x_', type=float)
    parser.add_argument('y_', type=float)
    parser.add_argument('z_', type=float)
    parser.add_argument('w_', type=float)
    args = parser.parse_args()

    rospy.init_node('publish_goal')
    pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # 转换为pose
    quat = [args.x_, args.y_, args.z_, args.w_]
    xyz = [args.x, args.y, args.z]

    goal_pose = PoseStamped()
    goal_pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    goal_pose.header.stamp = rospy.Time().now()
    goal_pose.header.frame_id = 'map'
    rospy.sleep(1)
    rospy.loginfo('goal Pose: {} {} {} {} {} {} {}'.format(
        args.x, args.y, args.z, args.x_, args.y_, args.z_, args.w_))
    pub_pose.publish(goal_pose)
