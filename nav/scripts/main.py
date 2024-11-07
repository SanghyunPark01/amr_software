#!/usr/bin/env python3
import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped, PoseStamped, Pose
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time

from enum import IntEnum

class Mode(IntEnum):
    START = 1
    REPEAT = 2
    STOP = 3
    PAUSE = 4

# change Pose to the correct frame 
def transformPose(pose,target_frame):
    #rospy.loginfo("Transforming pose to {} frame".format(target_frame))
    if pose.header.frame_id == target_frame:
        # already in correct frame
        return pose
    if not hasattr(transformPose, 'listener'):
        transformPose.listener = tf.TransformListener()
    try:
        transformPose.listener.waitForTransform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = transformPose.listener.transformPose(target_frame, pose)
        ret = PoseStamped()
        ret.header.frame_id = target_frame
        ret.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()

def transformWaypoints(waypoints, target_frame):
    for i in range(len(waypoints)):
        waypoints[i] = transformPose(waypoints[i], target_frame)
    return waypoints

def convert_PoseWithCovStamped_to_PoseStamped(pose):
    """Converts PoseWithCovarianceStamped to PoseStamped"""
    #rospy.loginfo('Converting PoseWithCovarianceStamped to PoseStamped')
    #rospy.loginfo('PoseWithCovarianceStamped: %s' % pose)
    pose_stamped = PoseStamped()
    pose_stamped.header = pose.header
    pose_stamped.pose = pose.pose.pose
    #rospy.loginfo('PoseStamped: %s' % pose_stamped)
    return pose_stamped

def convert_PoseStampedArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose for pose in waypoints]
    rospy.loginfo('Converted PoseStampedArray to PoseArray: %s' % poses)
    return poses

class FollowPath(State):
    def __init__(self):
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','camera_init')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link')
        self.duration = rospy.get_param('~wait_duration', 0.0)

        self.waypoints_topic = rospy.get_param('~waypoints_topic','/waypoints')
        self.addpose_topic = rospy.get_param('~addpose_topic','/add_pose')
        self.addpose_rviz_topic = rospy.get_param('~addpose_rviz_topic','/initialpose')
        self.operating_mode_topic = rospy.get_param('~operating_mode_topic','/operating_mode')

        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoint_poses')

        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        self.waypoints = []
        self.waypoint_idx = 0
        self.operating_mode = Mode.STOP.value

        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()

        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.1)
        rospy.loginfo('Distance tolerance: %s' % self.distance_tolerance)
        rospy.loginfo('Distance tolerance type: %s' % type(self.distance_tolerance))
       
        rospy.Subscriber(self.waypoints_topic, PoseArray, self.waypoint_callback)
        rospy.Subscriber(self.addpose_topic, PoseStamped, self.addpose_callback)
        rospy.Subscriber(self.addpose_rviz_topic, PoseWithCovarianceStamped, self.add_pose_from_rviz_callback)
        rospy.Subscriber(self.operating_mode_topic, Int32, self.operation_mode_callback)
        rospy.loginfo('Subscribed to topics')
        self.initialize_path_queue()
        rospy.loginfo('Waiting for new waypoints...')

        exec_thread = threading.Thread(target=self.execute)
        exec_thread.start()

        rospy.loginfo('################ FollowPath initialized ###############')

        rospy.spin()

    def execute(self):
        rospy.loginfo("Executing follow_path")
        while not rospy.is_shutdown():
            if self.operating_mode == Mode.START.value or self.operating_mode == Mode.REPEAT.value:
                rospy.loginfo('Starting to follow path')
                self.start_following_path()
            elif self.operating_mode == Mode.STOP.value:
                rospy.sleep(1)
            elif self.operating_mode == Mode.PAUSE.value:
                rospy.sleep(1)
            else:
                rospy.loginfo('Invalid operation mode')
                rospy.sleep(1)

    def start_following_path(self):
        while(True):
            if self.operating_mode == Mode.STOP.value:
                return
            if self.operating_mode == Mode.PAUSE.value:
                rospy.sleep(1)
                continue

            if self.waypoints == []:
                rospy.loginfo('The waypoint queue is empty, Waiting for new waypoints...')
                rospy.sleep(1)
                continue
            rospy.loginfo('Following the waypoints')
            rospy.loginfo('waypoint_idx: %s' % self.waypoint_idx)
            rospy.loginfo('waypoint: %s' % self.waypoints[self.waypoint_idx])
            waypoint = self.waypoints[self.waypoint_idx]
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.position.x, waypoint.pose.position.y))
            
            self.client.send_goal(goal)
            #This is the loop which exist when the robot is near a certain GOAL point.
            distance = 10
            while(distance > self.distance_tolerance):
                if( self.operating_mode == Mode.STOP.value or self.operating_mode == Mode.PAUSE.value ):
                    return
                now = rospy.Time.now()
                self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                distance = math.sqrt(pow(waypoint.pose.position.x-trans[0],2)+pow(waypoint.pose.position.y-trans[1],2))

            self.waypoint_idx += 1
            if self.waypoint_idx == len(self.waypoints):
                if(self.operating_mode == Mode.REPEAT.value):
                    self.waypoint_idx = self.waypoint_idx % len(self.waypoints)
                else:
                    rospy.loginfo('All waypoints are finished')
                    self.initialize_path_queue()
                    return

    def stop_following_path(self):
        self.initialize_path_queue()
        self.client.cancel_goal()
        
    def initialize_path_queue(self):
        self.waypoints = [] # the waypoint queue
        rospy.loginfo('Waypoint queue is initialized')
        self.waypoint_idx = 0  # the index of the waypoint under execution
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseStampedArray_to_PoseArray(self.waypoints))
    
    def waypoint_callback(self, msg):
        self.waypoints = msg.poses #array of geometry_msgs/PoseStamped, header, pose
        self.waypoint_idx = 0
        rospy.loginfo('Waypoint queue updated')
        print(self.waypoints)
        # Publish as PoseArray so that you can see them in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseStampedArray_to_PoseArray(self.waypoints))

    def add_pose_from_rviz_callback(self, msg):
        rospy.loginfo('Adding new pose from rviz')
        pose = convert_PoseWithCovStamped_to_PoseStamped(msg)
        self.waypoints.append(transformPose(pose, "map"))
        self.poseArray_publisher.publish(convert_PoseStampedArray_to_PoseArray(self.waypoints))
        #rospy.loginfo(pose.pose.position)
                              

    def addpose_callback(self, msg):
        rospy.loginfo('Adding new pose')
        #add PoseStamped msg to the waypoints queue
        self.waypoints.append(msg)
        # Publish as PoseArray so that you can see them in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseStampedArray_to_PoseArray(self.waypoints))

    def operation_mode_callback(self, msg):
        self.operating_mode = msg.data
        rospy.loginfo('Operating mode updated to %s' % self.operating_mode)
        if(self.operating_mode == Mode.START.value):
            rospy.loginfo('##### START Follow Path #####')
        elif(self.operating_mode == Mode.REPEAT.value):
            rospy.loginfo('##### REPEAT Follow Path #####')
        elif(self.operating_mode == Mode.STOP.value):
            rospy.loginfo('##### STOP Follow Path #####')
            self.stop_following_path()
        elif(self.operating_mode == Mode.PAUSE.value):
            rospy.loginfo('##### PAUSE Follow Path #####')
            self.client.cancel_goal()
        else:
            rospy.loginfo('##### INVALID OPERATION MODE #####')
        
def main():
    rospy.init_node('follow_waypoints')
    print("Executing follow_waypoints")
    follow_path = FollowPath()
    print("Finished follow_waypoints")


if __name__ == "__main__":
    main()
