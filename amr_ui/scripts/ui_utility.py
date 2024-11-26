import cv2
from PyQt5.QtGui import *
import numpy as np

import yaml
import math

import rospy
from geometry_msgs.msg import Pose, PoseArray


def covert_cv2qt_rgba(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2RGBA)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGBA8888)
    
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01

def convert_cv2qt(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGB888)
    
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01

def read_yaml(yaml_path):
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except FileNotFoundError:
        print(f"Error: The file {yaml_path} was not found.")
        return None
    except yaml.YAMLError as exc:
        print(f"Error: An issue occurred while parsing the YAML file: {exc}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

def save_waypoints(file_path, waypoints):
    with open(file_path, 'w') as file:
        for waypoint in waypoints:
            file.write(f"{waypoint[0]},{waypoint[1]}\n")

def load_waypoints_from_txt(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))
            waypoints.append((x, y))
    return waypoints

def convert_waypoint_ros(waypoints):
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "map"

    for i in range(len(waypoints)):
        # current waypoint
        x1, y1 = waypoints[i]
        # next waypoint (last waypoint -> wrap-around)(마지막의 경우 첫 번째로 wrap-around)
        x2, y2 = waypoints[(i + 1) % len(waypoints)]
        
        # orientation(yaw angle)
        dx = x2 - x1
        dy = y2 - y1
        yaw = math.atan2(dy, dx)
        
        # yaw to quaternion
        quaternion = yaw_to_quaternion(yaw)
        
        # Pose
        pose = Pose()
        pose.position.x = x1
        pose.position.y = y1
        pose.position.z = 0.0
        
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        # add to PoseArray
        pose_array.poses.append(pose)

    return pose_array

def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (qx, qy, qz, qw)