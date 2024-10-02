import cv2
from PyQt5.QtGui import *
import numpy as np
import yaml

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