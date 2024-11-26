#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
import time
import yaml
import subprocess

# ros
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import PoseArray
from amr_ui.srv import *

# QT
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *
import pyqtgraph as pg

path__ = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path__)

# Custom
import ui_utility
from map_converter import MapConverter
from waypoint_decision import WaypointDecision

# ==========================================================================
dir_ui_path__ = path__ + "/../ui/"
ui_path = dir_ui_path__ + "amr_ui.ui"
logo_path = dir_ui_path__ + "POSTECH.png"
shell_path = path__ + "/../shell_script"
q_UI_form = uic.loadUiType(ui_path)[0]
# ==========================================================================

class UI(QMainWindow,q_UI_form):
    def __init__(self):
        # ui init
        super().__init__()
        self.setupUi(self)
        self._m_home_path = os.path.expanduser('~')
        #
        self.ImageView_2d_gridmap = pg.ImageView(view=pg.PlotItem())
        self.ImageView_2d_gridmap.ui.histogram.hide()
        self.ImageView_2d_gridmap.ui.roiBtn.hide()
        self.ImageView_2d_gridmap.ui.menuBtn.hide()
        self.ImageView_2d_gridmap.getView().hideAxis('left')
        self.ImageView_2d_gridmap.getView().hideAxis('bottom')
        self.verticalLayout_vis_gridmap.addWidget(self.ImageView_2d_gridmap)
        #
        self.ImageView_waypoint = pg.ImageView(view=pg.PlotItem())
        self.ImageView_waypoint.ui.histogram.hide()
        self.ImageView_waypoint.ui.roiBtn.hide()
        self.ImageView_waypoint.ui.menuBtn.hide()
        self.ImageView_waypoint.getView().hideAxis('left')
        self.ImageView_waypoint.getView().hideAxis('bottom')
        self.verticalLayout_vis_waypoint.addWidget(self.ImageView_waypoint)


        # map data dir
        self.map_data_dir = path__ + "/../map_data"
        if not os.path.exists(self.map_data_dir):
            os.makedirs(self.map_data_dir)
        
        # logo setting
        logo = cv2.imread(logo_path, cv2.IMREAD_UNCHANGED)
        logo = cv2.resize(logo, dsize = (0,0), fx = 0.91, fy = 0.91)
        logo_pix_map = ui_utility.covert_cv2qt_rgba(logo)
        self.label_logo_postech.setPixmap(logo_pix_map)

        # init ros
            # param
        self.GRID_RESOLUTION = rospy.get_param("/map_converter/handler/octree_resolution")
            # node
        rospy.init_node('amr_ui', anonymous = True)
        self._m_sub_conversion_status = rospy.Subscriber("/ui/map_converter_status/5089e6a42f124640607c98bd9cb4c890", Float32, self.callback_conversion_status, queue_size=1)
        self._m_pub_gridmap = rospy.Publisher("ui/gridmap", String, queue_size = 1)
            # for nav
                # TODO: ros waypoint publisher
        self._m_pub_waypoint = rospy.Publisher("/waypoints", PoseArray, queue_size= 10)
                # mode
        self._m_pub_nav_execute = rospy.Publisher("/operation_mode", Int32, queue_size = 10)

        # init class
        self._m_map_converter = MapConverter()
        self._m_waypoint_decision = WaypointDecision()

        # init ui function
        self.pushButton_close.clicked.connect(self.shutdown_sys)
            # map converter
        self.pushButton_gridmap_setting.clicked.connect(self.set_map_converter)
        self.pushButton_gridmap_convert.clicked.connect(self.convert_3d_to_2d)
            # waypoint decision
        self.pushButton_waypoint_setting.clicked.connect(self.set_waypoint)
        self.pushButton_waypoint_start.clicked.connect(self.start_waypoint_decision)
            # Nav
        self.pushButton_gridmap_open.clicked.connect(self.open_gridmap)
        self.pushButton_waypoint_open.clicked.connect(self.open_waypont)
        self.pushButton_set_nav.clicked.connect(self.pub_map_waypoint)
        self.pushButton_start_nav.clicked.connect(self.pub_start_nav)
        self._m_pause_nav = True
        self.pushButton_stop_nav.clicked.connect(self.pub_stop_nav)
        
            # etc
        self.pushButton_slam.clicked.connect(self.run_slam)
        self.pushButton_localization.clicked.connect(self.run_localization)
        self.pushButton_navigation.clicked.connect(self.run_navigation)

        # var
            # map converter
        self._m_curr_z_value = 0
        self._m_grid_map_path = None
        self._m_conversion_stop_enabled = False
        self._m_girdmap_status = False
            # waypoint
        self._m_waypoint = []
        self._m_waypoint_ros = PoseArray()
        self._m_waypoint_path = ""
        self._m_waypoint_status = False
            # navigation
        self._m_gridmap_config = ""

    # drag and drop
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()
    def dropEvent(self, event):
        files = [u.toLocalFile() for u in event.mimeData().urls()]
        if len(files) > 1:
            return
        if len(files) == 1 and files[0][-4:] == ".pgm":
            self.set_gridmap_path(files[0])
        elif len(files) == 1 and files[0][-4:] == ".txt":
            self.load_waypoints(files[0])
        else: 
            return

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ ROS Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def callback_conversion_status(self, msg):
        self.progressBar_conversion_status.setValue(int(msg.data))
        if msg.data >= 100:
            self.pushButton_gridmap_setting.setEnabled(True)
            self.pushButton_gridmap_convert.setText("Convert")
            self._m_conversion_stop_enabled = False

            file_name = str(self._m_curr_z_value)
            file_name_list = list(file_name)
            for idx in range(len(file_name_list)):
                if file_name_list[idx] == '.':
                    file_name_list[idx] = '_'
            file_name = ''.join(file_name_list)
            grid_map_path = self.map_data_dir + "/grid_map_" + file_name + ".pgm"
            self.set_gridmap_path(grid_map_path)

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ UI Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def shutdown_sys(self):
        QCoreApplication.instance().quit()
        os.system("killall -9 rosmaster && killall -9 rosout")
    # map converter
    def set_map_converter(self):
        self._m_map_converter.show()
    def convert_3d_to_2d(self):
        self._m_curr_z_value = self._m_map_converter.getValueZ()
        if self._m_conversion_stop_enabled:
            self._m_map_converter.stop()
            self.pushButton_gridmap_setting.setEnabled(True)
            self.pushButton_gridmap_convert.setText("Convert")
            self._m_conversion_stop_enabled = False
            return
        if self._m_map_converter.convert():
            self.pushButton_gridmap_setting.setEnabled(False)
            self.pushButton_gridmap_convert.setText("Stop")
            self._m_conversion_stop_enabled = True
    def set_gridmap_path(self, girdmap_path):
        self._m_grid_map_path = girdmap_path
        self.lineEdit_gridmap_path.setText(self._m_grid_map_path)
        self.set_2d_grid_img(self._m_grid_map_path)

        self._m_gridmap_config = self._m_grid_map_path.replace('.pgm', '.yaml')
        self._m_girdmap_status = True

    # waypoint decision
    def set_waypoint(self):
        if self._m_grid_map_path is None:
            return
        if self._m_grid_map_path[-4:] == ".pgm":
            self._m_waypoint_decision.set_girdmap_path_to_waypoint_decision(self._m_grid_map_path)
            self._m_waypoint_decision.show()
    def start_waypoint_decision(self):
        if not self._m_waypoint_decision.is_ready():
            return
        # if, is ready
        self._m_waypoint.clear()
        self.gen_waypoint()

    # Nav
    def open_gridmap(self):
        file_name = QFileDialog.getOpenFileName(self, 'Open .pgm file', self._m_home_path, '.pgm(*.pgm)')
        if not file_name[0]:
            return
        if file_name[0][-4:] != ".pgm":
            return
        self.set_gridmap_path(file_name[0])
    def open_waypont(self):
        file_name = QFileDialog.getOpenFileName(self, 'Open .txt file', self._m_home_path, '.txt(*.txt)')
        if not file_name[0]:
            return
        if file_name[0][-4:] != ".txt":
            return
        self.load_waypoints(file_name[0])
    def pub_map_waypoint(self):
        if not self._m_girdmap_status or not self._m_waypoint_status:
            return
        self._m_pub_gridmap.publish(self._m_gridmap_config) # need to implement subscribe code in Nav Stack

        # TODO: waypoint publish
        # print(self._m_waypoint)
        self._m_waypoint_ros = ui_utility.convert_waypoint_ros(self._m_waypoint)
        # print(self._m_waypoint_ros)
        self._m_pub_waypoint.publish(self._m_waypoint_ros)


    def pub_start_nav(self):
        if self._m_pause_nav:
            # pub
            if self.radioButton_nav_mode.isChecked(): # repeat mode
                self._m_pub_nav_execute.publish(2)
            else: # start
                self._m_pub_nav_execute.publish(1)
            self._m_pause_nav = False
            self.pushButton_start_nav.setText("Pause")
            return
        else:
            # pub pause
            self._m_pub_nav_execute.publish(3)
            self._m_pause_nav = True
            self.pushButton_start_nav.setText("Start")
            return
    def pub_stop_nav(self):
        self._m_pub_nav_execute.publish(0)

    # etc
    def run_slam(self):
        slam_sh = shell_path + "/slam.sh"
        subprocess.call('sh ' + slam_sh, shell = True)
    def run_localization(self):
        slam_sh = shell_path + "/localization.sh"
        subprocess.call('sh ' + slam_sh, shell = True)
    def run_navigation(self):
        slam_sh = shell_path + "/navigation.sh"
        subprocess.call('sh ' + slam_sh, shell = True)

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def set_2d_grid_img(self , grid_map_path):
        gridmap_img = cv2.imread(grid_map_path, cv2.IMREAD_GRAYSCALE) # (height, width) #IMREAD_UNCHANGED IMREAD_GRAYSCALE
        gridmap_img = gridmap_img.T # (height, width)

        # set image
        self.ImageView_2d_gridmap.setImage(gridmap_img, levels = (0, 255))

        # set view - zoom
        width, height = gridmap_img.shape
        self.ImageView_2d_gridmap.getView().setRange(xRange=[0 + width/2, width/4 + width/2], yRange=[0 + height/2, height/4 + height/2], padding=0)

    def set_girdmap_color_img(self, img_color):
        self.ImageView_waypoint.setImage(np.transpose(img_color, (1, 0, 2)), levels = (0, 255))
        width, height, _ = img_color.shape
        self.ImageView_waypoint.getView().setRange(xRange=[0 + width/2, width/4 + width/2], yRange=[0 + height/2, height/4 + height/2], padding=0)
    
    def gen_waypoint(self):
        info__ = self._m_waypoint_decision.get_info() 
        wall_info = info__[0] # image coordinate
        interval_info = info__[1] # real scale
        right_side = self._m_waypoint_decision.get_orientation() # True: right side, False: left side
        
        # waypoint decision with image coordinate
        interval_info = [(x / self.GRID_RESOLUTION) for x in interval_info] # real to img
        interval_info.sort(reverse=False)

        x1, y1 = wall_info[0]
        x2, y2 = wall_info[1]
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx**2 + dy**2)

        perpendicular_dx = -dy / length
        perpendicular_dy = dx / length

        if not right_side:
            perpendicular_dx *= -1
            perpendicular_dy *= -1

        for idx, interval in enumerate(interval_info):
            new_start_point = (x1 + interval * perpendicular_dx, y1 + interval * perpendicular_dy)
            new_end_point = (x2 + interval * perpendicular_dx, y2 + interval * perpendicular_dy)
            points = [new_start_point, new_end_point] if idx % 2 == 0 else [new_end_point, new_start_point]
            self._m_waypoint.extend(points)

        gird_map_img = cv2.imread(self._m_grid_map_path, cv2.IMREAD_UNCHANGED) # (height, width)
        color_grid_map = cv2.cvtColor(gird_map_img, cv2.COLOR_GRAY2RGB)

        # for visualize - wall
        distance_wall = np.hypot(wall_info[0][0] - wall_info[1][0], wall_info[0][1] - wall_info[1][1])
        tip_length_wall = 20 / distance_wall if distance_wall != 0 else 0
        cv2.arrowedLine(color_grid_map, wall_info[0], wall_info[1], (255, 0, 0), 3, tipLength=tip_length_wall)

        # for visualize - waypoint
        prev_pt = None
        for pt in self._m_waypoint:
            curr_pt = tuple(map(int, pt))
            cv2.circle(color_grid_map, curr_pt, 5, (0, 255, 0), -1)
            if prev_pt is not None:
                distance = np.hypot(prev_pt[0] - curr_pt[0], prev_pt[1] - curr_pt[1])
                tip_length = 20 / distance if distance != 0 else 0
                cv2.arrowedLine(color_grid_map, prev_pt, curr_pt, (0, 0, 255), 3, tipLength=tip_length)
            prev_pt = curr_pt
        curr_pt = tuple(map(int, self._m_waypoint[0]))
        distance = np.hypot(prev_pt[0] - curr_pt[0], prev_pt[1] - curr_pt[1])
        tip_length = 20 / distance if distance != 0 else 0
        cv2.arrowedLine(color_grid_map, prev_pt, curr_pt, (0, 0, 255), 3, tipLength=tip_length)


        self.set_girdmap_color_img(color_grid_map)

        # image coordinate to world coordinate(real scale)
        self._m_gridmap_config = self._m_grid_map_path.replace('.pgm', '.yaml')
        map_setting = ui_utility.read_yaml(self._m_gridmap_config)
        origin_x = map_setting['origin'][0]
        origin_y = map_setting['origin'][1]
        self._m_waypoint = [(pt[0] * self.GRID_RESOLUTION + origin_x,(gird_map_img.shape[0] - pt[1]) * self.GRID_RESOLUTION + origin_y)for pt in self._m_waypoint]

        # save waypoint
        file_name = "waypoints.txt"
        waypoint_path = self.map_data_dir + "/waypoint/" + file_name

        ui_utility.save_waypoints(waypoint_path, self._m_waypoint)
        self._m_waypoint_path = waypoint_path
        self.lineEdit_waypoint_path.setText(self._m_waypoint_path)
        self._m_waypoint_status = True
    
    def load_waypoints(self, path):
        self._m_waypoint_path = path
        self.lineEdit_waypoint_path.setText(self._m_waypoint_path)
        self._m_waypoint = ui_utility.load_waypoints_from_txt(self._m_waypoint_path)
        self._m_waypoint_status = True


# ==========================================================================
if __name__ == '__main__':
    try:
        qApp = QApplication(sys.argv)
        qt_ui = UI()
        qt_ui.show()
        qApp.exec_()
    
    except:
        pass