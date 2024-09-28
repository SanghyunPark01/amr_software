#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2

# ros
import rospy
from std_msgs.msg import *
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
q_UI_form = uic.loadUiType(ui_path)[0]
# ==========================================================================

class UI(QMainWindow,q_UI_form):
    def __init__(self):
        # ui init
        super().__init__()
        self.setupUi(self)
        self.ImageView_2d_gridmap = pg.ImageView(view=pg.PlotItem())
        self.ImageView_2d_gridmap.ui.histogram.hide()
        self.ImageView_2d_gridmap.ui.roiBtn.hide()
        self.ImageView_2d_gridmap.ui.menuBtn.hide()
        self.ImageView_2d_gridmap.getView().hideAxis('left')
        self.ImageView_2d_gridmap.getView().hideAxis('bottom')
        self.verticalLayout_vis_gridmap.addWidget(self.ImageView_2d_gridmap)

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
        rospy.init_node('amr_ui', anonymous = True)
        self._m_sub_conversion_status = rospy.Subscriber("/ui/map_converter_status/5089e6a42f124640607c98bd9cb4c890", Float32, self.callback_conversion_status, queue_size=1)
        
        # init class
        self._m_map_converter = MapConverter()
        self._m_waypoint_decision = WaypointDecision()

        # init ui function
        self.pushButton_close.clicked.connect(self.shutdown_sys)
            # map converter
        self.pushButton_gridmap_setting.clicked.connect(self.set_map_converter)
        self.pushButton_gridmap_convert.clicked.connect(self.convert_3d_to_2d)
        self.pushButton_waypoint_setting.clicked.connect(self.set_waypoint)

        # var
            # map converter
        self._m_curr_z_value = 0
        self._m_grid_map_path = None
        self._m_conversion_stop_enabled = False

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
            self._m_grid_map_path = self.map_data_dir + "/grid_map_" + file_name + ".pgm"
            
            self.lineEdit_save_dir_bag.setText(self._m_grid_map_path)
            self.set_2d_grid_img(self._m_grid_map_path)

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ UI Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def shutdown_sys(self):
        QCoreApplication.instance().quit()

    def set_map_converter(self):
        self._m_map_converter.show()
    def convert_3d_to_2d(self):
        self._m_curr_z_value = self._m_map_converter.getValueZ()
        if self._m_conversion_stop_enabled:
            self._m_map_converter.stop()
            print("stop!!")
            self.pushButton_gridmap_setting.setEnabled(True)
            self.pushButton_gridmap_convert.setText("Convert")
            self._m_conversion_stop_enabled = False
            return
        if self._m_map_converter.convert():
            self.pushButton_gridmap_setting.setEnabled(False)
            self.pushButton_gridmap_convert.setText("Stop")
            self._m_conversion_stop_enabled = True

    def set_waypoint(self):
        self._m_waypoint_decision.show()

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def set_2d_grid_img(self , grid_map_path):
        gridmap_img = cv2.imread(grid_map_path, cv2.IMREAD_UNCHANGED) # (height, width)
        gridmap_img = gridmap_img.T # (height, width)

        # set image
        self.ImageView_2d_gridmap.setImage(gridmap_img)
        # set view - zoom
        width, height = gridmap_img.shape
        self.ImageView_2d_gridmap.getView().setRange(xRange=[0 + width/2, width/4 + width/2], yRange=[0 + height/2, height/4 + height/2], padding=0)

# ==========================================================================
if __name__ == '__main__':
    try:
        qApp = QApplication(sys.argv)
        qt_ui = UI()
        qt_ui.show()
        qApp.exec_()
    
    except:
        pass