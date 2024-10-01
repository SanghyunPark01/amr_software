import sys
import os
import cv2
import numpy as np
import math
import copy

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication, Qt, QSize
from PyQt5.QtGui import *
import pyqtgraph as pg

import rospy


path__ = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path__)

# Custom
import ui_utility

# ==========================================================================
path__ = path__ + "/../ui/"
ui_path = path__ + "waypoint_decision_ui.ui"
arrow_path = path__+ "img/"
q_UI_form = uic.loadUiType(ui_path)[0]
# ==========================================================================


class WaypointDecision(QMainWindow,q_UI_form):
    def __init__(self):
        # ui init
        super().__init__()
        self.setupUi(self)
        # set arrow image
        arrow_path_right = arrow_path + "arrow_right.png"
        arrow_path_left = arrow_path + "arrow_left.png"
        arrow_path_up = arrow_path + "arrow_up.png"
        arrow_path_down = arrow_path + "arrow_down.png"
        arrow_path_clockwise = arrow_path + "arrow_clockwise.png"
        arrow_path_anticlockwise = arrow_path + "arrow_anticlockwise.png"
        self.pushButton_arrow_right.setIcon(QIcon(arrow_path_right))
        self.pushButton_arrow_right.setIconSize(QSize(self.pushButton_arrow_right.size().width(), self.pushButton_arrow_right.size().height()))
        self.pushButton_arrow_left.setIcon(QIcon(arrow_path_left))
        self.pushButton_arrow_left.setIconSize(QSize(self.pushButton_arrow_left.size().width(), self.pushButton_arrow_left.size().height()))
        self.pushButton_arrow_up.setIcon(QIcon(arrow_path_up))
        self.pushButton_arrow_up.setIconSize(QSize(self.pushButton_arrow_up.size().width(), self.pushButton_arrow_up.size().height()))
        self.pushButton_arrow_down.setIcon(QIcon(arrow_path_down))
        self.pushButton_arrow_down.setIconSize(QSize(self.pushButton_arrow_down.size().width(), self.pushButton_arrow_down.size().height()))
        self.pushButton_clockwise.setIcon(QIcon(arrow_path_clockwise))
        self.pushButton_clockwise.setIconSize(QSize(self.pushButton_clockwise.size().width(), self.pushButton_clockwise.size().height()))
        self.pushButton_anticlockwise.setIcon(QIcon(arrow_path_anticlockwise))
        self.pushButton_anticlockwise.setIconSize(QSize(self.pushButton_anticlockwise.size().width(), self.pushButton_anticlockwise.size().height()))
        # set gridmap view
        self.ImageView_2d_gridmap = pg.ImageView(view=pg.PlotItem())
        self.ImageView_2d_gridmap.ui.histogram.hide()
        self.ImageView_2d_gridmap.ui.roiBtn.hide()
        self.ImageView_2d_gridmap.ui.menuBtn.hide()
        self.ImageView_2d_gridmap.getView().hideAxis('left')
        self.ImageView_2d_gridmap.getView().hideAxis('bottom')
        self.verticalLayout_vis_gridmap.addWidget(self.ImageView_2d_gridmap)

        # init ui function
        self.pushButton_close.clicked.connect(self.shutdown_sys)
        self.pushButton_wall_line_automatic_init.clicked.connect(self.init_wall_automatic)
            # for interval value
        self.doubleSpinBox_interval_value.valueChanged.connect(self.change_interval_value_of_spinbox)
        self.horizontalSlider_interval.valueChanged.connect(self.change_interval_value_of_slider)
        self.pushButton_up_interval.clicked.connect(self.up_interval_value)
        self.pushButton_down_interval.clicked.connect(self.down_interval_value)
        self.pushButton_add_interval.clicked.connect(self.add_interval_value)
        self.pushButton_clear_interval.clicked.connect(self.clear_interval_value)
            # for wall line length
        self.horizontalSlider_wall_length.valueChanged.connect(self.change_wall_length_of_slider)
        self.doubleSpinBox_line_length.valueChanged.connect(self.change_wall_length_of_spinbox)
        self.pushButton_expand_line.clicked.connect(self.expand_wall_length)
        self.pushButton_reduce_line.clicked.connect(self.reduce_wall_length)
            # for wall angle
        self.dial_rotation_wall_line.valueChanged.connect(self.rotate_wall_angle)
        self.pushButton_clockwise.clicked.connect(self.rotate_wall_clockwise)
        self.pushButton_anticlockwise.clicked.connect(self.rotate_wall_anticlockwise)
            # for wall position
        self.pushButton_arrow_up.clicked.connect(self.up_wall_pose)
        self.pushButton_arrow_down.clicked.connect(self.down_wall_pose)
        self.pushButton_arrow_right.clicked.connect(self.right_wall_pose)
        self.pushButton_arrow_left.clicked.connect(self.left_wall_pose)

        # var
            # class
        self._m_gridmap_wall = GridMapWall()
        self._m_view_saved = False
            # grid map
        self._m_gridmap_img = None
            # interval
        self._m_max_interval_value = 10 #[m]
        self.doubleSpinBox_interval_value.setMaximum(self._m_max_interval_value)
        self._m_interval_list = []
            # wall length
        self._m_min_wall_line_length = None
        self._m_max_wall_line_length = None
        
        # init
        self.clear_interval_value()

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ UI Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def shutdown_sys(self):
        self.close()

    # interval setting
    def change_interval_value_of_spinbox(self, value):
        # sync with slider
        slider_value = (value/self._m_max_interval_value)*self.horizontalSlider_interval.maximum()
        self.horizontalSlider_interval.setValue(slider_value)
    def change_interval_value_of_slider(self, value):
        # sync with spinbox
        spinbox_value = (value/self.horizontalSlider_interval.maximum())*(self._m_max_interval_value)
        self.doubleSpinBox_interval_value.setValue(spinbox_value)
    def up_interval_value(self):
        self.doubleSpinBox_interval_value.setValue(self.doubleSpinBox_interval_value.value() + 0.1)
    def down_interval_value(self):
        self.doubleSpinBox_interval_value.setValue(self.doubleSpinBox_interval_value.value() - 0.1)
    def add_interval_value(self):
        if self.doubleSpinBox_interval_value.value() in self._m_interval_list:
            return
        self._m_interval_list.append(self.doubleSpinBox_interval_value.value())
        self.textBrowser_interval_list.insertPlainText("\n"+str(self.doubleSpinBox_interval_value.value()))
    def clear_interval_value(self):
        self.textBrowser_interval_list.clear()
        self._m_interval_list.clear()
        self.textBrowser_interval_list.insertPlainText("[Intreval list]")

    # wall length
    def change_wall_length_of_spinbox(self, value):
        # sync with slider
        slider_value = ((value - self._m_min_wall_line_length) /(self._m_max_wall_line_length - self._m_min_wall_line_length))*self.horizontalSlider_wall_length.maximum()
        self.horizontalSlider_wall_length.setValue(slider_value)
        self._m_gridmap_wall.set_wall_length(value)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())
    def change_wall_length_of_slider(self, value):
        # sync with spinbox
        spinbox_value = (value/self.horizontalSlider_wall_length.maximum())*(self._m_max_wall_line_length - self._m_min_wall_line_length) + self._m_min_wall_line_length
        self.doubleSpinBox_line_length.setValue(spinbox_value)
    def expand_wall_length(self):
        if self.radioButton_fine_mode.isChecked():
            self.doubleSpinBox_line_length.setValue(self.doubleSpinBox_line_length.value() + 0.01)
        else:
            self.doubleSpinBox_line_length.setValue(self.doubleSpinBox_line_length.value() + 0.1)
    def reduce_wall_length(self):
        if self.radioButton_fine_mode.isChecked():
            self.doubleSpinBox_line_length.setValue(self.doubleSpinBox_line_length.value() - 0.01)
        else:
            self.doubleSpinBox_line_length.setValue(self.doubleSpinBox_line_length.value() - 0.1)

    # wall angle
    def rotate_wall_angle(self, angle_degree):
        self._m_gridmap_wall.set_wall_angle(angle_degree/10)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())
    def rotate_wall_clockwise(self):
        if self.radioButton_fine_mode.isChecked():
            self.dial_rotation_wall_line.setValue(self._m_gridmap_wall.get_wall_angle()*10 + 1) # 0.1 degree
        else:
            self.dial_rotation_wall_line.setValue(self._m_gridmap_wall.get_wall_angle()*10 + 10) # 1 degree
    def rotate_wall_anticlockwise(self):
        if self.radioButton_fine_mode.isChecked():
            self.dial_rotation_wall_line.setValue(self._m_gridmap_wall.get_wall_angle()*10 - 0.1)
        else:
            self.dial_rotation_wall_line.setValue(self._m_gridmap_wall.get_wall_angle()*10 - 10)

    # wall position
    def up_wall_pose(self):
        if self.radioButton_fine_mode.isChecked():
            self._m_gridmap_wall.translate_wall_position(0 ,-1)
        else:
            self._m_gridmap_wall.translate_wall_position(0 ,-10)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())
    def down_wall_pose(self):
        if self.radioButton_fine_mode.isChecked():
            self._m_gridmap_wall.translate_wall_position(0 ,1)
        else:
            self._m_gridmap_wall.translate_wall_position(0 ,10)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())
    def right_wall_pose(self):
        if self.radioButton_fine_mode.isChecked():
            self._m_gridmap_wall.translate_wall_position(1 ,0)
        else:
            self._m_gridmap_wall.translate_wall_position(10 ,0)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())
    def left_wall_pose(self):
        if self.radioButton_fine_mode.isChecked():
            self._m_gridmap_wall.translate_wall_position(-1 ,0)
        else:
            self._m_gridmap_wall.translate_wall_position(-10 ,0)
        self.set_girdmap_img(self._m_gridmap_wall.get_img())

    # automatic decision
    def init_wall_automatic(self):
        self._m_gridmap_wall.auto_wall_decision()
        self.doubleSpinBox_line_length.setValue(self._m_gridmap_wall.get_wall_length())
        angle = ((self._m_gridmap_wall.get_wall_angle()+360)%360)
        self.dial_rotation_wall_line.setValue(angle*10)

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def set_girdmap_path_to_waypoint_decision(self, path):
        # initialize this
        self._m_gridmap_wall.initialize(path)
        self.init_girdmap_view()
        self.init_wall()
        self._m_view_saved = True
    
    def init_wall(self):
        self._m_min_wall_line_length, self._m_max_wall_line_length = self._m_gridmap_wall.get_minmax_wall_length()
        self.doubleSpinBox_line_length.setMaximum(self._m_max_wall_line_length)
        self.doubleSpinBox_line_length.setMinimum(self._m_min_wall_line_length)
        self.doubleSpinBox_line_length.setValue(self._m_max_wall_line_length/8)
        self._m_gridmap_wall.set_wall_length(self._m_max_wall_line_length/8)

    def init_girdmap_view(self):
        # (height, width)->(width, height)
        gridmap_img =  np.transpose(self._m_gridmap_wall.get_img(), (1, 0, 2))
        # set image
        self.ImageView_2d_gridmap.setImage(gridmap_img, levels = (0, 255))
        # set view - zoom
        width, height, _ = gridmap_img.shape
        self.ImageView_2d_gridmap.getView().setRange(xRange=[0 + width/2, width/4 + width/2], yRange=[0 + height/2, height/4 + height/2], padding=0)

    def set_girdmap_img(self, img_color):
        curr_view_x_range = self.ImageView_2d_gridmap.getView().viewRange()[0]
        curr_view_y_range = self.ImageView_2d_gridmap.getView().viewRange()[1]

        self.ImageView_2d_gridmap.setImage(np.transpose(img_color, (1, 0, 2)), levels = (0, 255))
        if not self._m_view_saved:
            width, height, _ = img_color.shape
            self.ImageView_2d_gridmap.getView().setRange(xRange=[0 + width/2, width/4 + width/2], yRange=[0 + height/2, height/4 + height/2], padding=0)
        else:
            self.ImageView_2d_gridmap.getView().setXRange(*curr_view_x_range, padding=0)
            self.ImageView_2d_gridmap.getView().setYRange(*curr_view_y_range, padding=0)

# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================

class GridMapWall():
    def __init__(self):
        self.GRID_RESOLUTION = rospy.get_param("/map_converter/handler/octree_resolution")
        self._m_original_gridmap_img = None
        self._m_original_gridmap_img_gray = None
        self._m_gridmap_img = None
        self._m_wall_line_center = [0, 0] # x, y
        self._m_wall_line_length = 1.0
        self._m_wall_line_angle = 0
        self._m_min_wall_line_length = 0.1
        self._m_max_wall_line_length = 10.0
        self._m_wall_angle_visualize_offset = 90.0

    def initialize(self, gird_map_path):
        gird_map_img = cv2.imread(gird_map_path, cv2.IMREAD_UNCHANGED) # (height, width)
        self._m_original_gridmap_img_gray = gird_map_img.copy()
        self._m_gridmap_img = cv2.cvtColor(gird_map_img, cv2.COLOR_GRAY2RGB)
        self._m_original_gridmap_img = cv2.cvtColor(gird_map_img, cv2.COLOR_GRAY2RGB)
        height, width, _ = self._m_gridmap_img.shape
        self._m_wall_line_center = [width/2, height/2]
        self._m_min_wall_line_length = 0.1
        self._m_max_wall_line_length = ((height*self.GRID_RESOLUTION)**2 + (width*self.GRID_RESOLUTION)**2)**(0.5)
        self._m_wall_line_length = 1.0
        self._m_wall_line_angle = 0
    
    def relocate_wall(self):
        pixel_level_length_half = (self._m_wall_line_length/2)/self.GRID_RESOLUTION
        rad_angle = math.radians(self._m_wall_line_angle + self._m_wall_angle_visualize_offset)
        tmp_gridmap_img = self._m_original_gridmap_img.copy()

        tmp_start_pt = (int(self._m_wall_line_center[0]), int(self._m_wall_line_center[1]))
        tmp_end_pt1 = (int(self._m_wall_line_center[0] + pixel_level_length_half*math.cos(rad_angle)), int(self._m_wall_line_center[1] + pixel_level_length_half*math.sin(rad_angle)))
        tmp_end_pt2 = (int(self._m_wall_line_center[0] - pixel_level_length_half*math.cos(rad_angle)), int(self._m_wall_line_center[1] - pixel_level_length_half*math.sin(rad_angle)))

        cv2.arrowedLine(tmp_gridmap_img, tmp_start_pt, tmp_end_pt1, (255, 0, 0), 3, tipLength=0.2)
        cv2.line(tmp_gridmap_img, tmp_start_pt, tmp_end_pt2, (255, 0, 0), 3)

        self._m_gridmap_img = tmp_gridmap_img

        # # debugging
        # resized_image = cv2.resize(tmp_gridmap_img, (0, 0), fx=0.25, fy=0.25)  # Resize to 25% of original size
        # cv2.imshow("test",resized_image)
        # cv2.waitKey(1)
    
    def auto_wall_decision(self):
        _, bin_img = cv2.threshold(self._m_original_gridmap_img_gray, 100, 255, cv2.THRESH_BINARY)
        inversion_img = 255 - bin_img

        kernel = np.ones((3, 3), np.uint8)
        erosion_image = cv2.erode(inversion_img, kernel, iterations=1)  # make erosion image
        dialate_image = cv2.dilate(erosion_image, kernel,iterations=1)
        filter_img = 255- dialate_image

        canny_edge_img = cv2.Canny(filter_img, 150, 270)
        line_result = cv2.cvtColor(self._m_original_gridmap_img_gray, cv2.COLOR_GRAY2BGR)

        max_length = 0
        longest_line = None
        lines = cv2.HoughLinesP(canny_edge_img, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=35)

        # longest line
        if lines is not None:
            longest_line = max(lines, key=lambda line: math.dist((line[0][0], line[0][1]), (line[0][2], line[0][3])))[0]

        if longest_line is not None:
            x1, y1, x2, y2 = longest_line
            self._m_wall_line_length = math.sqrt((x1-x2)**2 + (y1-y2)**2)*self.GRID_RESOLUTION
            self._m_wall_line_center[0] = (x1 + x2)/2
            self._m_wall_line_center[1] = (y1 + y2)/2
            self._m_wall_line_angle = math.degrees(math.atan2(y2-y1, x2-x1)) - self._m_wall_angle_visualize_offset

            self.relocate_wall()
            
    # set
        # length & angle
    def set_wall_length(self, length):
        self._m_wall_line_length = length
        self.relocate_wall()
    def set_wall_angle(self, angle):
        self._m_wall_line_angle = angle
        self.relocate_wall()
        # translation
    def translate_wall_position(self, dx, dy):
        self._m_wall_line_center[0] += dx
        self._m_wall_line_center[1] += dy
        self.relocate_wall()
        # rotation
    def rotation_wall_angle(self, angle):
        self._m_wall_line_angle += angle
        self.relocate_wall()
        # goto
    def goto_wall_position(self, x, y, angle):
        self._m_wall_line_center[0] = x
        self._m_wall_line_center[1] = y
        self._m_wall_line_angle = angle
        self.relocate_wall()

    # get
    def get_img(self):
        return self._m_gridmap_img
    def get_minmax_wall_length(self):
        return self._m_min_wall_line_length, self._m_max_wall_line_length
    def get_wall_angle(self):
        return self._m_wall_line_angle
    def get_wall_center(self):
        return self._m_wall_line_center
    def get_wall_length(self):
        return self._m_wall_line_length