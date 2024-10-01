import sys
import os
import open3d as o3d
import numpy as np
from time import time
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import *
from amr_ui.srv import *

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication, Qt, QTimer
from PyQt5.QtGui import *
from PyQt5.QtOpenGL import QGLWidget
import pyqtgraph.opengl as gl

path__ = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path__)

path__ = path__ + "/../ui/"
ui_path = path__ + "map_converter_ui.ui"
q_UI_form = uic.loadUiType(ui_path)[0]

class MapConverter(QMainWindow,q_UI_form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self._m_home_path = os.path.expanduser('~')

        # param
        self.VOXEL_SIZE = rospy.get_param("/map_converter/ui/voxel_size")
        self.POINT_SIZE = rospy.get_param("/map_converter/ui/point_size")
        self.Z_PLANE_SIZE = rospy.get_param("/map_converter/ui/z_plane_size")
        # ros
        self._m_client_conversion_flag = rospy.ServiceProxy('/ui/map_converter/bc43cc2993d551056f4ef3801d525ce0', map_conversion_setting)

        # init viewer
        self._m_map_viewer = Viewer(self)
        self.verticalLayout_vis_map.addWidget(self._m_map_viewer.initialize_widget())

        # init ui function
            # for system
        self.pushButton_close.clicked.connect(self.shutdown_sys)
        self.pushButton_open_pcd.clicked.connect(self.open_pcd)
        self.pushButton_clear_log.clicked.connect(self.clear_log)
            # for z value
        self.doubleSpinBox_z_value.valueChanged.connect(self.change_value_of_spinbox)
        self.verticalSlider_slide_z.valueChanged.connect(self.change_value_of_slider)
        self.pushButton_up_z.clicked.connect(self.up_z_value)
        self.pushButton_down_z.clicked.connect(self.down_z_value)
            # for viewing direction
        self.pushButton_view_home.clicked.connect(self.reset_viewing_direction)

        # member variable
            # path
        self._m_pcd_path = ""
            # z min-max ratio
        self._m_z_ratio = 1.0
        self._m_z_offset = 0
            # setting OK
        self._m_setting_status = False

        # init log
        self.textBrowser_log.insertPlainText("[SUCCESS]Init Success!")

    # drag and drop
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()
    def dropEvent(self, event):
        files = [u.toLocalFile() for u in event.mimeData().urls()]
        if len(files) > 1:
            self.add_log("[FAIL]Reject Drag and Drop. Drag \"Only One\" File")
        if len(files) == 1 and files[0][-4:] == ".pcd":
            self.load_3d_map(files[0])
        else: 
            self.add_log("[FAIL]Reject Drag and Drop. Drag Only \".pcd\"")
    
    def getValueZ(self):
        return self.doubleSpinBox_z_value.value()

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ UI Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def shutdown_sys(self):
        self.close()

    def open_pcd(self):
        file_name = QFileDialog.getOpenFileName(self, 'Open .pcd file', self._m_home_path, 'pcd(*.pcd)')
        if not file_name[0]:
            return
        if file_name[0][-4:] != ".pcd":
            self.add_log("[FAIL]File format is not pcd")
            return
        self.load_3d_map(file_name[0])

    def clear_log(self):
        self.textBrowser_log.clear()
        self.textBrowser_log.insertPlainText("[INFO]Clear log")

    def change_value_of_spinbox(self, value):
        # sync with slider
        slider_value = (value - self._m_z_offset)*self._m_z_ratio
        self.verticalSlider_slide_z.setValue(slider_value)
        # set z to viewer
        self._m_map_viewer.set_z_plane(value)

    def change_value_of_slider(self, value):
        # sync with spinbox
        spinbox_value = (value/self._m_z_ratio) + self._m_z_offset
        self.doubleSpinBox_z_value.setValue(spinbox_value)

    def up_z_value(self):
        self.doubleSpinBox_z_value.setValue(self.doubleSpinBox_z_value.value() + 0.1)
    def down_z_value(self):
        self.doubleSpinBox_z_value.setValue(self.doubleSpinBox_z_value.value() - 0.1)
    
    def reset_viewing_direction(self):
        self._m_map_viewer.init_viewing_direction()
        self.add_log("[INFO]Reset vewing direction")

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For Callback Function @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def add_log(self, log_msg):
        self.textBrowser_log.insertPlainText("\n")
        self.textBrowser_log.insertPlainText(log_msg)
        self.textBrowser_log.verticalScrollBar().setValue(self.textBrowser_log.verticalScrollBar().maximum())

    def load_3d_map(self, file_name):
        self._m_pcd_path = file_name
        self.lineEdit_3dmap_dir.setText(file_name[:-4])
        self.add_log("[SUCCESS]Open pcd file:" + file_name)

        # read pcd file
        time_tmp = time()
        pcd = o3d.io.read_point_cloud(file_name)
        self.add_log("[INFO]Map read time:" + f"{time() - time_tmp:.2f}s")

        # voxelization
        time_tmp = time()
        voxel_down_pcd = pcd.voxel_down_sample(voxel_size = self.VOXEL_SIZE)
        self.add_log("[INFO]Map downsample time:" + f"{time() - time_tmp:.2f}s")

        # pointcloud center
        center = voxel_down_pcd.get_center()

        # init 3d map
        points = np.asarray(voxel_down_pcd.points)
        z_max, z_min = self._m_map_viewer.init_pointcloud(points)

        # ui z min max value
        self.label_z_max.setText(f"{z_max:.2f}")
        self.label_z_min.setText(f"{z_min:.2f}")
        self.doubleSpinBox_z_value.setMaximum(z_max)
        self.doubleSpinBox_z_value.setMinimum(z_min)
        self._m_z_offset = z_min
        self._m_z_ratio = (self.verticalSlider_slide_z.maximum() - self.verticalSlider_slide_z.minimum())/(z_max - z_min)
        self.verticalSlider_slide_z.setValue((0 - self._m_z_offset) * self._m_z_ratio)

        # init z_plane
        self._m_map_viewer.init_z_plane(center)
        self.doubleSpinBox_z_value.setValue(z_min)

        # is OK
        self._m_setting_status = True
    
    def convert(self):
        if not self._m_setting_status:
            return False
        map_setting = map_conversion_settingRequest()
        map_setting.pcd_path = self._m_pcd_path
        map_setting.z_value = self.doubleSpinBox_z_value.value()
        res_map_conversion = self._m_client_conversion_flag(map_setting)
        return True
    
    def stop(self):
        map_setting = map_conversion_settingRequest()
        map_setting.pcd_path = "stop_bc43cc2993d551056f4ef3801d525ce0"
        map_setting.z_value = 2001
        res_map_conversion = self._m_client_conversion_flag(map_setting)
    
    def release(self):
        # memory release
        self._m_map_viewer.reset()
        
# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================
# ============================================================================================

class Viewer(QWidget):
    def __init__(self, parent=None):
        super(Viewer, self).__init__(parent)
        self.POINT_SIZE = parent.POINT_SIZE
        self.Z_PLANE_SIZE = parent.Z_PLANE_SIZE

        self._m_glwidget = gl.GLViewWidget()
        self.init_view()
        # 
        self._m_camera_reset_info = [
            self._m_glwidget.opts['center'],
            self._m_glwidget.opts['distance'],
            self._m_glwidget.opts['elevation'],
            self._m_glwidget.opts['azimuth']
            ]

        self._m_curr_z_value = 0.0
        self._m_map_data = None
        self._m_z_mesh_item = None
    
    def init_viewing_direction(self):
        self._m_glwidget.setCameraPosition(pos=self._m_camera_reset_info[0],
                                           distance=self._m_camera_reset_info[1], 
                                           azimuth=self._m_camera_reset_info[3], 
                                           elevation=self._m_camera_reset_info[2])

    def init_view(self):
        self._m_glwidget.opts['distance'] = 200
        grid = gl.GLGridItem()
        grid.scale(10, 10, 1)
        self._m_glwidget.clear()
        self._m_glwidget.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(x=100, y=100, z=100)  # 축 크기 설정
        self._m_glwidget.addItem(axis)

    def initialize_widget(self):
        return self._m_glwidget
    
    def init_pointcloud(self, np_points):
        self.init_view()
        self._m_map_data = np_points

        # color init
        z_vals = np_points[:, 2]
        z_min, z_max = z_vals.min(), z_vals.max()
        norm_z = (z_vals - z_min) / (z_max - z_min)

        # use rainbow color map
        cmap = plt.get_cmap('rainbow')
        colors = cmap(norm_z)
        colors[:, 3] = 0.7 # alpha

        # view pointcloud
        scatter = gl.GLScatterPlotItem(pos=self._m_map_data, size=self.POINT_SIZE, color=colors, pxMode=False)
        scatter.setGLOptions('translucent')
        # scatter.setGLOptions('opaque')
        self._m_glwidget.addItem(scatter)

        return z_max, z_min

    def init_z_plane(self, center):
        # init rectangle
        z_plane = o3d.geometry.TriangleMesh.create_box(width=self.Z_PLANE_SIZE, height=self.Z_PLANE_SIZE, depth=0.001)
        # pose
        z_plane.translate(np.array([center[0] - self.Z_PLANE_SIZE/2, center[1] - self.Z_PLANE_SIZE/2, 0]))
        # color
        z_plane.paint_uniform_color([0.9, 0.1, 0.1]) 

        # Open3D TriangleMesh to PyQtGraph
        vertices = np.asarray(z_plane.vertices)
        triangles = np.asarray(z_plane.triangles)
        vertex_colors = np.asarray(z_plane.vertex_colors)

        # Generate GLMeshItem of PyQtGraph
        meshdata = gl.MeshData(vertexes=vertices, faces=triangles, vertexColors=vertex_colors)
        self._m_z_mesh_item = gl.GLMeshItem(meshdata=meshdata, smooth=False, drawEdges=False, edgeColor=(0, 0, 0, 1))
        self._m_z_mesh_item.setGLOptions('opaque')

        self._m_glwidget.addItem(self._m_z_mesh_item)

    def set_z_plane(self, new_z_value):
        dz = new_z_value - self._m_curr_z_value
        self._m_z_mesh_item.translate(0, 0, dz)
        self._m_curr_z_value = new_z_value

    def reset(self):
        self.init_view()
        self._m_map_data = None
        self._m_z_mesh_item = None