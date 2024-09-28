import sys
import os

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

path__ = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path__)

path__ = path__ + "/../ui/"
ui_path = path__ + "waypoint_decision_ui.ui"
q_UI_form = uic.loadUiType(ui_path)[0]

class WaypointDecision(QMainWindow,q_UI_form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton_close.clicked.connect(self.shutdown_sys)

    def shutdown_sys(self):
        self.close()
