#!/usr/bin/env python3

import sys
import rospy
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from rosbot_competition_msgs.msg import MissionEvent
from cv_bridge import CvBridge

# Attempt to import rviz bindings
try:
    from rviz import bindings as rviz

    HAS_RVIZ = True
except ImportError:
    HAS_RVIZ = False
    rospy.logwarn("rviz python bindings not found. Falling back to map placeholder.")


class RosThread(QThread):
    new_image = pyqtSignal(QImage)
    new_event = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()

    def run(self):
        rospy.Subscriber("/perception/annotated_image", Image, self.image_callback)
        rospy.Subscriber("/mission_events", MissionEvent, self.event_callback)
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            q_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.new_image.emit(q_img)
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")

    def event_callback(self, msg):
        self.new_event.emit(f"[{msg.level}] {msg.message}")


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROSbot Competition Dashboard")
        self.resize(1200, 700)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_layout = QHBoxLayout()

        # Camera Feed
        self.image_label = QLabel("Waiting for camera feed...")
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: black; color: white;")
        top_layout.addWidget(self.image_label)

        # RViz Live Map
        if HAS_RVIZ:
            self.rviz_frame = rviz.VisualizationFrame()
            self.rviz_frame.setSplashPath("")
            self.rviz_frame.initialize()

            reader = rviz.YamlConfigReader()
            config = rviz.Config()
            # If we had a pre-made config, we would load it.
            # For now, just add a Map and RobotModel dynamically
            manager = self.rviz_frame.getManager()
            manager.createDisplay("rviz/Map", "LiveMap", True)
            manager.createDisplay("rviz/RobotModel", "Robot", True)

            top_layout.addWidget(self.rviz_frame)
        else:
            self.rviz_placeholder = QLabel("RViz Bindings Not Available")
            self.rviz_placeholder.setMinimumSize(640, 480)
            self.rviz_placeholder.setAlignment(Qt.AlignCenter)
            top_layout.addWidget(self.rviz_placeholder)

        main_layout.addLayout(top_layout)

        # Event Log
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(200)
        self.console.setStyleSheet(
            "background-color: #2b2b2b; color: #a9b7c6; font-family: monospace;"
        )
        main_layout.addWidget(self.console)

        # Start ROS Thread
        self.ros_thread = RosThread()
        self.ros_thread.new_image.connect(self.update_image)
        self.ros_thread.new_event.connect(self.append_log)
        self.ros_thread.start()

    def update_image(self, q_img):
        pixmap = QPixmap.fromImage(q_img)
        self.image_label.setPixmap(
            pixmap.scaled(
                self.image_label.width(),
                self.image_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def append_log(self, text):
        self.console.appendPlainText(text)


def main():
    rospy.init_node("dashboard_node", anonymous=True)
    app = QApplication(sys.argv)
    dash = Dashboard()
    dash.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
