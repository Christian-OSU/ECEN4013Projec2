import sys
import serial
import numpy as np
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl
from pyqtgraph import Vector

class IMUBoxViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU 3D Box Viewer")

        # UI layout
        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for IMU data...")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # 3D view
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10)
        self.layout.addWidget(self.view)

        # Create box
        self.box = gl.GLBoxItem(size=Vector(3, 1, 2))
        self.view.addItem(self.box)

        # Orientation angles
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        # Serial setup
        try:
            self.ser = serial.Serial(port='COM9', baudrate=115200, timeout=1)
        except serial.SerialException as e:
            self.label.setText(f"Serial error: {e}")
            self.ser = None

        # Timer loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_orientation)
        self.timer.start(50)

    def update_orientation(self):
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.label.setText(f"Received: {line}")
                    values = line.split(',')
                    if len(values) == 3:
                        self.roll = float(values[0])
                        self.pitch = float(values[1])
                        self.yaw = float(values[2])
                        self.apply_rotation()
            except Exception as e:
                self.label.setText(f"Read error: {e}")

    def apply_rotation(self):
        self.box.resetTransform()

        # Convert degrees to radians
        rx, ry, rz = np.radians([self.roll, self.pitch, self.yaw])

        # Rotation matrices
        rot_x = np.array([[1, 0, 0, 0],
                          [0, np.cos(rx), -np.sin(rx), 0],
                          [0, np.sin(rx), np.cos(rx), 0],
                          [0, 0, 0, 1]])

        rot_y = np.array([[np.cos(ry), 0, np.sin(ry), 0],
                          [0, 1, 0, 0],
                          [-np.sin(ry), 0, np.cos(ry), 0],
                          [0, 0, 0, 1]])

        rot_z = np.array([[np.cos(rz), -np.sin(rz), 0, 0],
                          [np.sin(rz), np.cos(rz), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Combined rotation: Z * Y * X
        rotation_matrix = rot_z @ rot_y @ rot_x

        # Apply transformation
        self.box.transform(rotation_matrix)

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IMUBoxViewer()
    window.resize(600, 600)
    window.show()
    sys.exit(app.exec())