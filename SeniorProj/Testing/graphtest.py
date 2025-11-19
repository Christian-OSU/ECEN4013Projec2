import sys
import serial
import numpy as np
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl
from pyqtgraph import Vector

class IMU3DBox(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D IMU Orientation Viewer")

        # UI setup
        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for IMU data...")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # 3D view setup
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10)
        self.layout.addWidget(self.view)

        # Create a cube mesh
        self.box = gl.GLBoxItem(size=Vector(2, 2, 2))
        self.view.addItem(self.box)

        # Serial setup
        try:
            self.ser = serial.Serial(port='COM9', baudrate=9600, timeout=1)
        except serial.SerialException as e:
            self.label.setText(f"Serial error: {e}")
            self.ser = None

        # Timer setup
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(10)

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.label.setText(f"Received: {line}")

                    # Expecting format: "Gyro: Xval, Yval, Zval"
                    if line.startswith("Gyro:"):
                        parts = line.replace("Gyro:", "").strip().split(',')
                        if len(parts) == 3:
                            x = float(parts[0])
                            y = float(parts[1])
                            z = float(parts[2])
                            print(f"Parsed Gyro → X: {x}, Y: {y}, Z: {z}")

                            # Convert degrees to radians
                            rx, ry, rz = np.radians([x, y, z])

                            # Build rotation matrix
                            rot_x = np.array([[1, 0, 0],
                                              [0, np.cos(rx), -np.sin(rx)],
                                              [0, np.sin(rx), np.cos(rx)]])
                            rot_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                                              [0, 1, 0],
                                              [-np.sin(ry), 0, np.cos(ry)]])
                            rot_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                                              [np.sin(rz), np.cos(rz), 0],
                                              [0, 0, 1]])

                            rotation = rot_z @ rot_y @ rot_x
                            self.box.resetTransform()
                            self.box.rotate(x, 1, 0, 0)
                            self.box.rotate(y, 0, 1, 0)
                            self.box.rotate(z, 0, 0, 1)
            except Exception as e:
                self.label.setText(f"Read error: {e}")

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IMU3DBox()
    window.resize(600, 600)
    window.show()
    sys.exit(app.exec())