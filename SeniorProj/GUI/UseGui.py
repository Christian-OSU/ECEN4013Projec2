import sys
import threading
import numpy as np
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData

# ---------------------------
# Configuration
# ---------------------------
PORT_NAME = "COM9"
BAUD_RATE = 115200
BUF_LEN = 300
SAMPLE_INTERVAL_MS = 30

# ---------------------------
# Sensor Data Class
# ---------------------------
class SensorData:
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)
        self.mag = np.zeros(3)

        self.time = ""
        self.fix = ""
        self.sat = ""
        self.lat = ""
        self.longi = ""
        self.elevation = ""

        self.accel_buf = np.zeros((BUF_LEN, 3))
        self.gyro_buf = np.zeros((BUF_LEN, 3))
        self.mag_buf = np.zeros((BUF_LEN, 3))
        self.buf_index = 0

        self.lock = threading.Lock()

# ---------------------------
# Serial Thread
# ---------------------------
def serial_thread(port_name, baud, sensor_data, running_flag):
    try:
        ser = serial.Serial(port_name, baud, timeout=1)
    except Exception as e:
        print("Error opening serial port:", e)
        running_flag[0] = False
        return

    while running_flag[0]:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            # IMU data
            if line.startswith("Orientation"):
                groups = [g.strip() for g in line.split(";")]
                with sensor_data.lock:
                    for g in groups:
                        if g.startswith("Orientation"):
                            vals = list(map(float, g.split(":")[1].split(",")))
                            sensor_data.pitch, sensor_data.roll, sensor_data.yaw = vals
                        elif g.startswith("Angular Velocity"):
                            sensor_data.gyro = np.array(list(map(float, g.split(":")[1].split(","))))
                        elif g.startswith("Linear Acceleration"):
                            sensor_data.accel = np.array(list(map(float, g.split(":")[1].split(","))))
                        elif g.startswith("Magnetic Field"):
                            sensor_data.mag = np.array(list(map(float, g.split(":")[1].split(","))))
                    idx = sensor_data.buf_index
                    sensor_data.accel_buf[idx] = sensor_data.accel
                    sensor_data.gyro_buf[idx] = sensor_data.gyro
                    sensor_data.mag_buf[idx] = sensor_data.mag
                    sensor_data.buf_index = (idx + 1) % BUF_LEN

            # GPS data
            elif line.startswith("Time"):
                groups = [g.strip() for g in line.split(";")]
                with sensor_data.lock:
                    for g in groups:
                        if g.startswith("Time"):
                            sensor_data.time = g.split(":")[1].strip()
                        elif g.startswith("Fix"):
                            sensor_data.fix = g.split(":")[1].strip()
                        elif g.startswith("Satellites"):
                            sensor_data.sat = g.split(":")[1].strip()
                        elif g.startswith("Latitude"):
                            sensor_data.lat = g.split(":")[1].strip()
                        elif g.startswith("Longitude"):
                            sensor_data.longi = g.split(":")[1].strip()
                        elif g.startswith("Elevation"):
                            sensor_data.elevation = g.split(":")[1].strip()
        except Exception as e:
            print("Serial parse error:", e, line)
    ser.close()

# ---------------------------
# Main Window
# ---------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU + GPS Monitor")
        self.resize(1200, 700)

        self.sensor_data = SensorData()
        self.running_flag = [False]

        layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(top_layout)

        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        top_layout.addWidget(self.start_btn)
        top_layout.addWidget(self.stop_btn)
        self.start_btn.clicked.connect(self.start)
        self.stop_btn.clicked.connect(self.stop)

        # ---------------------------
        # 3D GL Widget
        # ---------------------------
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=5)
        layout.insertWidget(0, self.gl_widget, 1)

        # Grid
        grid = gl.GLGridItem()
        grid.scale(1,1,1)
        self.gl_widget.addItem(grid)

        # Box using GLMeshItem for visibility
        # Create a cube mesh manually
        verts = np.array([
            [-0.5,-0.5,-0.5],
            [ 0.5,-0.5,-0.5],
            [ 0.5, 0.5,-0.5],
            [-0.5, 0.5,-0.5],
            [-0.5,-0.5, 0.5],
            [ 0.5,-0.5, 0.5],
            [ 0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5]
        ])
        
        faces = np.array([
            [0,1,2], [0,2,3],  # bottom
            [4,5,6], [4,6,7],  # top
            [0,1,5], [0,5,4],  # front
            [2,3,7], [2,7,6],  # back
            [1,2,6], [1,6,5],  # right
            [0,3,7], [0,7,4]   # left
        ])
        
        md = gl.MeshData(vertexes=verts, faces=faces)
        self.box = gl.GLMeshItem(meshdata=md,
                                 smooth=False,
                                 color=(0.4,0.6,1,0.8),
                                 shader='shaded',
                                 drawEdges=True,
                                 edgeColor=(1,1,1,1))
        self.box.translate(0,0,0.5)
        self.gl_widget.addItem(self.box)


        # ---------------------------
        # Plots
        # ---------------------------
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        self.accel_plot = self.plot_widget.addPlot(title="Acceleration (m/s²)")
        self.accel_plot.setYRange(-16,16)
        self.accel_curves = [self.accel_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]
        self.plot_widget.nextRow()

        self.gyro_plot = self.plot_widget.addPlot(title="Gyro (rad/s)")
        self.gyro_plot.setYRange(-8,8)
        self.gyro_curves = [self.gyro_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]
        self.plot_widget.nextRow()

        self.mag_plot = self.plot_widget.addPlot(title="Mag (µT)")
        self.mag_plot.setYRange(-200,200)
        self.mag_curves = [self.mag_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]

        # Text
        self.info_text = QtWidgets.QTextEdit()
        self.info_text.setReadOnly(True)
        layout.addWidget(self.info_text)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(SAMPLE_INTERVAL_MS)

    def start(self):
        if self.running_flag[0]:
            return
        self.running_flag[0] = True
        self.serial_thread = threading.Thread(target=serial_thread,
                                              args=(PORT_NAME, BAUD_RATE, self.sensor_data, self.running_flag))
        self.serial_thread.start()

    def stop(self):
        self.running_flag[0] = False

    def closeEvent(self, event):
        self.running_flag[0] = False
        event.accept()

    def update_plots(self):
        with self.sensor_data.lock:
            idx = self.sensor_data.buf_index
            accel_data = np.roll(self.sensor_data.accel_buf, -idx, axis=0)
            gyro_data = np.roll(self.sensor_data.gyro_buf, -idx, axis=0)
            mag_data = np.roll(self.sensor_data.mag_buf, -idx, axis=0)

            for i in range(3):
                self.accel_curves[i].setData(accel_data[:,i])
                self.gyro_curves[i].setData(gyro_data[:,i])
                self.mag_curves[i].setData(mag_data[:,i])

            # Text
            text = f"""Orientation (pitch,roll,yaw): {self.sensor_data.pitch:.2f}, {self.sensor_data.roll:.2f}, {self.sensor_data.yaw:.2f}
Gyro: {self.sensor_data.gyro[0]:.2f}, {self.sensor_data.gyro[1]:.2f}, {self.sensor_data.gyro[2]:.2f}
Accel: {self.sensor_data.accel[0]:.2f}, {self.sensor_data.accel[1]:.2f}, {self.sensor_data.accel[2]:.2f}
Mag: {self.sensor_data.mag[0]:.2f}, {self.sensor_data.mag[1]:.2f}, {self.sensor_data.mag[2]:.2f}
Time: {self.sensor_data.time} | Fix: {self.sensor_data.fix} | Satellites: {self.sensor_data.sat}
Lat: {self.sensor_data.lat} | Lon: {self.sensor_data.longi} | Elev: {self.sensor_data.elevation}
"""
            self.info_text.setPlainText(text)

            # Rotate box
            pitch = self.sensor_data.pitch
            roll  = self.sensor_data.roll
            yaw   = self.sensor_data.yaw
            self.box.resetTransform()
            self.box.translate(0,0,0.5)
            self.box.rotate(pitch,1,0,0)
            self.box.rotate(yaw,0,1,0)
            self.box.rotate(roll,0,0,1)

# ---------------------------
# Run
# ---------------------------
if __name__ == "__main__":
    # Attempt to open serial port first
    try:
        ser_test = serial.Serial(PORT_NAME, BAUD_RATE, timeout=1)
        ser_test.close()
    except Exception as e:
        print("Cannot open serial port:", e)
        sys.exit(1)

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
