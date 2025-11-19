import sys
import threading
import time
import numpy as np
import serial

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, GLLinePlotItem

# ---------------------------
# Configuration
# ---------------------------
PORT_NAME = "COM9"  # replace with your port
BAUD_RATE = 115200
BUF_LEN = 300
SAMPLE_INTERVAL_MS = 30

# ---------------------------
# Helper: Create Cube Mesh
# ---------------------------
def create_cube_mesh(size=(1,1,1)):
    """Returns (vertices, faces) for a cube of given size"""
    sx, sy, sz = size[0]/2, size[1]/2, size[2]/2
    vertices = np.array([
        [-sx,-sy,-sz],
        [ sx,-sy,-sz],
        [ sx, sy,-sz],
        [-sx, sy,-sz],
        [-sx,-sy, sz],
        [ sx,-sy, sz],
        [ sx, sy, sz],
        [-sx, sy, sz]
    ])
    faces = np.array([
        [0,1,2],[0,2,3],  # bottom
        [4,5,6],[4,6,7],  # top
        [0,1,5],[0,5,4],  # front
        [1,2,6],[1,6,5],  # right
        [2,3,7],[2,7,6],  # back
        [3,0,4],[3,4,7]   # left
    ])
    return vertices, faces

# ---------------------------
# Sensor Data Class
# ---------------------------
class SensorData:
    def __init__(self):
        # IMU values
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)
        self.mag = np.zeros(3)

        # GPS values
        self.time = ""
        self.fix = ""
        self.sat = ""
        self.lat = ""
        self.longi = ""
        self.elevation = ""

        # Circular buffers
        self.accel_buf = np.zeros((BUF_LEN,3))
        self.gyro_buf = np.zeros((BUF_LEN,3))
        self.mag_buf = np.zeros((BUF_LEN,3))
        self.buf_index = 0

        # Thread lock
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
                            vals = list(map(float, g.split(":")[1].split(",")))
                            sensor_data.gyro = np.array(vals)
                        elif g.startswith("Linear Acceleration"):
                            vals = list(map(float, g.split(":")[1].split(",")))
                            sensor_data.accel = np.array(vals)
                        elif g.startswith("Magnetic Field"):
                            vals = list(map(float, g.split(":")[1].split(",")))
                            sensor_data.mag = np.array(vals)
                    # Update buffers
                    idx = sensor_data.buf_index
                    sensor_data.accel_buf[idx] = sensor_data.accel
                    sensor_data.gyro_buf[idx] = sensor_data.gyro
                    sensor_data.mag_buf[idx] = sensor_data.mag
                    sensor_data.buf_index = (idx+1)%BUF_LEN

            # GPS data
            elif line.startswith("Time"):
                groups = [g.strip() for g in line.split(";")]
                with sensor_data.lock:
                    for g in groups:
                        if g.startswith("Time"):
                            # Extract the raw time string after "Time:"
                            raw_time = g.split(":", 1)[1].strip()  # split only on first colon
                            # Split into parts
                            parts = raw_time.split(":")
                            # Ensure we have hours, minutes, seconds
                            if len(parts) == 3:
                                h, m, s = parts
                            elif len(parts) == 2:
                                h = 0
                                m, s = parts
                            else:
                                h = m = 0
                                s = parts[0]
                            # Format as HH:MM:SS
                            sensor_data.time = f"{int(h):02d}:{int(m):02d}:{int(s):02d}"

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
        self.resize(1200,700)

        self.sensor_data = SensorData()
        self.running_flag = [False]

        # ---------------------------
        # Layout
        layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(top_layout)

        # Start/Stop buttons
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        top_layout.addWidget(self.start_btn)
        top_layout.addWidget(self.stop_btn)
        self.start_btn.clicked.connect(self.start)
        self.stop_btn.clicked.connect(self.stop)

        # ---------------------------
        # GL Widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=6)
        layout.insertWidget(0, self.gl_widget, 1)

        # Grid
        grid = gl.GLGridItem()
        grid.scale(1,1,1)
        self.gl_widget.addItem(grid)

        # Cube mesh
        self.box_size = (1,0.5,0.3)
        verts, faces = create_cube_mesh(self.box_size)
        self.faces = faces
        self.box = GLMeshItem(vertexes=verts, faces=faces, smooth=False,
                              color=(0.4,0.6,1,0.8), shader='shaded',
                              drawEdges=True, edgeColor=(0,0,0,1))
        self.gl_widget.addItem(self.box)
        self.original_vertices = verts.copy()

        # Axes
        axis_len = 1.5
        self.x_axis = GLLinePlotItem(pos=np.array([[0,0,0],[axis_len,0,0]]), color=(1,0,0,1), width=3, antialias=True)
        self.y_axis = GLLinePlotItem(pos=np.array([[0,0,0],[0,axis_len,0]]), color=(0,1,0,1), width=3, antialias=True)
        self.z_axis = GLLinePlotItem(pos=np.array([[0,0,0],[0,0,axis_len]]), color=(0,0,1,1), width=3, antialias=True)
        for axis in [self.x_axis, self.y_axis, self.z_axis]:
            self.gl_widget.addItem(axis)

        # ---------------------------
        # Strip charts
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # Accelerometer
        self.accel_plot = self.plot_widget.addPlot(title="Acceleration (m/s²)")
        self.accel_plot.setYRange(-16,16)
        self.accel_curves = [self.accel_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]
        self.plot_widget.nextRow()
        # Gyro
        self.gyro_plot = self.plot_widget.addPlot(title="Gyro (rad/s)")
        self.gyro_plot.setYRange(-8,8)
        self.gyro_curves = [self.gyro_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]
        self.plot_widget.nextRow()
        # Magnetometer
        self.mag_plot = self.plot_widget.addPlot(title="Mag (µT)")
        self.mag_plot.setYRange(-200,200)
        self.mag_curves = [self.mag_plot.plot(pen=pg.mkPen(c)) for c in ['r','g','b']]

        # GPS/IMU text
        self.info_text = QtWidgets.QTextEdit()
        self.info_text.setReadOnly(True)
        layout.addWidget(self.info_text)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(SAMPLE_INTERVAL_MS)

    # ---------------------------
    # Start / Stop
    def start(self):
        if self.running_flag[0]:
            return
        self.running_flag[0] = True
        self.serial_thread = threading.Thread(target=serial_thread,
                                              args=(PORT_NAME, BAUD_RATE, self.sensor_data, self.running_flag))
        self.serial_thread.start()

    def stop(self):
        self.running_flag[0] = False

    # ---------------------------
    # Rotation helper
    def rotation_matrix(self, yaw, pitch, roll):
        yaw = np.radians(yaw)
        pitch = np.radians(pitch)
        roll = np.radians(roll)
        Rx = np.array([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])
        Ry = np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
        Rz = np.array([[np.cos(roll),-np.sin(roll),0],[np.sin(roll),np.cos(roll),0],[0,0,1]])
        return Rz @ Ry @ Rx

    # ---------------------------
    # Update GUI
    def update_plots(self):
        with self.sensor_data.lock:
            idx = self.sensor_data.buf_index
            accel_data = np.roll(self.sensor_data.accel_buf, -idx, axis=0)
            gyro_data = np.roll(self.sensor_data.gyro_buf, -idx, axis=0)
            mag_data = np.roll(self.sensor_data.mag_buf, -idx, axis=0)

            # Update curves
            for i in range(3):
                self.accel_curves[i].setData(accel_data[:,i])
                self.gyro_curves[i].setData(gyro_data[:,i])
                self.mag_curves[i].setData(mag_data[:,i])

            # Update GPS/IMU text
            text = f"""Orientation (pitch,roll,yaw): {self.sensor_data.pitch:.2f}, {self.sensor_data.roll:.2f}, {self.sensor_data.yaw:.2f}
Gyro: {self.sensor_data.gyro[0]:.2f}, {self.sensor_data.gyro[1]:.2f}, {self.sensor_data.gyro[2]:.2f}
Accel: {self.sensor_data.accel[0]:.2f}, {self.sensor_data.accel[1]:.2f}, {self.sensor_data.accel[2]:.2f}
Mag: {self.sensor_data.mag[0]:.2f}, {self.sensor_data.mag[1]:.2f}, {self.sensor_data.mag[2]:.2f}
Time: {self.sensor_data.time} | Fix: {self.sensor_data.fix} | Satellites: {self.sensor_data.sat}
Lat: {self.sensor_data.lat} | Lon: {self.sensor_data.longi} | Elev: {self.sensor_data.elevation}
"""
            self.info_text.setPlainText(text)

            # ---------------------------
            # Rotate cube
            R = self.rotation_matrix(self.sensor_data.pitch, self.sensor_data.roll, self.sensor_data.yaw)
            verts_rot = (R @ self.original_vertices.T).T
            self.box.setMeshData(vertexes=verts_rot, faces=self.faces)

            # Rotate axes
            axes_positions = [
                np.array([[0,0,0],[1.5,0,0]]),
                np.array([[0,0,0],[0,1.5,0]]),
                np.array([[0,0,0],[0,0,1.5]])
            ]
            for axis, pos in zip([self.x_axis,self.y_axis,self.z_axis], axes_positions):
                rotated = (R @ pos.T).T
                axis.setData(pos=rotated)

# ---------------------------
# Run
# ---------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
