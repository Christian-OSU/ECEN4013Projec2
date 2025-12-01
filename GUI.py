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
PORT_NAME = "COM8"  # replace with your port
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
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.sat = 0
        
        # Uptime
        self.uptime = "00:00:00"

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

            with sensor_data.lock:
                # Parse Accel: x,y,z
                if line.startswith("Accel:"):
                    vals = line.split(":")[1].strip().split(",")
                    sensor_data.accel = np.array([float(v) for v in vals])
                    idx = sensor_data.buf_index
                    sensor_data.accel_buf[idx] = sensor_data.accel
                    sensor_data.buf_index = (idx+1) % BUF_LEN

                # Parse Gyro: x,y,z
                elif line.startswith("Gyro:"):
                    vals = line.split(":")[1].strip().split(",")
                    sensor_data.gyro = np.array([float(v) for v in vals])
                    idx = sensor_data.buf_index
                    sensor_data.gyro_buf[idx] = sensor_data.gyro

                # Parse Mag: x,y,z
                elif line.startswith("Mag:"):
                    vals = line.split(":")[1].strip().split(",")
                    sensor_data.mag = np.array([float(v) for v in vals])
                    idx = sensor_data.buf_index
                    sensor_data.mag_buf[idx] = sensor_data.mag

                # Parse GPS: Lat: x, Lon: y, Alt: z, Sat: n
                elif line.startswith("Lat:"):
                    # Split by comma to get each field
                    parts = line.split(",")
                    for part in parts:
                        part = part.strip()
                        if part.startswith("Lat:"):
                            sensor_data.lat = float(part.split(":")[1].strip())
                        elif part.startswith("Lon:"):
                            sensor_data.lon = float(part.split(":")[1].strip())
                        elif part.startswith("Alt:"):
                            sensor_data.alt = float(part.split(":")[1].strip())
                        elif part.startswith("Sat:"):
                            sensor_data.sat = int(part.split(":")[1].strip())
                
                # Parse Uptime: hh:mm:ss
                elif line.startswith("Uptime:"):
                    # Split only on first colon to preserve hh:mm:ss format
                    sensor_data.uptime = line.split(":", 1)[1].strip()

                # Calculate orientation from accelerometer (simple tilt calculation)
                # This gives pitch and roll, but not yaw (which requires magnetometer fusion)
                ax, ay, az = sensor_data.accel
                sensor_data.pitch = np.degrees(np.arctan2(ay, az))
                sensor_data.roll = np.degrees(np.arctan2(-ax, np.sqrt(ay*ay + az*az)))
                # Yaw calculation from magnetometer (simplified, assumes level sensor)
                mx, my, mz = sensor_data.mag
                sensor_data.yaw = np.degrees(np.arctan2(-my, mx))

        except Exception as e:
            print("Serial parse error:", e, "Line:", line)
    
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
        self.start_time = None

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
        self.accel_plot.addLegend()
        self.accel_curves = [
            self.accel_plot.plot(pen=pg.mkPen('r'), name='X'),
            self.accel_plot.plot(pen=pg.mkPen('g'), name='Y'),
            self.accel_plot.plot(pen=pg.mkPen('b'), name='Z')
        ]
        self.plot_widget.nextRow()
        # Gyro
        self.gyro_plot = self.plot_widget.addPlot(title="Gyro (rad/s)")
        self.gyro_plot.setYRange(-8,8)
        self.gyro_plot.addLegend()
        self.gyro_curves = [
            self.gyro_plot.plot(pen=pg.mkPen('r'), name='X'),
            self.gyro_plot.plot(pen=pg.mkPen('g'), name='Y'),
            self.gyro_plot.plot(pen=pg.mkPen('b'), name='Z')
        ]
        self.plot_widget.nextRow()
        # Magnetometer
        self.mag_plot = self.plot_widget.addPlot(title="Mag (µT)")
        self.mag_plot.setYRange(-200,200)
        self.mag_plot.addLegend()
        self.mag_curves = [
            self.mag_plot.plot(pen=pg.mkPen('r'), name='X'),
            self.mag_plot.plot(pen=pg.mkPen('g'), name='Y'),
            self.mag_plot.plot(pen=pg.mkPen('b'), name='Z')
        ]

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
    def rotation_matrix(self, pitch, roll, yaw):
        pitch_rad = np.radians(pitch)
        roll_rad = np.radians(roll)
        yaw_rad = np.radians(yaw)
        
        # Rotation around X axis (pitch)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
                       [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
        
        # Rotation around Y axis (roll)
        Ry = np.array([[np.cos(roll_rad), 0, np.sin(roll_rad)],
                       [0, 1, 0],
                       [-np.sin(roll_rad), 0, np.cos(roll_rad)]])
        
        # Rotation around Z axis (yaw)
        Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                       [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                       [0, 0, 1]])
        
        # Apply rotations in order: yaw, then roll, then pitch
        return Rx @ Ry @ Rz

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
            text = f"""Orientation (pitch, roll, yaw): {self.sensor_data.pitch:.2f}°, {self.sensor_data.roll:.2f}°, {self.sensor_data.yaw:.2f}°
Gyro: {self.sensor_data.gyro[0]:.4f}, {self.sensor_data.gyro[1]:.4f}, {self.sensor_data.gyro[2]:.4f} rad/s
Accel: {self.sensor_data.accel[0]:.4f}, {self.sensor_data.accel[1]:.4f}, {self.sensor_data.accel[2]:.4f} m/s²
Mag: {self.sensor_data.mag[0]:.2f}, {self.sensor_data.mag[1]:.2f}, {self.sensor_data.mag[2]:.2f} µT
GPS - Lat: {self.sensor_data.lat:.6f}, Lon: {self.sensor_data.lon:.6f}, Alt: {self.sensor_data.alt:.2f}m
Satellites: {self.sensor_data.sat}
Uptime: {self.sensor_data.uptime}
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