import sys
import threading
import time
import numpy as np
import serial

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# ---------------------------
# Configuration
# ---------------------------
PORT_NAME = "COM9"  # replace with your port
BAUD_RATE = 115200
BUF_LEN = 300
SAMPLE_INTERVAL_MS = 30

# ---------------------------
# Sensor Data Class
# ---------------------------
class SensorData:
    def __init__(self):
        # Latest IMU values
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
        self.accel_buf = np.zeros((BUF_LEN, 3))
        self.gyro_buf = np.zeros((BUF_LEN, 3))
        self.mag_buf = np.zeros((BUF_LEN, 3))
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
                    sensor_data.buf_index = (idx + 1) % BUF_LEN

            # GPS data
            elif line.startswith("Time"):
                groups = [g.strip() for g in line.split(";")]
                with sensor_data.lock:
                    for g in groups:
                        if g.startswith("Time"):
                            sensor_data.time = ":".join(g.split(":")[1:]).strip()
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
        self.serial_thread_handle = None

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

        # 3D GL Widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=5)
        layout.insertWidget(0, self.gl_widget, 1)  # insert at top

        # Grid for reference
        grid = gl.GLGridItem()
        grid.scale(1,1,1)
        self.gl_widget.addItem(grid)

        # Box
        self.box = gl.GLBoxItem(size=QtGui.QVector3D(1,0.3,0.6), color=(0.4,0.6,1,1))
        self.gl_widget.addItem(self.box)

        # Right panel: Strip charts
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

        # GPS + IMU text
        self.info_text = QtWidgets.QTextEdit()
        self.info_text.setReadOnly(True)
        layout.addWidget(self.info_text)

        # Timer for GUI updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(SAMPLE_INTERVAL_MS)

    def start(self):
        if self.running_flag[0]:
            return
        self.running_flag[0] = True
        self.serial_thread_handle = threading.Thread(
            target=serial_thread,
            args=(PORT_NAME, BAUD_RATE, self.sensor_data, self.running_flag)
        )
        self.serial_thread_handle.start()

    def stop(self):
        self.running_flag[0] = False
        if self.serial_thread_handle:
            self.serial_thread_handle.join()
            self.serial_thread_handle = None

    def closeEvent(self, event):
        """Graceful shutdown on window close."""
        self.stop()
        event.accept()

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
            text = f"""Orientation (yaw,pitch,roll): {self.sensor_data.pitch:.2f}, {self.sensor_data.roll:.2f}, {self.sensor_data.yaw:.2f}
Gyro: {self.sensor_data.gyro[0]:.2f}, {self.sensor_data.gyro[1]:.2f}, {self.sensor_data.gyro[2]:.2f}
Accel: {self.sensor_data.accel[0]:.2f}, {self.sensor_data.accel[1]:.2f}, {self.sensor_data.accel[2]:.2f}
Mag: {self.sensor_data.mag[0]:.2f}, {self.sensor_data.mag[1]:.2f}, {self.sensor_data.mag[2]:.2f}
Time: {self.sensor_data.time} | Fix: {self.sensor_data.fix} | Satellites: {self.sensor_data.sat}
Lat: {self.sensor_data.lat} | Lon: {self.sensor_data.longi} | Elev: {self.sensor_data.elevation}
"""
            self.info_text.setPlainText(text)

            # Update box orientation
            pitch = self.sensor_data.pitch
            roll  = self.sensor_data.roll
            yaw   = self.sensor_data.yaw
            self.box.resetTransform()
            self.box.rotate(pitch, 1,0,0)
            self.box.rotate(yaw, 0,1,0)
            self.box.rotate(roll, 0,0,1)

# ---------------------------
# Pre-check serial before launching GUI
# ---------------------------
try:
    ser_test = serial.Serial(PORT_NAME, BAUD_RATE, timeout=1)
    ser_test.close()
except Exception as e:
    print(f"Error opening serial port {PORT_NAME}: {e}")
    sys.exit(1)

# ---------------------------
# Run GUI
# ---------------------------
app = QtWidgets.QApplication(sys.argv)
win = MainWindow()
win.show()
sys.exit(app.exec_())
