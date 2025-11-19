import sys
import serial
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer

class SerialIMUReader(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Serial Monitor")

        # UI setup
        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for IMU data...")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # Serial setup
        try:
            self.ser = serial.Serial(port='COM9', baudrate=9600, timeout=1)
        except serial.SerialException as e:
            self.label.setText(f"Serial error: {e}")
            self.ser = None

        # Timer setup
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(10)  # Poll every 100 ms

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Recieved: {line}")
                    self.label.setText(f"Received: {line}")
            except Exception as e:
                self.label.setText(f"Read error: {e}")

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

# Main app
app = QApplication(sys.argv)
window = SerialIMUReader()
window.show()
app.exec()