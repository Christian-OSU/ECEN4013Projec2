import sys
import serial
from PyQt6.QtGui import QPainter, QPen, QTransform
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer, Qt

class IMUBoxWidget(QWidget):
    def __init__(self): 
        super().__init__()
        self.angle = 0  # Rotation angle in degrees

    def set_angle(self, angle):
        self.angle = angle
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(Qt.GlobalColor.black, 2))

        # Center transform
        transform = QTransform()
        center_x = self.width() / 2
        center_y = self.height() / 2
        transform.translate(center_x, center_y)
        transform.rotate(self.angle)
        painter.setTransform(transform)

        # Draw box centered at origin
        box_size = 100
        painter.drawRect(int(-box_size/2), int(-box_size/2), int(box_size), int(box_size))        
class SerialIMUReader(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Orientation Viewer")

        # UI setup
        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for IMU data...")
        self.box_widget = IMUBoxWidget()
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.box_widget)
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

                            # Example: use X to rotate the box
                            self.box_widget.set_angle(x)
            except Exception as e:
                self.label.setText(f"Read error: {e}")

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialIMUReader()
    window.show()
    sys.exit(app.exec())