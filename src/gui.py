from collections import deque
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
import pyqtgraph as pg

from worker import ProcessingWorker

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IMU + Robot Monitor")
        self.resize(1000,900)

        self.max_points = 200
        self.sample_index = 0

        self.x_data = deque(maxlen = self.max_points)
        self.pitch_data = deque(maxlen = self.max_points)
        self.roll_data = deque(maxlen = self.max_points)
        self.yaw_data = deque(maxlen = self.max_points)
        
        self.x_data2 = deque(maxlen = self.max_points)
        self.pitch2_data = deque(maxlen = self.max_points)
        self.roll2_data = deque(maxlen = self.max_points)
        self.yaw2_data = deque(maxlen = self.max_points)

        self.pitch_label = QLabel("Pitch: 0.00")
        self.roll_label = QLabel("Roll: 0.00")
        self.yaw_label = QLabel("Yaw: 0.00")

        self.pitch2_label = QLabel("Pitch: 0.00")
        self.roll2_label = QLabel("Roll: 0.00")
        self.yaw2_label = QLabel("Yaw: 0.00")

        self.command_label = QLabel("Command: -")

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")
        self.plot_widget.setTitle("Sensor 1 Orientation")
        self.plot_widget.setLabel("left", "Angle", units = "deg")
        self.plot_widget.setLabel("bottom", "Samples")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x = True, y = True)

        self.pitch_curve = self.plot_widget.plot(
            pen = pg.mkPen(color = "r", width = 2),name = "Pitch1"
        )
        self.roll_curve = self.plot_widget.plot(
            pen = pg.mkPen(color = "g", width = 2),name = "Roll1"
        )
        self.yaw_curve = self.plot_widget.plot(
            pen = pg.mkPen(color = "b", width = 2),name = "Yaw1"
        )

        self.plot_widget2 = pg.PlotWidget()
        self.plot_widget2.setBackground("w")
        self.plot_widget2.setTitle("Sensor 2 Translation")
        self.plot_widget2.setLabel("left", "Angle", units = "deg")
        self.plot_widget2.setLabel("bottom", "Samples")
        self.plot_widget2.addLegend()
        self.plot_widget2.showGrid(x = True, y = True)

        self.pitch2_curve = self.plot_widget2.plot(
            pen = pg.mkPen(color = "m", width = 2),name = "Pitch2"
        )
        self.roll2_curve = self.plot_widget2.plot(
            pen = pg.mkPen(color = "c", width = 2),name = "Roll2"
        )
        self.yaw2_curve = self.plot_widget2.plot(
            pen = pg.mkPen(color = "k", width = 2),name = "Yaw2"
        )
        
        layout = QVBoxLayout()
        layout.addWidget(self.pitch_label)
        layout.addWidget(self.roll_label)
        layout.addWidget(self.yaw_label)

        layout.addWidget(self.pitch2_label)
        layout.addWidget(self.roll2_label)
        layout.addWidget(self.yaw2_label)

        layout.addWidget(self.command_label)
        layout.addWidget(self.plot_widget)
        layout.addWidget(self.plot_widget2)

        self.setLayout(layout)

        self.worker = ProcessingWorker()
        self.worker.data_updated.connect(self.update_display)
        self.worker.start()

    def update_display(self, pitch1, roll1, yaw1, pitch2, roll2, yaw2, command):
        self.pitch_label.setText(f"Pitch1:{pitch1:.2f}")
        self.roll_label.setText(f"Roll1:{roll1:.2f}")
        self.yaw_label.setText(f"Yaw1:{yaw1:.2f}")

        self.pitch2_label.setText(f"Pitch2:{pitch2:.2f}")
        self.roll2_label.setText(f"Roll2:{roll2:.2f}")
        self.yaw2_label.setText(f"Yaw2:{yaw2:.2f}")

        self.command_label.setText(f"Command:{command}")

        self.x_data.append(self.sample_index)
        self.pitch_data.append(pitch1)
        self.roll_data.append(roll1)
        self.yaw_data.append(yaw1)

        self.x_data2.append(self.sample_index)
        self.pitch2_data.append(pitch2)
        self.roll2_data.append(roll2)
        self.yaw2_data.append(yaw2)

        self.sample_index += 1

        self.pitch_curve.setData(list(self.x_data),list(self.pitch_data))
        self.roll_curve.setData(list(self.x_data), list(self.roll_data))
        self.yaw_curve.setData(list(self.x_data), list(self.yaw_data))

        self.pitch2_curve.setData(list(self.x_data2),list(self.pitch2_data))
        self.roll2_curve.setData(list(self.x_data2),list(self.roll2_data))
        self.yaw2_curve.setData(list(self.x_data2),list(self.yaw2_data))


    def closeEvent(self,event):
        self.worker.stop()
        self.worker.wait()
        event.accept()
        
