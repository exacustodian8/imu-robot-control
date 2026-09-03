import time
import serial
import numpy as np
import keyboard
from scipy.spatial.transform import Rotation as R
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from PyQt5.QtCore import QThread, pyqtSignal


class ProcessingWorker(QThread):
    data_updated = pyqtSignal(float, float, float, float, float, float, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True

        self.uc = None
        self.ur = None
        self.ser = None

        # translation control
        self.translation_gain = 0.0025   # m/s
        self.max_speed = 0.05            # m/s
        self.deadzone_deg = 5.0
        self.alpha = 0.2
        self.max_position_delta = 0.20   # meters from initial pose

        self.prev_time = time.time()

        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        self.base_pose = None

        # zero references
        self.zero_ori = None     # [pitch1, roll1, yaw1]
        self.zero_trans = None   # [pitch2, roll2, yaw2]

        self._z_was_pressed = False

    def low_pass_filter(self, old, new, alpha):
        return alpha * new + (1 - alpha) * old

    def apply_deadzone(self, value, dz):
        if abs(value) < dz:
            return 0.0
        return value

    def angle_diff_deg(self, current, zero):
        diff = current - zero
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def run(self):
        try:
            #Use "localhost" for the simulator
            #Use the robot's local IP address for a physical robot
            self.uc = RTDEControl("localhost")
            self.ur = RTDEReceive("localhost")
            self.ser = serial.Serial("COM5", 115200, timeout=1)
            self.ser.reset_input_buffer()
            time.sleep(2)

            if not self.uc.isConnected():
                print("robot not connected")
                return

            self.base_pose = np.array(self.ur.getActualTCPPose()[:3], dtype=float)
            print(f"Base pose locked: {self.base_pose}")

            while self._running:
                try:
                    if keyboard.is_pressed('q') or keyboard.is_pressed('esc'):
                        print("Stopping worker from keyboard...")
                        self.stop()
                        break

                    current_time = time.time()
                    dt = current_time - self.prev_time
                    self.prev_time = current_time

                    data = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    individual = data.split(',')

                    if len(individual) >= 6:
                        pitch1, roll1, yaw1, pitch2, roll2, yaw2 = map(float, individual[:6])

                        z_pressed = keyboard.is_pressed('z')
                        if z_pressed and not self._z_was_pressed:
                            self.zero_ori = np.array([pitch1, roll1, yaw1], dtype=float)
                            self.zero_trans = np.array([pitch2, roll2, yaw2], dtype=float)
                            self.position[:] = 0.0
                            self.velocity[:] = 0.0
                            print("Zeroed orientation and translation references")
                        self._z_was_pressed = z_pressed

                        if self.zero_ori is None:
                            self.zero_ori = np.array([pitch1, roll1, yaw1], dtype=float)
                        if self.zero_trans is None:
                            self.zero_trans = np.array([pitch2, roll2, yaw2], dtype=float)

                        # ORIENTATION
                        rel_pitch1 = self.angle_diff_deg(pitch1, self.zero_ori[0])
                        rel_roll1  = self.angle_diff_deg(roll1,  self.zero_ori[1])
                        rel_yaw1   = self.angle_diff_deg(yaw1,   self.zero_ori[2])

                        sensor_rot = R.from_euler(
                            'xyz',
                            [rel_pitch1, rel_roll1, rel_yaw1],
                            degrees=True
                        )

                        correction_rot = R.from_euler('y', -180, degrees=True)
                        corr_rot = R.from_euler('z', 90, degrees=True)
                        final_rot = sensor_rot * corr_rot * correction_rot
                        axis_angle = final_rot.as_rotvec()

                        #TRANSLATION
                        rel_pitch2 = self.angle_diff_deg(pitch2, self.zero_trans[0])
                        rel_roll2  = self.angle_diff_deg(roll2,  self.zero_trans[1])
                        rel_yaw2   = self.angle_diff_deg(yaw2,   self.zero_trans[2])

                        # deadzone
                        rel_pitch2 = self.apply_deadzone(rel_pitch2, self.deadzone_deg)
                        rel_roll2 = self.apply_deadzone(rel_roll2, self.deadzone_deg)

                        # map tilt to XY velocity
                        vx_cmd = np.clip(self.translation_gain * rel_pitch2, -self.max_speed, self.max_speed)
                        vy_cmd = np.clip(self.translation_gain * rel_roll2,  -self.max_speed, self.max_speed)
                        vz_cmd = 0.0

                        vel_cmd = np.array([vx_cmd, vy_cmd, vz_cmd], dtype=float)

                        # smooth velocity
                        self.velocity = self.low_pass_filter(self.velocity, vel_cmd, self.alpha)

                        # stop tiny residual motion
                        if np.linalg.norm(vel_cmd) < 1e-4:
                            self.velocity[:] = 0.0

                        # integrate to position offset
                        self.position += self.velocity * dt

                        # clamp workspace offset from locked base pose
                        self.position = np.clip(
                            self.position,
                            -self.max_position_delta,
                            self.max_position_delta
                        )

                        target_xyz = self.base_pose + self.position
                        target_pose = target_xyz.tolist() + axis_angle.tolist()

                        self.uc.servoL(target_pose, 0.1, 0.01, 0.05, 0.1, 300)

                        cmd_text = (
                            f"servoL([{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}, "
                            f"{target_pose[3]:.3f}, {target_pose[4]:.3f}, {target_pose[5]:.3f}])"
                        )

                        self.data_updated.emit(
                            rel_pitch1, rel_roll1, rel_yaw1, 
                            rel_pitch2, rel_roll2, rel_yaw2,  
                            cmd_text)

                    else:
                        print(f"bad data: {data}")

                    time.sleep(0.01)

                except Exception as e:
                    print(f"error in loop: {e}")

        except Exception as e:
            print(f"worker startup error: {e}")

        finally:
            self.cleanup()

    def stop(self):
        self._running = False

    def cleanup(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            print(f"serial close error: {e}")

        try:
            if self.uc is not None:
                self.uc.speedStop()
        except Exception as e:
            print(f"robot stop error: {e}")
