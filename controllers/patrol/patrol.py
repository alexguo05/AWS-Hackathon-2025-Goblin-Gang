# controllers/patrol/patrol.py
# (same copyright / header)

from controller import Robot
import sys, json  # <-- NEW: json for compact messages
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class Mavic (Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 0.5

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # Sensors
        self.camera = self.getDevice("camera"); self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit"); self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps"); self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro"); self.gyro.enable(self.time_step)

        # Motors
        self.front_left_motor  = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor   = self.getDevice("rear left propeller")
        self.rear_right_motor  = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch"); self.camera_pitch_motor.setPosition(0.5)
        for m in (self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor):
            m.setPosition(float('inf')); m.setVelocity(1)

        # NEW: radio emitter (optional but recommended)
        self.emitter = self.getDevice("emitter")
        if self.emitter:
            # (Optional) set radio params here; must match your world if set there too
            try:
                self.emitter.setChannel(1)
                self.emitter.setRange(-1)     # unlimited for easy testing; set to e.g. 60.0 later
                self.emitter.setBaudRate(9600)
            except Exception:
                pass
            self._tx_every_steps = max(1, int(0.2 * 1000.0 / self.time_step))  # ~5 Hz
            self._tx_tick = 0
            self._id = self.getName()
        else:
            print("[patrol] WARNING: No 'emitter' device on this robot; position will not be broadcast.")

        self.current_pose = 6 * [0]  # X, Y, Z, roll, pitch, yaw
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        
        print(f"FOV is {self.camera.getFov()} while max FOV is {self.camera.getMaxFov()}")

    # NEW: tiny helper to transmit pose
    def _broadcast_pose(self, x, y, z, yaw):
        if not self.emitter: 
            return
        payload = {
            "type": "pose",
            "id": self._id,
            "p": [x, y, z, yaw],
            "t": self.getTime()
        }
        # keep it small—compact JSON
        self.emitter.send(json.dumps(payload, separators=(',', ':')).encode('utf-8'))

    def set_position(self, pos):
        self.current_pose = pos

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        if self.target_position[0:2] == [0, 0]:
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            self.target_index = (self.target_index + 1) % len(waypoints)
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ", self.target_position[0:2])

        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1],
            self.target_position[0] - self.current_pose[0]
        )
        angle_left = self.target_position[2] - self.current_pose[5]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.hypot(self.target_position[0] - self.current_pose[0],
                                     self.target_position[1] - self.current_pose[1])
            print(f"remaning angle: {angle_left:.4f}, remaning distance: {distance_left:.4f}")
        return yaw_disturbance, pitch_disturbance

    def run(self):
        t1 = self.getTime()
        roll_disturbance = pitch_disturbance = yaw_disturbance = 0

        waypoints = [[-30, 20], [-60, 20], [-60, 10], [-30, 5]]
        self.target_altitude = 15

        while self.step(self.time_step) != -1:
            # Sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_accel, pitch_accel, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # NEW: periodic broadcast (independent of motion)
            if self.emitter:
                self._tx_tick += 1
                if self._tx_tick >= self._tx_every_steps:
                    self._tx_tick = 0
                    self._broadcast_pose(x_pos, y_pos, altitude, yaw)

            if altitude > self.target_altitude - 1:
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
                    t1 = self.getTime()

            # Control
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_accel + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_accel + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_diff_alt = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * (clamped_diff_alt ** 3.0)

            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(fl)
            self.front_right_motor.setVelocity(-fr)
            self.rear_left_motor.setVelocity(-rl)
            self.rear_right_motor.setVelocity(rr)

print("Simulation running")
robot = Mavic()
robot.run()
