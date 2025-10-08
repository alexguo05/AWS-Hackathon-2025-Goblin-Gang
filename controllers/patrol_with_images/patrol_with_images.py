# Copyright 1996-2024 Cyberbotics Ltd.
# ... (license header unchanged)

"""Mavic drone flying in a circle around a target point with collision diagnostics."""

from controller import Robot
import sys
import os
import cv2
from datetime import datetime
import math
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class CirclePatrolDrone(Robot):
    # Constants, empirically found (from official example)
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 5.0  # meters

    # ---- Geofence (edit these to match your world) ----
    # If you suspect a world “border”, set a safe rectangle here.
    GEOFENCE = {
        "xmin": -120.0, "xmax": 120.0,
        "ymin": -120.0, "ymax": 120.0,
        "margin": 2.0  # warn before actually crossing
    }

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # Mission parameters
        self.target_center = [-50.35, 11.25]
        self.circle_radius = 30.0
        self.target_altitude = 20.0

        # Waypoints
        self.num_waypoints = 8
        self.waypoints = None

        # Image capture
        self.last_image_time = 0
        self.image_interval_seconds = 2.0
        self.images_dir = "circle_patrol_images"
        os.makedirs(self.images_dir, exist_ok=True)

        # State
        self.current_pose = 6 * [0]  # X,Y,Z,yaw,pitch,roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.waypoints_initialized = False
        self.first_waypoint_reached = False

        # Collision/Proximity state
        self.touch = None
        self.distance_sensors = []   # list of dicts: {"name": str, "dev": DistanceSensor/RangeFinder, "kind": "ds"|"rf"}
        self._last_collision_print = 0.0
        self._last_proximity_print = 0.0
        self._last_geofence_print = 0.0
        self._print_cooldown = 0.5  # seconds

        self.prev_altitude = None
        self.prev_time = None


        # Initialize devices
        self._initialize_devices()

    def _check_ground_touch(self, altitude, now):
        """
        Heuristic: flag when we hit the ground (or very low objects)
        using altitude and vertical speed.
        """
        touched = False
        if self.prev_altitude is not None and self.prev_time is not None:
            dt = max(1e-6, now - self.prev_time)
            vz = (altitude - self.prev_altitude) / dt  # m/s
            # If we're at/near ground and vertical speed trends into it, flag it.
            if altitude < 0.15 and vz <= 0.05:
                print("💥 Ground contact likely (alt≈{:.2f} m, vz≈{:.2f} m/s)".format(altitude, vz))
                touched = True
        self.prev_altitude = altitude
        self.prev_time = now
        return touched


    def _generate_circle_waypoints(self, start_x, start_y):
        all_waypoints = []
        for i in range(self.num_waypoints):
            angle = 2 * math.pi * i / self.num_waypoints
            x = self.target_center[0] + self.circle_radius * math.cos(angle)
            y = self.target_center[1] + self.circle_radius * math.sin(angle)
            all_waypoints.append([x, y, angle])

        min_distance = float('inf')
        closest_index = 0
        for i, waypoint in enumerate(all_waypoints):
            distance = math.sqrt((waypoint[0] - start_x)**2 + (waypoint[1] - start_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        ordered_waypoints = []
        for i in range(self.num_waypoints):
            index = (closest_index + i) % self.num_waypoints
            ordered_waypoints.append([all_waypoints[index][0], all_waypoints[index][1]])

        print("🎯 Starting from waypoint closest to drone position")
        print(f"📍 Starting waypoint: ({ordered_waypoints[0][0]:.1f}, {ordered_waypoints[0][1]:.1f})")
        return ordered_waypoints

    def _maybe_add_distance_sensor(self, name, friendly, kind="ds"):
        try:
            dev = self.getDevice(name)
            dev.enable(self.time_step)
            self.distance_sensors.append({"name": friendly, "dev": dev, "kind": kind})
            print(f"🧭 Prox sensor enabled: {friendly}")
        except Exception:
            pass

    def _initialize_devices(self):
        """Initialize all devices (camera, IMU, motors, and optional collision sensors)."""
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor  = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor   = self.getDevice("rear left propeller")
        self.rear_right_motor  = self.getDevice("rear right propeller")

        # Camera gimbal
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.5)
        try:
            self.camera_yaw_motor = self.getDevice("camera yaw")
            self.camera_yaw_motor.setPosition(1.3)
            print("📷 Camera set to face 90° left")
        except Exception:
            print("📷 Camera yaw motor not available, camera faces forward")

        for m in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
            m.setPosition(float('inf'))
            m.setVelocity(1)

        # --- Optional collision sensing ---
        # 1) Touch sensor (bumper). Add in .wbt: TouchSensor { name "bumper" type "bumper" } on the drone body.
        try:
            self.touch = self.getDevice("bumper")  # name must match your world
            self.touch.enable(self.time_step)
            print("🛡️ Touch sensor (bumper) enabled")
        except Exception:
            self.touch = None

        # 2) Distance sensors (common names—add yours if different)
        # If your PROTO exposes RangeFinder, you can also add it as a proximity cue.
        # Try a few typical names:
        for name, friendly in [
            ("front distance sensor", "front"),
            ("left distance sensor",  "left"),
            ("right distance sensor", "right"),
            ("back distance sensor",  "rear"),
            ("down sensor",           "down"),
            ("up distance sensor",    "up"),
            ("ds_front",              "front"),
            ("ds_left",               "left"),
            ("ds_right",              "right"),
            ("ds_rear",               "rear"),
        ]:
            self._maybe_add_distance_sensor(name, friendly, kind="ds")

        # Try a RangeFinder (e.g., "range finder" typically points down)
        try:
            rf = self.getDevice("range finder")
            rf.enable(self.time_step)
            self.distance_sensors.append({"name": "down-range", "dev": rf, "kind": "rf"})
            print("🧭 RangeFinder enabled: down-range")
        except Exception:
            pass
        try:
            self.touch = self.getDevice("bumper")
            self.touch.enable(self.time_step)
            print("🛡️ Touch sensor enabled (bumper)")
        except Exception:
            self.touch = None
            print("🛡️ No touch sensor found; add TouchSensor { name \"bumper\" type \"bumper\" } to the drone for hard collision events.")


    def set_position(self, pos):
        self.current_pose = pos

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        if self.target_position[0:2] == [0, 0]:
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            if not self.first_waypoint_reached:
                self.first_waypoint_reached = True
                print("✅ First waypoint reached! Starting image capture...")
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
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) +
                                    ((self.target_position[1] - self.current_pose[1]) ** 2))
            print(f"remaining angle: {angle_left:.4f}, remaining distance: {distance_left:.4f}")
        return yaw_disturbance, pitch_disturbance

    # -------- Collision / proximity checks --------
    def _print_throttled(self, kind, msg):
        now = self.getTime()
        last_attr = f"_last_{kind}_print"
        last = getattr(self, last_attr)
        if now - last >= self._print_cooldown:
            print(msg)
            setattr(self, last_attr, now)

    def _check_touch_collision(self):
        if not hasattr(self, "touch") or self.touch is None:
            return False
        try:
            if self.touch.getValue() > 0.0:
                print("💥 Collision detected (touch/bumper)")
                return True
        except:
            pass
        return False


    def _check_proximity(self):
        """Warn if any distance sensor reports a very close obstacle.
        Returns True if any sensor is below threshold."""
        hit = False
        for entry in self.distance_sensors:
            dev = entry["dev"]
            name = entry["name"]
            try:
                if entry["kind"] == "rf":
                    # RangeFinder: use a rough threshold on range image center depth
                    width = dev.getWidth()
                    height = dev.getHeight()
                    img = dev.getRangeImage()
                    if img:
                        # depth at center pixel
                        center_depth = img[(height // 2) * width + (width // 2)]
                        if center_depth is not None and center_depth > 0 and center_depth < 0.6:
                            self._print_throttled("proximity", f"⚠️ Obstacle ~{center_depth:.2f} m ({name})")
                            hit = True
                else:
                    # DistanceSensor: best-effort—compare absolute value (meters for many sensors)
                    val = dev.getValue()
                    maxv = getattr(dev, "getMaxValue", lambda: None)()
                    minv = getattr(dev, "getMinValue", lambda: None)()
                    # Heuristic: if units are meters, value will be in (minv, maxv] ~ meters.
                    # Use 0.6 m front/side/up; 0.3 m down to avoid false positives from ground.
                    thresh = 0.6 if "down" not in name else 0.3
                    if val is not None and val > 0 and val < thresh:
                        self._print_throttled("proximity", f"⚠️ Obstacle ~{val:.2f} m ({name})")
                        hit = True
            except Exception:
                continue
        return hit

    def _check_geofence(self):
        x, y = self.current_pose[0], self.current_pose[1]
        g = self.GEOFENCE
        near = False
        crossed = False
        # Near boundary warnings
        if (x < g["xmin"] + g["margin"]) or (x > g["xmax"] - g["margin"]) or \
           (y < g["ymin"] + g["margin"]) or (y > g["ymax"] - g["margin"]):
            self._print_throttled("geofence",
                                  f"🚧 Near geofence: x={x:.1f} (min {g['xmin']}, max {g['xmax']}), "
                                  f"y={y:.1f} (min {g['ymin']}, max {g['ymax']})")
            near = True
        # Crossed boundary
        if x < g["xmin"] or x > g["xmax"] or y < g["ymin"] or y > g["ymax"]:
            self._print_throttled("geofence", "🧱 Geofence boundary crossed")
            crossed = True
        return near or crossed

    def save_camera_image(self):
        try:
            image = self.camera.getImage()
            if image:
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
                rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                x, y, z = self.current_pose[0], self.current_pose[1], self.current_pose[2]
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"{self.images_dir}/circle_patrol_{timestamp}_x{x:.1f}_y{y:.1f}_z{z:.1f}.jpg"
                cv2.imwrite(filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                print(f"📸 Saved: {filename}")
        except Exception as e:
            print(f"❌ Error saving image: {e}")

    def run(self):
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        print("🚁 Starting circle patrol mission...")
        print(f"🎯 Target center: {self.target_center}")
        print(f"📏 Circle radius: {self.circle_radius}m")
        print(f"📏 Flight altitude: {self.target_altitude}m")
        print(f"📍 Will generate {self.num_waypoints} waypoints after takeoff")
        print("🔄 Flight direction: Counterclockwise from closest point")

        while self.step(self.time_step) != -1:
            # Sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # --- Collision & boundary checks ---
            collided = self._check_touch_collision()
            near_obstacle = self._check_proximity()
            geofence_event = self._check_geofence()

            # Optional: if you want an automatic reaction, uncomment to gently stop/ascend
            # if collided or near_obstacle or geofence_event:
            #     # soften horizontal motion
            #     yaw_disturbance = 0.0
            #     pitch_disturbance = 0.0
            #     # nudge up a bit
            #     self.target_altitude = max(self.target_altitude, altitude + 0.3)

            # Imaging
            current_time = self.getTime()
            if self.first_waypoint_reached and current_time - self.last_image_time >= self.image_interval_seconds:
                self.save_camera_image()
                self.last_image_time = current_time

            # Waypoint init & guidance
            if altitude > self.target_altitude - 1:
                if not self.waypoints_initialized:
                    self.waypoints = self._generate_circle_waypoints(x_pos, y_pos)
                    self.waypoints_initialized = True
                    print(f"✅ Waypoints initialized from drone position ({x_pos:.1f}, {y_pos:.1f})")
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(self.waypoints, verbose_target=True)
                    t1 = self.getTime()

            # Motors
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(fl)
            self.front_right_motor.setVelocity(-fr)
            self.rear_left_motor.setVelocity(-rl)
            self.rear_right_motor.setVelocity(rr)
            # --- Boundary & contact diagnostics (no extra devices needed) ---
            self._check_geofence()
            self._check_ground_touch(altitude, self.getTime())
            self._check_touch_collision()




# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5

robot = CirclePatrolDrone()
robot.run()
