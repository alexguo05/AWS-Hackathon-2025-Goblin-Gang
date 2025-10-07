# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Mavic drone flying in a circle around a target point.
   Based on official Webots Mavic patrol example."""

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
    target_precision = 5.0  # Precision in meters (larger = faster, moves to next waypoint sooner)

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # Mission parameters
        self.target_center = [-50.35, 11.25]  # Center of circle
        self.circle_radius = 30.0  # Radius in meters
        self.target_altitude = 20.0  # Flight altitude
        
        # Generate circle waypoints
        self.num_waypoints = 8  # Fewer waypoints = faster flight (was 36)
        self.waypoints = None  # Will be generated after we know starting position
        
        # Image capture
        self.last_image_time = 0
        self.image_interval_seconds = 2.0
        self.images_dir = "circle_patrol_images"
        os.makedirs(self.images_dir, exist_ok=True)

        # State tracking (SAME AS ORIGINAL EXAMPLE)
        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.waypoints_initialized = False
        self.first_waypoint_reached = False  # Track when to start taking pictures

        # Initialize devices
        self._initialize_devices()

    def _generate_circle_waypoints(self, start_x, start_y):
        """Generate waypoints in a circle around the target center.
        Starts from the waypoint closest to the drone's current position,
        then goes counterclockwise.
        
        Args:
            start_x: Current X position of drone
            start_y: Current Y position of drone
        """
        # Generate all possible waypoints around the circle
        all_waypoints = []
        for i in range(self.num_waypoints):
            angle = 2 * math.pi * i / self.num_waypoints
            x = self.target_center[0] + self.circle_radius * math.cos(angle)
            y = self.target_center[1] + self.circle_radius * math.sin(angle)
            all_waypoints.append([x, y, angle])  # Store angle too
        
        # Find the closest waypoint to the drone's current position
        min_distance = float('inf')
        closest_index = 0
        for i, waypoint in enumerate(all_waypoints):
            distance = math.sqrt((waypoint[0] - start_x)**2 + (waypoint[1] - start_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Reorder waypoints to start from closest and go counterclockwise
        ordered_waypoints = []
        for i in range(self.num_waypoints):
            index = (closest_index + i) % self.num_waypoints
            ordered_waypoints.append([all_waypoints[index][0], all_waypoints[index][1]])
        
        print(f"🎯 Starting from waypoint closest to drone position")
        print(f"📍 Starting waypoint: ({ordered_waypoints[0][0]:.1f}, {ordered_waypoints[0][1]:.1f})")
        
        return ordered_waypoints

    def _initialize_devices(self):
        """Initialize all drone devices."""
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        
        try:
            self.camera.setFov(1.5)
            print("📷 Camera FOV increased")
        except:
            print("📷 Using default camera FOV")
        
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        # Camera setup - pitch down and yaw 90 degrees left
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.5)  # Look down more (higher value = more downward)
        
        # Try to get camera yaw motor if available
        try:
            self.camera_yaw_motor = self.getDevice("camera yaw")
            # Set to 90 degrees left (pi/2 radians)
            self.camera_yaw_motor.setPosition(1.3)  # 90 degrees = π/2 radians
            print("📷 Camera set to face 90° left")
        except:
            print("📷 Camera yaw motor not available, camera faces forward")
        
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def set_position(self, pos):
        """Set the new absolute position of the robot.
        EXACTLY AS IN ORIGINAL EXAMPLE.
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """Move the robot to the given coordinates.
        EXACTLY AS IN ORIGINAL EXAMPLE.
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaining angle and distance
            verbose_target (bool): whether to print targets
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            # Mark first waypoint as reached
            if not self.first_waypoint_reached:
                self.first_waypoint_reached = True
                print("✅ First waypoint reached! Starting image capture...")
            
            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ", self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], 
            self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaining angle: {:.4f}, remaining distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def save_camera_image(self):
        """Save camera image with GPS coordinates."""
        try:
            image = self.camera.getImage()
            if image:
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8)
                image_array = image_array.reshape((height, width, 4))
                
                rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                
                x, y, z = self.current_pose[0], self.current_pose[1], self.current_pose[2]
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"{self.images_dir}/circle_patrol_{timestamp}_x{x:.1f}_y{y:.1f}_z{z:.1f}.jpg"
                
                cv2.imwrite(filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                print(f"📸 Saved: {filename}")
        except Exception as e:
            print(f"❌ Error saving image: {e}")

    def run(self):
        """Main flight control loop.
        SAME STRUCTURE AS ORIGINAL EXAMPLE."""
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        print("🚁 Starting circle patrol mission...")
        print(f"🎯 Target center: {self.target_center}")
        print(f"📏 Circle radius: {self.circle_radius}m")
        print(f"📏 Flight altitude: {self.target_altitude}m")
        print(f"📍 Will generate {self.num_waypoints} waypoints after takeoff")
        print(f"🔄 Flight direction: Counterclockwise from closest point")

        while self.step(self.time_step) != -1:

            # Read sensors (EXACTLY AS ORIGINAL)
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # Save images periodically (ONLY after first waypoint is reached)
            current_time = self.getTime()
            if self.first_waypoint_reached and current_time - self.last_image_time >= self.image_interval_seconds:
                self.save_camera_image()
                self.last_image_time = current_time

            if altitude > self.target_altitude - 1:
                # Initialize waypoints on first time reaching altitude
                if not self.waypoints_initialized:
                    self.waypoints = self._generate_circle_waypoints(x_pos, y_pos)
                    self.waypoints_initialized = True
                    print(f"✅ Waypoints initialized from drone position ({x_pos:.1f}, {y_pos:.1f})")
                
                # as soon as it reach the target altitude, compute the disturbances to go to the waypoints
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        self.waypoints, verbose_target=True)
                    t1 = self.getTime()

            # Motor control (EXACTLY AS ORIGINAL)
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5

robot = CirclePatrolDrone()
robot.run()