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

"""Grid-based world photography controller for Mavic drone.
   Systematically covers the entire 400x400m world using a lawn-mower pattern
   and takes photos at regular intervals for complete coverage."""

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


class GridWorldPhotographyDrone(Robot):
    # Constants from original patrol example
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 10.0  # Larger precision for faster grid traversal

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # World coverage parameters
        self.world_size = 400  # 400x400 meter world from mavic_2_pro.wbt
        self.grid_spacing = 50  # Distance between flight lines (adjust for coverage density)
        self.target_altitude = 60.0  # Higher altitude for better coverage
        
        # Image capture settings
        self.last_image_time = 0
        self.image_interval_seconds = 3.0  # Take photo every 3 seconds
        self.images_dir = "grid_world_images"
        os.makedirs(self.images_dir, exist_ok=True)
        
        # Flight state tracking
        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.waypoints_initialized = False
        self.mission_complete = False
        
        # Generate grid waypoints
        self.waypoints = self._generate_grid_waypoints()
        
        # Initialize devices
        self._initialize_devices()
        
        print(f"🗺️ Grid World Photography Mission Initialized")
        print(f"📏 World size: {self.world_size}x{self.world_size}m")
        print(f"📐 Grid spacing: {self.grid_spacing}m")
        print(f"✈️ Flight altitude: {self.target_altitude}m")
        print(f"📍 Total waypoints: {len(self.waypoints)}")
        print(f"📸 Photo interval: {self.image_interval_seconds}s")

    def _generate_grid_waypoints(self):
        """Generate a systematic lawn-mower pattern to cover the entire world."""
        waypoints = []
        
        # Calculate grid boundaries (centered on world)
        start_x = -self.world_size // 2 + 20  # Leave 20m margin
        end_x = self.world_size // 2 - 20
        start_y = -self.world_size // 2 + 20
        end_y = self.world_size // 2 - 20
        
        y = start_y
        going_right = True
        
        while y <= end_y:
            if going_right:
                # Go from left to right
                waypoints.append([start_x, y])
                waypoints.append([end_x, y])
            else:
                # Go from right to left
                waypoints.append([end_x, y])
                waypoints.append([start_x, y])
            
            y += self.grid_spacing
            going_right = not going_right
        
        print(f"📋 Generated {len(waypoints)} waypoints for systematic coverage")
        return waypoints

    def _initialize_devices(self):
        """Initialize all drone devices."""
        # Camera with high FOV for better coverage
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        
        try:
            self.camera.setFov(1.8)  # Wide field of view for better coverage
            print("📷 Camera FOV set to wide angle")
        except:
            print("📷 Using default camera FOV")
        
        # Navigation sensors
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Motor controls
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        # Camera positioning - look straight down for mapping
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(1.5)  # Look straight down
        
        # Initialize motors
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def set_position(self, pos):
        """Set the new absolute position of the robot."""
        self.current_pose = pos

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """Move the robot through the grid waypoints systematically."""
        
        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print(f"🎯 First target: {self.target_position[0:2]}")

        # Check if we've reached current target
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            self.target_index += 1
            
            if self.target_index >= len(waypoints):
                if not self.mission_complete:
                    print("🎉 GRID PHOTOGRAPHY MISSION COMPLETE!")
                    print(f"📸 Check the '{self.images_dir}' directory for all captured images")
                    self.mission_complete = True
                # Keep flying to last waypoint
                self.target_index = len(waypoints) - 1
            
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                progress = (self.target_index / len(waypoints)) * 100
                print(f"✅ Target reached! Progress: {progress:.1f}% - New target: {self.target_position[0:2]}")

        # Calculate heading to target
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], 
            self.target_position[0] - self.current_pose[0])
        
        # Calculate turn angle
        angle_left = self.target_position[2] - self.current_pose[5]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Generate control inputs
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + 
                                  ((self.target_position[1] - self.current_pose[1]) ** 2))
            print(f"📐 Remaining angle: {angle_left:.4f}, distance: {distance_left:.1f}m")
        
        return yaw_disturbance, pitch_disturbance

    def save_camera_image(self):
        """Save camera image with GPS coordinates and grid progress."""
        try:
            image = self.camera.getImage()
            if image:
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8)
                image_array = image_array.reshape((height, width, 4))
                
                rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                
                x, y, z = self.current_pose[0], self.current_pose[1], self.current_pose[2]
                progress = (self.target_index / len(self.waypoints)) * 100
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"{self.images_dir}/grid_{self.target_index:03d}_{timestamp}_x{x:.0f}_y{y:.0f}_alt{z:.0f}.jpg"
                
                cv2.imwrite(filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                print(f"📸 [{progress:.1f}%] Saved: {filename}")
        except Exception as e:
            print(f"❌ Error saving image: {e}")

    def run(self):
        """Main flight control loop."""
        t1 = self.getTime()
        
        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        print("🚁 Starting Grid World Photography Mission...")
        
        while self.step(self.time_step) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # Take photos at regular intervals once at altitude
            current_time = self.getTime()
            if altitude > self.target_altitude - 5 and current_time - self.last_image_time >= self.image_interval_seconds:
                self.save_camera_image()
                self.last_image_time = current_time

            # Start navigation once at target altitude
            if altitude > self.target_altitude - 2:
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        self.waypoints, verbose_target=True)
                    t1 = self.getTime()

            # Flight control (same as original patrol)
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            # Motor control
            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# Initialize and run the grid photography drone
robot = GridWorldPhotographyDrone()
robot.run()

