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

"""Enhanced Python controller for Mavic patrolling with image extraction.
   This controller saves camera images during flight with GPS coordinates.
   Images are saved to 'drone_images' directory with timestamps and location data."""

from controller import Robot
import sys
import os
import cv2
from datetime import datetime
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicWithImages (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
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
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        
        # Image capture settings
        self.image_counter = 0
        self.image_save_interval = 20  # Save image every 20 steps (adjust as needed)
        self.images_dir = "drone_images"
        os.makedirs(self.images_dir, exist_ok=True)
        
        # Data logging
        self.flight_data = []
        self.start_time = self.getTime()

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def save_camera_image(self):
        """Save the current camera image to disk with GPS coordinates."""
        try:
            # Get camera image
            image = self.camera.getImage()
            if image:
                # Convert to numpy array
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8)
                image_array = image_array.reshape((height, width, 4))  # RGBA
                
                # Convert RGBA to RGB
                rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                
                # Get current GPS coordinates
                x, y, z = self.gps.getValues()
                roll, pitch, yaw = self.imu.getRollPitchYaw()
                
                # Generate filename with timestamp and GPS coordinates
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"{self.images_dir}/drone_image_{timestamp}_x{x:.1f}_y{y:.1f}_z{z:.1f}.jpg"
                
                # Save image
                cv2.imwrite(filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                print(f"📸 Saved image: {filename}")
                
                # Log flight data
                self.log_flight_data(x, y, z, roll, pitch, yaw, filename)
                
        except Exception as e:
            print(f"❌ Error saving image: {e}")

    def log_flight_data(self, x, y, z, roll, pitch, yaw, image_filename):
        """Log flight data with image reference."""
        flight_data_point = {
            'timestamp': self.getTime() - self.start_time,
            'gps_x': x,
            'gps_y': y,
            'gps_z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'image_filename': image_filename
        }
        self.flight_data.append(flight_data_point)

    def save_flight_data(self):
        """Save flight data to CSV file."""
        try:
            import pandas as pd
            df = pd.DataFrame(self.flight_data)
            csv_filename = f"{self.images_dir}/flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            df.to_csv(csv_filename, index=False)
            print(f"📊 Flight data saved: {csv_filename}")
        except ImportError:
            # Fallback to basic CSV if pandas not available
            csv_filename = f"{self.images_dir}/flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(csv_filename, 'w') as f:
                f.write("timestamp,gps_x,gps_y,gps_z,roll,pitch,yaw,image_filename\n")
                for data in self.flight_data:
                    f.write(f"{data['timestamp']},{data['gps_x']},{data['gps_y']},{data['gps_z']},"
                           f"{data['roll']},{data['pitch']},{data['yaw']},{data['image_filename']}\n")
            print(f"📊 Flight data saved: {csv_filename}")

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
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

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
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
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def run(self):
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Specify the patrol coordinates
        waypoints = [[-30, 20], [-60, 20], [-60, 10], [-30, 5]]
        # target altitude of the robot in meters
        self.target_altitude = 15

        print("🚁 Starting drone patrol with image capture...")
        print(f"📁 Images will be saved to: {self.images_dir}/")
        print(f"🎯 Patrol waypoints: {waypoints}")

        while self.step(self.time_step) != -1:

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # Save camera image periodically
            if self.image_counter % self.image_save_interval == 0:
                self.save_camera_image()

            self.image_counter += 1

            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        waypoints)
                    t1 = self.getTime()

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

        # Save flight data when simulation ends
        print("💾 Saving flight data...")
        self.save_flight_data()
        print(f"✅ Simulation complete! Check {self.images_dir}/ for images and data.")


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5

robot = MavicWithImages()
robot.run()
