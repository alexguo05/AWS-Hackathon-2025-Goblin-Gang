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

"""Task-based Circle Patrol Drone Controller
   Executes circle patrol missions assigned by supervisor via JSON tasks.
   Based on official Webots Mavic patrol example."""

from controller import Robot
import sys
import os
import json
import time
import cv2
from datetime import datetime
import math
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class TaskBasedCirclePatrolDrone(Robot):
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

        # Extract drone ID from name (e.g., "drone_0" -> 0)
        robot_name = self.getName()
        try:
            self.drone_id = int(robot_name.split('_')[-1])
        except:
            self.drone_id = 0

        # Mission parameters
        self.target_altitude = 40.0  # Flight altitude
        self.num_waypoints = 8  # Number of waypoints per circle (should match supervisor)
        
        # Task management (NEW - centralized circle-based task system)
        self.current_task = None  # Current circle task being executed
        self.task_state = "WAITING_FOR_TASKS"  # WAITING_FOR_TASKS -> EXECUTING -> COMPLETE
        self.current_circle_id = 0  # Track which circle we're currently flying
        self.current_waypoint_index = 0  # Track current waypoint within circle
        self.flight_path = []  # Store flight path for visualization
        self.circle_data = {}  # Store detailed circle information
        self.completed_tasks = []  # Track completed tasks
        
        # Flight state
        self.flight_phase = "TAKEOFF"  # TAKEOFF -> PATROL -> RETURN_HOME -> LANDING -> LANDED
        self.return_home_position = [0, 0]  # Return to spawn location
        
        # Image capture
        self.last_image_time = 0
        self.image_interval_seconds = 10.0
        self.images_dir = f"circle_patrol_images_drone_{self.drone_id}"
        os.makedirs(self.images_dir, exist_ok=True)
        self.capturing_images = False  # Only capture during circle patrol, not transitions

        # State tracking
        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.first_waypoint_reached = False  # Track when to start taking pictures

        # Task communication directory (NEW - centralized task system)
        controller_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(controller_dir))
        self.task_dir = os.path.join(project_root, "lawnmower_tasks")
        os.makedirs(self.task_dir, exist_ok=True)
        self.task_pool_file = os.path.join(self.task_dir, "task_pool.json")
        self.task_request_file = os.path.join(self.task_dir, f"drone_{self.drone_id}_request.json")
        self.task_complete_file = os.path.join(self.task_dir, f"drone_{self.drone_id}_complete.json")

        # Initialize devices
        self._initialize_devices()

        print(f"🚁 Centralized Task-Based Circle Patrol Drone {self.drone_id} initialized")
        print(f"   Target altitude: {self.target_altitude}m")
        print(f"   Task pool file: {self.task_pool_file}")
        print(f"   Request file: {self.task_request_file}")
        print(f"   Complete file: {self.task_complete_file}")
        
        print(f"   📂 Task pool exists: {os.path.exists(self.task_pool_file)}")
        print(f"   🔄 Ready to request tasks from supervisor")

    def visualize_flight_path(self):
        """Print a comprehensive visual representation of the drone's flight path."""
        if not self.flight_path:
            print("📊 No flight path data available yet")
            return
        
        print(f"\n{'='*80}")
        print(f"📊 COMPREHENSIVE FLIGHT PATH VISUALIZATION")
        print(f"{'='*80}")
        print(f"📍 Total waypoints visited: {len(self.flight_path)}")
        print(f"🎯 Circles completed: {self.current_circle_id}")
        print(f"⏱️  Mission duration: {self.getTime():.1f} seconds")
        
        # Calculate total distance flown
        total_distance = 0
        for i in range(1, len(self.flight_path)):
            prev_pos = self.flight_path[i-1]
            curr_pos = self.flight_path[i]
            distance = math.sqrt((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)
            total_distance += distance
        
        print(f"📏 Total distance flown: {total_distance:.1f} meters")
        print(f"📊 Average speed: {total_distance/max(self.getTime(), 1):.1f} m/s")
        
        # Calculate image capture statistics
        total_images = len([f for f in os.listdir(self.images_dir) if f.endswith('.jpg')]) if os.path.exists(self.images_dir) else 0
        print(f"📸 Total images captured: {total_images}")
        print(f"📊 Images per circle: {total_images / max(self.current_circle_id, 1):.1f}")
        
        # Group waypoints by circle using circle_data
        print(f"\n🎯 DETAILED CIRCLE BREAKDOWN:")
        print(f"{'='*80}")
        
        for circle_id in sorted(self.circle_data.keys()):
            circle_info = self.circle_data[circle_id]
            waypoints = circle_info['waypoints']
            
            print(f"\n🔄 Circle {circle_id}:")
            print(f"   📍 Center: ({circle_info['center'][0]:.1f}, {circle_info['center'][1]:.1f})")
            print(f"   📏 Radius: {circle_info['radius']:.1f}m")
            print(f"   🎯 Waypoints completed: {len(waypoints)}/{self.num_waypoints}")
            
            # Calculate circle distance
            circle_distance = 0
            for i in range(1, len(waypoints)):
                prev_pos = waypoints[i-1]
                curr_pos = waypoints[i]
                distance = math.sqrt((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)
                circle_distance += distance
            
            print(f"   📏 Distance flown: {circle_distance:.1f}m")
            print(f"   ⏱️  Time spent: {circle_info['end_time'] - circle_info['start_time']:.1f}s")
            
            # Show waypoint details
            print(f"   🎯 Waypoint sequence:")
            for i, pos in enumerate(waypoints):
                print(f"      {i+1:2d}. ({pos[0]:6.1f}, {pos[1]:6.1f})")
        
        # Create ASCII art visualization
        print(f"\n🗺️  ASCII FLIGHT PATH MAP:")
        print(f"{'='*80}")
        self._create_ascii_map()
        
        print(f"\n{'='*80}")
        print(f"✅ FLIGHT PATH VISUALIZATION COMPLETE")
        print(f"{'='*80}")

    def _create_ascii_map(self):
        """Create an ASCII art representation of the flight path."""
        if not self.flight_path:
            return
        
        # Find bounds
        min_x = min(pos[0] for pos in self.flight_path)
        max_x = max(pos[0] for pos in self.flight_path)
        min_y = min(pos[1] for pos in self.flight_path)
        max_y = max(pos[1] for pos in self.flight_path)
        
        # Add padding
        padding = 5
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding
        
        # Calculate grid size
        width = int(max_x - min_x)
        height = int(max_y - min_y)
        
        # Create grid (limit size for readability)
        max_size = 60
        if width > max_size or height > max_size:
            scale = max_size / max(width, height)
            width = int(width * scale)
            height = int(height * scale)
        
        # Initialize grid
        grid = [[' ' for _ in range(width + 1)] for _ in range(height + 1)]
        
        # Plot waypoints
        for i, pos in enumerate(self.flight_path):
            x = int((pos[0] - min_x) * width / (max_x - min_x))
            y = int((pos[1] - min_y) * height / (max_y - min_y))
            
            if 0 <= x <= width and 0 <= y <= height:
                if i == 0:
                    grid[height - y][x] = 'S'  # Start
                elif i == len(self.flight_path) - 1:
                    grid[height - y][x] = 'E'  # End
                else:
                    # Use different symbols for different circles
                    circle_id = self._get_circle_for_waypoint(i)
                    if circle_id == 1:
                        grid[height - y][x] = '1'
                    elif circle_id == 2:
                        grid[height - y][x] = '2'
                    elif circle_id == 3:
                        grid[height - y][x] = '3'
                    elif circle_id == 4:
                        grid[height - y][x] = '4'
                    else:
                        grid[height - y][x] = '*'
        
        # Draw grid
        print(f"   Scale: X={min_x:.0f} to {max_x:.0f}, Y={min_y:.0f} to {max_y:.0f}")
        print(f"   Legend: S=Start, E=End, 1-4=Circle waypoints, *=Other")
        print()
        
        for row in grid:
            print(f"   {''.join(row)}")
        
        print()

    def _get_circle_for_waypoint(self, waypoint_index):
        """Determine which circle a waypoint belongs to."""
        waypoint_count = 0
        current_circle = 1
        
        for i in range(waypoint_index + 1):
            if waypoint_count == 0 and i > 0:
                current_circle += 1
            waypoint_count = (waypoint_count + 1) % self.num_waypoints
        
        return current_circle

    def request_task_from_supervisor(self):
        """Request a task from the supervisor's centralized pool."""
        try:
            # Create task request
            request_data = {
                "drone_id": self.drone_id,
                "timestamp": self.getTime(),
                "status": "requesting"
            }
            
            # Write request file
            with open(self.task_request_file, 'w') as f:
                json.dump(request_data, f, indent=2)
            
            # Wait for supervisor response
            response_file = os.path.join(self.task_dir, f"drone_{self.drone_id}_response.json")
            max_wait_time = 3.0  # Maximum wait time in seconds
            wait_start = self.getTime()
            
            while self.getTime() - wait_start < max_wait_time:
                if os.path.exists(response_file):
                    try:
                        with open(response_file, 'r') as f:
                            response_data = json.load(f)
                        
                        if response_data.get('status') == 'assigned' and response_data.get('task'):
                            print(f"📋 Drone {self.drone_id}: Received task {response_data['task']['task_id']}")
                            # Remove response file after reading to allow future requests
                            os.remove(response_file)
                            return response_data['task']
                        elif response_data.get('status') == 'no_tasks':
                            print(f"📋 Drone {self.drone_id}: No tasks available")
                            # Remove response file after reading to allow future requests
                            os.remove(response_file)
                            return None
                    except Exception as e:
                        print(f"❌ Error reading response: {e}")
                        break
                
                # Small delay before checking again
                time.sleep(0.05)
            
            print(f"⚠️  Drone {self.drone_id}: Timeout waiting for task assignment")
            return None
            
        except Exception as e:
            print(f"❌ Error requesting task: {e}")
            return None
    
    def complete_current_task(self):
        """Mark the current task as completed."""
        if self.current_task is None:
            return False
        
        try:
            # Create completion notification
            complete_data = {
                "drone_id": self.drone_id,
                "task_id": self.current_task['task_id'],
                "timestamp": self.getTime(),
                "status": "completed"
            }
            
            # Write completion file
            with open(self.task_complete_file, 'w') as f:
                json.dump(complete_data, f, indent=2)
            
            # Supervisor will process the completion file
            
            print(f"✅ Drone {self.drone_id}: Completed task {self.current_task['task_id']} - {self.current_task['description']}")
            self.completed_tasks.append(self.current_task)
            self.current_task = None
            return True
            
        except Exception as e:
            print(f"❌ Drone {self.drone_id}: Error completing task: {e}")
            return False

    def show_flight_progress(self):
        """Show current flight progress in real-time."""
        if not self.flight_path:
            return
        
        print(f"\n📊 CURRENT FLIGHT PROGRESS:")
        print(f"   🎯 Current Circle: {self.current_circle_id}")
        print(f"   📍 Waypoints visited: {len(self.flight_path)}")
        print(f"   ⏱️  Flight time: {self.getTime():.1f}s")
        print(f"   📸 Image capture: {'ENABLED' if self.capturing_images else 'DISABLED'}")
        
        if self.current_circle_id > 0 and self.current_circle_id in self.circle_data:
            circle_info = self.circle_data[self.current_circle_id]
            print(f"   🔄 Circle {self.current_circle_id} progress: {len(circle_info['waypoints'])}/{self.num_waypoints} waypoints")
            print(f"   📏 Distance in current circle: {self._calculate_circle_distance(self.current_circle_id):.1f}m")

    def _calculate_circle_distance(self, circle_id):
        """Calculate distance flown in a specific circle."""
        if circle_id not in self.circle_data:
            return 0
        
        waypoints = self.circle_data[circle_id]['waypoints']
        if len(waypoints) < 2:
            return 0
        
        distance = 0
        for i in range(1, len(waypoints)):
            prev_pos = waypoints[i-1]
            curr_pos = waypoints[i]
            distance += math.sqrt((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)
        
        return distance

    def set_camera_orientation(self, camera_yaw_deg, camera_pitch_deg):
        """Set camera orientation in degrees."""
        # Convert degrees to radians
        yaw_rad = math.radians(camera_yaw_deg)
        pitch_rad = math.radians(camera_pitch_deg)
        
        self.camera_yaw_motor.setPosition(yaw_rad)
        self.camera_pitch_motor.setPosition(pitch_rad)

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

    def move_to_target(self, target_xy, verbose_movement=False, verbose_target=False):
        """Move the robot to the given coordinates.
        Modified for task-based system.
        Parameters:
            target_xy (list): [X,Y] target coordinates
            verbose_movement (bool): whether to print remaining angle and distance
            verbose_target (bool): whether to print targets
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
            reached (bool): whether target has been reached
        """
        
        # Calculate distance to target
        distance = math.sqrt(
            (target_xy[0] - self.current_pose[0])**2 +
            (target_xy[1] - self.current_pose[1])**2
        )
        
        # Check if reached
        reached = distance < self.target_precision
        
        if reached:
            return 0, 0, True
        
        # Calculate target angle
        target_angle = np.arctan2(
            target_xy[1] - self.current_pose[1],
            target_xy[0] - self.current_pose[0]
        )
        
        # Calculate angle difference
        angle_left = target_angle - self.current_pose[5]
        # Normalize to [-pi, pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)) if abs(angle_left) > 1e-10 else self.MAX_PITCH_DISTURBANCE,
            self.MAX_PITCH_DISTURBANCE, 0.1
        )

        if verbose_movement:
            print("remaining angle: {:.4f}, remaining distance: {:.4f}".format(
                angle_left, distance))
        return yaw_disturbance, pitch_disturbance, False

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
        """Main flight control loop - Task-based circle patrol."""
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        print(f"\n🚀 Drone {self.drone_id}: Starting task-based circle patrol mission...")

        while self.step(self.time_step) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            current_time = self.getTime()

            # ========== FLIGHT PHASE MANAGEMENT ==========

            if self.flight_phase == "TAKEOFF":
                # Takeoff to target altitude
                if altitude > self.target_altitude - 1:
                    self.flight_phase = "PATROL"
                    print(f"✅ Drone {self.drone_id}: Reached {self.target_altitude}m - Starting PATROL")
            
            elif self.flight_phase == "PATROL":
                # Load tasks if not yet loaded
                if self.task_state == "WAITING_FOR_TASKS":
                    print(f"⏳ Drone {self.drone_id}: Requesting first circle task from supervisor...")
                    self.current_task = self.request_task_from_supervisor()
                    if self.current_task:
                        print(f"📋 Drone {self.drone_id}: Got circle task - {self.current_task['description']}")
                        self.task_state = "EXECUTING"
                        self.current_waypoint_index = 0  # Start with first waypoint
                        
                        # Initialize circle data
                        circle_id = self.current_task.get('circle_id', 0)
                        self.current_circle_id = circle_id
                        self.circle_data[circle_id] = {
                            'center': self.current_task.get('circle_center', [0, 0]),
                            'radius': 25.0,  # Should match supervisor
                            'waypoints': [],
                            'start_time': self.getTime(),
                            'end_time': self.getTime()
                        }
                        
                        # Enable image capture for circle patrol
                        self.capturing_images = True
                        print(f"📸 Image capture ENABLED for Circle {circle_id}")
                        
                        # Set camera orientation
                        if 'camera_yaw' in self.current_task and 'camera_pitch' in self.current_task:
                            self.set_camera_orientation(
                                self.current_task['camera_yaw'],
                                self.current_task['camera_pitch']
                            )
                    else:
                        print(f"⚠️  Drone {self.drone_id}: No tasks available yet, will retry...")
                
                # Execute circle patrol tasks
                elif self.task_state == "EXECUTING":
                    # Check if we have a current task
                    if self.current_task is None:
                        # Request a new task from supervisor
                        self.current_task = self.request_task_from_supervisor()
                        
                        if self.current_task is None:
                            # No more tasks available
                            self.task_state = "COMPLETE"
                            self.flight_phase = "RETURN_HOME"
                            print(f"\n{'='*70}")
                            print(f"✅ Drone {self.drone_id}: All circle tasks complete!")
                            print(f"🏠 Returning to home position (0, 0)...")
                            print(f"{'='*70}\n")
                            
                            # Show flight path visualization
                            self.visualize_flight_path()
                            continue
                    
                    # Execute circle patrol task
                    if self.current_task.get('type') == "circle_patrol":
                        waypoints = self.current_task.get('waypoints', [])
                        circle_id = self.current_task.get('circle_id', 0)
                        
                        # Check if we've completed all waypoints in this circle
                        if self.current_waypoint_index >= len(waypoints):
                            print(f"\n{'='*70}")
                            print(f"🎉 DRONE {self.drone_id} COMPLETED CIRCLE {circle_id} PATROL")
                            print(f"{'='*70}")
                            print(f"✅ Drone {self.drone_id}: Finished all {len(waypoints)} waypoints")
                            print(f"📊 Circle {circle_id} completed successfully by Drone {self.drone_id}!")
                            print(f"{'='*70}\n")
                            
                            # Complete this circle task
                            self.complete_current_task()
                            
                            # Request next circle task
                            self.current_task = self.request_task_from_supervisor()
                            if self.current_task:
                                print(f"📋 Drone {self.drone_id}: Got next circle task - {self.current_task['description']}")
                                self.current_waypoint_index = 0
                                
                                # Initialize next circle data
                                next_circle_id = self.current_task.get('circle_id', 0)
                                self.current_circle_id = next_circle_id
                                self.circle_data[next_circle_id] = {
                                    'center': self.current_task.get('circle_center', [0, 0]),
                                    'radius': 25.0,
                                    'waypoints': [],
                                    'start_time': self.getTime(),
                                    'end_time': self.getTime()
                                }
                                
                                # Enable image capture for next circle
                                self.capturing_images = True
                                print(f"📸 Image capture ENABLED for Circle {next_circle_id}")
                                
                                # Set camera orientation
                                if 'camera_yaw' in self.current_task and 'camera_pitch' in self.current_task:
                                    self.set_camera_orientation(
                                        self.current_task['camera_yaw'],
                                        self.current_task['camera_pitch']
                                    )
                            else:
                                # No more tasks available
                                self.task_state = "COMPLETE"
                                self.flight_phase = "RETURN_HOME"
                                print(f"\n{'='*70}")
                                print(f"✅ Drone {self.drone_id}: All circle tasks complete!")
                                print(f"🏠 Returning to home position (0, 0)...")
                                print(f"{'='*70}\n")
                                
                                # Show flight path visualization
                                self.visualize_flight_path()
                                continue
                        else:
                            # Execute current waypoint
                            target_position = waypoints[self.current_waypoint_index]
                            waypoint_id = self.current_waypoint_index + 1
                            
                            # Mark first waypoint as reached for image capture
                            if not self.first_waypoint_reached:
                                self.first_waypoint_reached = True
                                print("✅ First waypoint reached! Starting image capture...")
                                self.capturing_images = True
                            
                            if self.getTime() - t1 > 0.1:
                                yaw_dist, pitch_dist, reached = self.move_to_target(target_position, verbose_target=True)
                                yaw_disturbance = yaw_dist
                                pitch_disturbance = pitch_dist
                                t1 = self.getTime()
                                
                                if reached:
                                    print(f"✅ Drone {self.drone_id} - Circle {circle_id} - Waypoint {waypoint_id}/{len(waypoints)}: ({target_position[0]:.1f}, {target_position[1]:.1f})")
                                    
                                    # Add to flight path
                                    self.flight_path.append(target_position)
                                    
                                    # Track waypoint in circle data
                                    if circle_id in self.circle_data:
                                        self.circle_data[circle_id]['waypoints'].append(target_position)
                                        self.circle_data[circle_id]['end_time'] = self.getTime()
                                    
                                    # Move to next waypoint
                                    self.current_waypoint_index += 1
            
            elif self.flight_phase == "RETURN_HOME":
                # Navigate back to home position (0, 0)
                if self.getTime() - t1 > 0.1:
                    yaw_dist, pitch_dist, reached = self.move_to_target(self.return_home_position, verbose_target=True)
                    yaw_disturbance = yaw_dist
                    pitch_disturbance = pitch_dist
                    t1 = self.getTime()

                    if reached:
                        print(f"\n{'='*70}")
                        print(f"✅ Drone {self.drone_id}: Reached home position (0, 0)")
                        print(f"🛬 Beginning landing sequence...")
                        print(f"{'='*70}\n")
                        self.flight_phase = "LANDING"
            
            elif self.flight_phase == "LANDING":
                # Descend to ground
                if altitude < 0.5:
                    self.flight_phase = "LANDED"
                    print(f"✅ Drone {self.drone_id}: LANDED - Mission complete!")
            
            elif self.flight_phase == "LANDED":
                # Stop motors
                for motor in [self.front_left_motor, self.front_right_motor,
                             self.rear_left_motor, self.rear_right_motor]:
                    motor.setVelocity(0)
                continue

            # Save images periodically (ONLY during circle patrol, not during transitions)
            if (self.first_waypoint_reached and 
                self.capturing_images and 
                current_time - self.last_image_time >= self.image_interval_seconds):
                self.save_camera_image()
                self.last_image_time = current_time

            # Show progress updates every 30 seconds
            if not hasattr(self, 'last_progress_time'):
                self.last_progress_time = 0
            if current_time - self.last_progress_time >= 30.0:
                self.show_flight_progress()
                self.last_progress_time = current_time

            # ========== MOTOR CONTROL ==========
            
            # Determine target altitude
            if self.flight_phase in ["TAKEOFF", "PATROL", "RETURN_HOME"]:
                target_alt = self.target_altitude  # Maintain target altitude
            elif self.flight_phase == "LANDING":
                target_alt = 0.0  # Descend to ground
            else:  # LANDED
                target_alt = 0.0
            
            # Stabilization
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            
            # Altitude control
            clamped_difference_altitude = clamp(
                target_alt - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            # Motor mixing
            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            # Apply to motors
            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5

robot = TaskBasedCirclePatrolDrone()
robot.run()