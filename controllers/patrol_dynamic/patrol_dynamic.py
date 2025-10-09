"""
Lawnmower Pattern Mapping Controller
Executes systematic grid mapping with precise camera control.
"""

from controller import Robot
import sys
import os
import json
import math
from datetime import datetime

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

try:
    import cv2
except ImportError:
    print("Warning: 'cv2' module not found. Images will not be saved.")
    cv2 = None

try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class LawnmowerDrone(Robot):
    # Constants (same as patrol_with_images)
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 5.0  # Precision in meters for waypoint arrival

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
        self.target_altitude = 35.0  # Flight altitude (increased by 10m)
        self.mapping_altitude = 35.0  # Altitude for mapping tasks
        
        # Circle mission parameters (activated via MQTT nav.circle)
        self.circle_active = False
        self.circle_center = [0.0, 0.0]
        self.circle_radius = 0.0
        self.circle_num_waypoints = 8
        self.circle_waypoints = None
        self.circle_target_index = 0
        self.circle_waypoints_initialized = False
        self.circle_first_waypoint_reached = False
        self.circle_waypoints_visited = 0
        self.circle_image_interval_seconds = 10.0
        self._circle_last_image_time = None
        self.circle_image_counter = 0

        # Task management
        self.tasks = []  # Will be loaded from supervisor
        self.current_task_index = 0
        self.task_state = "WAITING_FOR_TASKS"  # WAITING_FOR_TASKS -> EXECUTING -> COMPLETE
        self.current_waypoint = None
        self.at_waypoint = False
        
        # Flight state
        self.flight_phase = "TAKEOFF"  # TAKEOFF -> MAPPING -> RETURN_HOME -> LANDING -> LANDED
        self.return_home_position = [0, 0]  # Will be overwritten with actual spawn XY
        self.home_position_xy = None  # Captured from initial GPS
        
        # State tracking (same as patrol_with_images)
        self.current_pose = 6 * [0]  # X, Y, Z, roll, pitch, yaw
        self.target_position = [0, 0, 0]
        
        # Deprecated: JSON task file flow removed; using MQTT
        
        # Image capture
        if cv2:
            self.images_dir = f"lawnmower_images_drone_{self.drone_id}"
            os.makedirs(self.images_dir, exist_ok=True)
            self.image_counter = 0
            
            # JSON metadata directory
            self.metadata_dir = os.path.join(self.images_dir, "metadata")
            os.makedirs(self.metadata_dir, exist_ok=True)
        
        # Initialize devices
        self._initialize_devices()
        
        print(f"🚁 Lawnmower Drone {self.drone_id} initialized")
        print(f"   Target altitude: {self.target_altitude}m")
        

        # MQTT subscribe to per-drone tasks/commands (optional)
        self.mqtt_client = None
        if mqtt is not None:
            try:
                self.mqtt_client = mqtt.Client()
                self.mqtt_client.on_message = self._on_mqtt_message
                self.mqtt_client.connect("localhost", 1883, 60)
                self.mqtt_client.subscribe(f"tasks/drone_{self.drone_id}")
                self.mqtt_client.subscribe(f"cmd/drone_{self.drone_id}/nav/climb")
                self.mqtt_client.subscribe(f"cmd/drone_{self.drone_id}/nav/circle")
                self.mqtt_client.loop_start()
                print(f"🔌 MQTT: drone_{self.drone_id} subscribed to tasks/cmd topics")
            except Exception as e:
                print(f"⚠️  MQTT init failed for drone_{self.drone_id}: {e}")
        
        # No JSON task loading; tasks arrive via MQTT

    def _on_mqtt_message(self, _client, _userdata, msg):
        """Handle incoming MQTT messages for tasks/commands."""
        try:
            payload_text = msg.payload.decode()
            data = json.loads(payload_text)
        except Exception:
            return
        msg_type = data.get("type")
        if msg.topic.endswith("/nav/climb") or msg_type == "nav.climb":
            args = data.get("args", {})
            try:
                dz = float(args.get("dz", 0.0))
            except Exception:
                dz = 0.0
            rate = args.get("rate", 1.0)
            self._apply_nav_climb(dz, rate)
            return
        if msg.topic.endswith("/nav/circle") or msg_type == "nav.circle":
            args = data.get("args", {})
            center = args.get("center", [0.0, 0.0])
            radius = args.get("radius", 0.0)
            altitude = args.get("altitude", None)
            if isinstance(center, list) and len(center) >= 2:
                try:
                    self.circle_center = [float(center[0]), float(center[1])]
                    self.circle_radius = float(radius)
                    if altitude is not None:
                        self.target_altitude = float(altitude)
                    # Initialize circle waypoints from current position
                    self._init_circle_waypoints_from_current()
                    self.circle_active = True
                    print(f"🟢 Drone {self.drone_id}: nav.circle center={self.circle_center} radius={self.circle_radius}")
                except Exception:
                    pass
            return

    def _apply_nav_climb(self, dz: float, rate: float):
        """Adjust target altitude by dz meters (positive up)."""
        current_altitude = float(self.current_pose[2]) if len(self.current_pose) >= 3 else 0.0
        new_target = max(0.0, current_altitude + dz)
        self.target_altitude = new_target
        print(f"🪁 Drone {self.drone_id}: nav.climb dz={dz} -> target_altitude={new_target:.1f}m")

    def _generate_circle_waypoints(self, start_x: float, start_y: float):
        """Generate ordered waypoints around circle starting at closest to current position."""
        if self.circle_radius <= 0.0:
            return []
        all_waypoints = []
        for i in range(self.circle_num_waypoints):
            angle = 2 * math.pi * i / self.circle_num_waypoints
            x = self.circle_center[0] + self.circle_radius * math.cos(angle)
            y = self.circle_center[1] + self.circle_radius * math.sin(angle)
            all_waypoints.append([x, y])
        # find closest index
        min_distance = float('inf')
        closest_index = 0
        for i, (wx, wy) in enumerate(all_waypoints):
            d = math.sqrt((wx - start_x)**2 + (wy - start_y)**2)
            if d < min_distance:
                min_distance = d
                closest_index = i
        ordered = []
        for i in range(self.circle_num_waypoints):
            idx = (closest_index + i) % self.circle_num_waypoints
            ordered.append(all_waypoints[idx])
        return ordered

    def _init_circle_waypoints_from_current(self):
        x_pos = float(self.current_pose[0]) if len(self.current_pose) >= 1 else 0.0
        y_pos = float(self.current_pose[1]) if len(self.current_pose) >= 2 else 0.0
        self.circle_waypoints = self._generate_circle_waypoints(x_pos, y_pos)
        self.circle_target_index = 0
        self.circle_waypoints_initialized = True
        self.circle_first_waypoint_reached = False
        self.circle_waypoints_visited = 0
        if self.circle_waypoints:
            print(f"🎯 Drone {self.drone_id}: circle waypoints initialized; first=({self.circle_waypoints[0][0]:.1f}, {self.circle_waypoints[0][1]:.1f})")

    def _initialize_devices(self):
        """Initialize all drone devices."""
        # Sensors
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Camera
        if cv2:
            self.camera = self.getDevice("camera")
            self.camera.enable(self.time_step)
            try:
                self.camera.setFov(1.5)
            except:
                pass
        
        # Camera motors
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_yaw_motor = self.getDevice("camera yaw")
        
        # Set initial camera position (face 90° left, tilted down)
        self.camera_pitch_motor.setPosition(0.5)           # Tilted down ~30°
        self.camera_yaw_motor.setPosition(1.57079632679)   # 90° left (π/2 radians)

        # Propeller motors
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def set_position(self, pos):
        """Update current position."""
        self.current_pose = pos
    
    def load_tasks(self):
        """Deprecated: JSON task flow removed."""
        return False
    
    def get_current_task(self):
        """Get the current task to execute."""
        if self.current_task_index < len(self.tasks):
            return self.tasks[self.current_task_index]
        return None
    
    def set_camera_orientation(self, camera_yaw_deg, camera_pitch_deg):
        """Set camera orientation in degrees."""
        # Convert degrees to radians
        yaw_rad = math.radians(camera_yaw_deg)
        pitch_rad = math.radians(camera_pitch_deg)
        
        self.camera_yaw_motor.setPosition(yaw_rad)
        self.camera_pitch_motor.setPosition(pitch_rad)
    
    def move_to_target(self, target_xy, verbose=False):
        """Move to a target XY position. Returns (yaw_dist, pitch_dist, reached)."""
        # Calculate target angle
        target_angle = np.arctan2(
            target_xy[1] - self.current_pose[1],
            target_xy[0] - self.current_pose[0]
        )
        
        # Calculate distance to target
        distance = math.sqrt(
            (target_xy[0] - self.current_pose[0])**2 +
            (target_xy[1] - self.current_pose[1])**2
        )
        
        # Check if reached
        reached = distance < self.target_precision
        
        if reached:
            return 0, 0, True
        
        # Calculate angle difference
        angle_left = target_angle - self.current_pose[5]
        # Normalize to [-pi, pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi
        
        # Calculate disturbances (same as patrol_with_images)
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)) if abs(angle_left) > 1e-10 else self.MAX_PITCH_DISTURBANCE,
            self.MAX_PITCH_DISTURBANCE, 0.1
        )
        
        if verbose:
            print(f"   Distance: {distance:.2f}m, Angle: {math.degrees(angle_left):.1f}°")
        
        return yaw_disturbance, pitch_disturbance, False
    
    def save_image(self, task_info=""):
        """Save camera image with GPS coordinates and metadata JSON."""
        if not cv2:
            return
        
        try:
            image = self.camera.getImage()
            if image:
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8)
                image_array = image_array.reshape((height, width, 4))
                
                # Convert RGBA to RGB
                rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                
                # Get current drone position
                x, y, z = self.current_pose[0], self.current_pose[1], self.current_pose[2]
                drone_roll, drone_pitch, drone_yaw = self.current_pose[3], self.current_pose[4], self.current_pose[5]
                
                # Get camera orientation (relative to drone)
                camera_pitch_relative = self.camera_pitch_motor.getTargetPosition()
                camera_yaw_relative = self.camera_yaw_motor.getTargetPosition()
                
                # Calculate absolute camera orientation (drone orientation + camera gimbal)
                camera_roll_absolute = drone_roll
                camera_pitch_absolute = drone_pitch + camera_pitch_relative
                camera_yaw_absolute = drone_yaw + camera_yaw_relative
                
                # Get camera focal length (convert FOV to focal length in pixels)
                fov = self.camera.getFov()  # Field of view in radians
                # Focal length in pixels: f = (image_width / 2) / tan(fov / 2)
                focal_length_pixels = (width / 2.0) / math.tan(fov / 2.0)
                
                # Create timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                
                # Create filename with all relevant info
                filename = f"lawnmower_{timestamp}_task{self.current_task_index}_{task_info}_x{x:.1f}_y{y:.1f}_z{z:.1f}.jpg"
                image_path = os.path.join(self.images_dir, filename)
                
                # Convert RGB to BGR for cv2.imwrite
                cv2.imwrite(image_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                
                # Create metadata JSON
                metadata = {
                    "image_filename": filename,
                    "timestamp": timestamp,
                    "task_index": self.current_task_index,
                    "task_info": task_info,
                    "drone_position": {
                        "x": float(x),
                        "y": float(y),
                        "z": float(z)
                    },
                    "drone_orientation": {
                        "roll": float(drone_roll),
                        "pitch": float(drone_pitch),
                        "yaw": float(drone_yaw)
                    },
                    "camera_orientation_absolute": {
                        "roll": float(camera_roll_absolute),
                        "pitch": float(camera_pitch_absolute),
                        "yaw": float(camera_yaw_absolute)
                    },
                    "camera_orientation_relative": {
                        "pitch": float(camera_pitch_relative),
                        "yaw": float(camera_yaw_relative)
                    },
                    "camera_intrinsics": {
                        "fov_radians": float(fov),
                        "fov_degrees": float(math.degrees(fov)),
                        "focal_length_pixels": float(focal_length_pixels),
                        "image_width": int(width),
                        "image_height": int(height),
                        "principal_point_x": float(width / 2.0),
                        "principal_point_y": float(height / 2.0)
                    }
                }
                
                # Save metadata JSON
                json_filename = filename.replace('.jpg', '.json')
                json_path = os.path.join(self.metadata_dir, json_filename)
                with open(json_path, 'w') as f:
                    json.dump(metadata, f, indent=2)
                
                print(f"📸 Saved: {filename}")
                print(f"   📄 Metadata: {json_filename}")
                self.image_counter += 1
            else:
                print(f"⚠️  No image data from camera")
        except Exception as e:
            print(f"❌ Error saving image: {e}")
            import traceback
            traceback.print_exc()

    def run(self):
        """Main flight control loop."""
        t1 = self.getTime()
        
        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        
        last_image_time = 0
        image_interval = 1.0  # Capture image every 1 second during task execution
        
        print(f"\n🚀 Drone {self.drone_id}: Starting lawnmower mapping mission...")
        
        while self.step(self.time_step) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # Capture home position once (actual spawn XY) for return
            if self.home_position_xy is None:
                self.home_position_xy = [x_pos, y_pos]
                self.return_home_position = self.home_position_xy
                print(f"🏠 Drone {self.drone_id}: Home position set to ({x_pos:.1f}, {y_pos:.1f})")
            
            current_time = self.getTime()
            
            # ========== FLIGHT PHASE MANAGEMENT ==========
            
            if self.flight_phase == "TAKEOFF":
                # Takeoff to target altitude
                if altitude > self.target_altitude - 1:
                    self.flight_phase = "MAPPING"
                    print(f"✅ Drone {self.drone_id}: Reached {self.target_altitude}m - Starting MAPPING")
                    # Try to load tasks
                    self.load_tasks()
            
            elif self.flight_phase == "MAPPING":
                # If using MQTT-only, immediately consider tasks as EXECUTING placeholder
                if self.task_state == "WAITING_FOR_TASKS":
                    self.task_state = "EXECUTING"

                # Circle mission handling (takes precedence over legacy tasks)
                if self.circle_active:
                    # Initialize waypoints on first pass
                    if not self.circle_waypoints_initialized:
                        self._init_circle_waypoints_from_current()
                    # Move toward current waypoint
                    if self.circle_waypoints and self.getTime() - t1 > 0.1:
                        target_xy = self.circle_waypoints[self.circle_target_index]
                        yaw_dist, pitch_dist, reached = self.move_to_target(target_xy, verbose=False)
                        yaw_disturbance = yaw_dist
                        pitch_disturbance = pitch_dist
                        t1 = self.getTime()

                        # Periodic image capture during circle mission
                        if self._circle_last_image_time is None:
                            self._circle_last_image_time = current_time
                        elif (current_time - self._circle_last_image_time) >= self.circle_image_interval_seconds:
                            self.save_image(f"circle_{self.circle_image_counter+1}")
                            self.circle_image_counter += 1
                            self._circle_last_image_time = current_time

                        if reached:
                            if not self.circle_first_waypoint_reached:
                                self.circle_first_waypoint_reached = True
                                self.circle_waypoints_visited = 1
                            else:
                                self.circle_waypoints_visited += 1

                            # Completed one full circle (visited all waypoints)
                            if self.circle_waypoints_visited >= len(self.circle_waypoints):
                                print(f"✅ Drone {self.drone_id}: Circle mission complete. Returning home...")
                                self.circle_active = False
                                self.flight_phase = "RETURN_HOME"
                                # Reset circle image timer for next mission
                                self._circle_last_image_time = None
                                self.circle_image_counter = 0
                            else:
                                self.circle_target_index = (self.circle_target_index + 1) % len(self.circle_waypoints)
                    # Skip legacy task execution
                    pass

                # Execute tasks (legacy scan flow retained; not used for nav.climb/circle)
                elif self.task_state == "EXECUTING":
                    task = self.get_current_task()
                    
                    if task is None:
                        # All tasks complete
                        self.task_state = "COMPLETE"
                        self.flight_phase = "RETURN_HOME"
                        print(f"\n{'='*70}")
                        print(f"✅ Drone {self.drone_id}: All tasks complete!")
                        print(f"🏠 Returning to home position (0, 0)...")
                        print(f"{'='*70}\n")
                    else:
                        # Execute current task (only "scan" type now)
                        task_type = task['type']
                        
                        if task_type == "scan":
                            # Execute scanning task
                            start_xy = task['start']
                            end_xy = task['end']
                            
                            # Initialize task state on first execution
                            if not hasattr(self, '_scan_phase'):
                                self._scan_phase = "NAVIGATE_TO_START"
                                self._scan_task_initialized = False
                                self._task_completed = False
                            
                            # Check if task was already completed (prevent re-entry)
                            if self._task_completed:
                                # Move to next task
                                self.current_task_index += 1
                                self._scan_phase = "NAVIGATE_TO_START"
                                self._scan_task_initialized = False
                                self._task_completed = False
                                continue
                            
                            # Phase 1: Navigate to start position
                            if self._scan_phase == "NAVIGATE_TO_START":
                                if not self._scan_task_initialized:
                                    print(f"\n{'='*70}")
                                    print(f"📋 Drone {self.drone_id}: Starting Task {self.current_task_index + 1}/{len(self.tasks)}")
                                    print(f"{'='*70}")
                                    print(f"   Type: Scan")
                                    print(f"   Start: ({start_xy[0]:.1f}, {start_xy[1]:.1f})")
                                    print(f"   End: ({end_xy[0]:.1f}, {end_xy[1]:.1f})")
                                    print(f"   Phase: Navigating to start position...")
                                    self._scan_task_initialized = True
                                
                                if self.getTime() - t1 > 0.1:
                                    yaw_dist, pitch_dist, reached = self.move_to_target(start_xy, verbose=False)
                                    yaw_disturbance = yaw_dist
                                    pitch_disturbance = pitch_dist
                                    t1 = self.getTime()
                                    
                                    if reached:
                                        print(f"✅ Reached start position ({start_xy[0]:.1f}, {start_xy[1]:.1f})")
                                        print(f"   Setting camera: yaw={task.get('camera_yaw', 0)}°, pitch={task.get('camera_pitch', 30)}°")
                                        # Set camera orientation
                                        self.set_camera_orientation(
                                            task.get('camera_yaw', 0),
                                            task.get('camera_pitch', 30)
                                        )
                                        # Move to scanning phase
                                        self._scan_phase = "SCANNING"
                                        print(f"📸 Phase: Scanning to ({end_xy[0]:.1f}, {end_xy[1]:.1f})...")
                            
                            # Phase 2: Scan from start to end
                            elif self._scan_phase == "SCANNING":
                                # Initialize scan progress tracking
                                if not hasattr(self, '_scan_start_time'):
                                    self._scan_start_time = current_time
                                    self._images_taken_this_task = 0
                                    self._scan_distance = math.sqrt(
                                        (end_xy[0] - start_xy[0])**2 + 
                                        (end_xy[1] - start_xy[1])**2
                                    )
                                
                                if self.getTime() - t1 > 0.1:
                                    yaw_dist, pitch_dist, reached = self.move_to_target(end_xy, verbose=False)
                                    yaw_disturbance = yaw_dist
                                    pitch_disturbance = pitch_dist
                                    t1 = self.getTime()
                                    
                                    # Calculate progress (0.0 to 1.0)
                                    current_distance = math.sqrt(
                                        (self.current_pose[0] - start_xy[0])**2 + 
                                        (self.current_pose[1] - start_xy[1])**2
                                    )
                                    progress = current_distance / self._scan_distance if self._scan_distance > 0 else 0
                                    
                                    # Take 10 images: evenly spaced from 10% to 90% progress
                                    image_checkpoints = [0.10, 0.19, 0.28, 0.37, 0.46, 0.55, 0.64, 0.73, 0.82, 0.90]
                                    
                                    for i, checkpoint in enumerate(image_checkpoints):
                                        if self._images_taken_this_task == i and progress >= checkpoint:
                                            self.save_image(f"scan_img{i+1}of10")
                                            self._images_taken_this_task += 1
                                            break
                                    
                                    if reached:
                                        # Take final image if we haven't taken 10 yet
                                        if self._images_taken_this_task < 10:
                                            self.save_image(f"scan_img10of10_final")
                                            self._images_taken_this_task = 10
                                        
                                        print(f"✅ Scan complete! Reached end position ({end_xy[0]:.1f}, {end_xy[1]:.1f})")
                                        print(f"   Captured {self._images_taken_this_task} images during scan")
                                        print(f"{'='*70}")
                                        print(f"✅ Task {self.current_task_index + 1}/{len(self.tasks)} COMPLETE")
                                        print(f"{'='*70}\n")
                                        # Mark task as completed and reset scan tracking
                                        self._task_completed = True
                                        if hasattr(self, '_scan_start_time'):
                                            del self._scan_start_time
                                        if hasattr(self, '_images_taken_this_task'):
                                            del self._images_taken_this_task
                                        if hasattr(self, '_scan_distance'):
                                            del self._scan_distance
            
            elif self.flight_phase == "RETURN_HOME":
                # Navigate back to home position (0, 0)
                if self.getTime() - t1 > 0.1:
                    yaw_dist, pitch_dist, reached = self.move_to_target(self.return_home_position, verbose=False)
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
            
            # ========== MOTOR CONTROL ==========
            
            # Determine target altitude
            if self.flight_phase in ["TAKEOFF", "MAPPING", "RETURN_HOME"]:
                target_alt = self.target_altitude  # Maintain 15m altitude
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


if __name__ == "__main__":
    robot = LawnmowerDrone()
    robot.run()
