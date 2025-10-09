"""Multi-drone photographer controller implementing circular photography patterns
   for 3D modeling with job coordination and failure recovery."""

from controller import Robot
import sys
import os
import math
import time
import json
from datetime import datetime
from typing import List, Optional, Tuple
import threading

# Import our coordination system
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from multi_drone_coordinator.multi_drone_coordinator import (
    get_coordinator, CirclePhotographyJob, PhotoMetadata, JobStatus
)

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

try:
    import cv2
except ImportError:
    print("Warning: 'cv2' module not found. Photo saving will be limited.")
    cv2 = None

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class CircularPhotographyDrone(Robot):
    """Drone controller for circular photography missions with job coordination."""
    
    # Flight control constants
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 2.0

    def __init__(self, drone_id: Optional[str] = None):
        Robot.__init__(self)
        
        # Generate unique drone ID if not provided
        if drone_id is None:
            self.drone_id = f"drone_{int(time.time() * 1000) % 100000}"
        else:
            self.drone_id = drone_id
            
        self.time_step = int(self.getBasicTimeStep())
        
        # Get coordinator instance
        self.coordinator = get_coordinator()
        
        # Flight state
        self.current_pose = [0, 0, 0, 0, 0, 0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.hover_altitude = 20.0
        
        # Job management
        self.current_job: Optional[CirclePhotographyJob] = None
        self.current_circle_positions: List[Tuple[float, float]] = []
        self.current_position_index = 0
        self.current_angle_index = 0
        self.photo_metadata: List[PhotoMetadata] = []
        
        # Photography state
        self.photos_dir = f"drone_{self.drone_id}_photos"
        os.makedirs(self.photos_dir, exist_ok=True)
        
        # Flight phases
        self.flight_phase = "initialization"  # initialization, seeking_job, traveling_to_job, executing_circle, returning_home
        
        # Heartbeat management
        self.last_heartbeat_time = 0
        self.heartbeat_interval = 5.0  # seconds
        
        # Initialize devices
        self._initialize_devices()
        
        # Register with coordinator
        self.coordinator.register_drone(self.drone_id)
        
        print(f"🚁 Circular Photography Drone {self.drone_id} initialized")

    def _initialize_devices(self):
        """Initialize all drone devices."""
        # Camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        
        try:
            self.camera.setFov(1.2)  # Moderate FOV for detailed photography
        except:
            pass
        
        # Navigation sensors
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Motors
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        # Camera pitch control for different angles
        self.camera_pitch_motor = self.getDevice("camera pitch")
        
        # Initialize motors
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def _send_heartbeat(self):
        """Send heartbeat to coordinator."""
        current_time = time.time()
        if current_time - self.last_heartbeat_time > self.heartbeat_interval:
            x, y, z = self.current_pose[0], self.current_pose[1], self.current_pose[2]
            self.coordinator.update_drone_heartbeat(self.drone_id, x, y, z)
            self.last_heartbeat_time = current_time

    def _request_new_job(self) -> bool:
        """Request a new job from the coordinator."""
        job = self.coordinator.request_job(self.drone_id)
        if job:
            self.current_job = job
            self._generate_circle_positions()
            self.flight_phase = "traveling_to_job"
            print(f"📋 {self.drone_id} received job: {job.job_id} at ({job.center_x}, {job.center_y})")
            return True
        return False

    def _generate_circle_positions(self):
        """Generate positions around the circle for the current job."""
        if not self.current_job:
            return
            
        job = self.current_job
        self.current_circle_positions = []
        
        for i in range(job.num_positions):
            angle = (2 * math.pi * i) / job.num_positions
            x = job.center_x + job.radius * math.cos(angle)
            y = job.center_y + job.radius * math.sin(angle)
            self.current_circle_positions.append((x, y))
        
        self.current_position_index = 0
        self.current_angle_index = 0
        self.photo_metadata = []

    def _move_to_position(self, target_x: float, target_y: float, target_z: float) -> bool:
        """Move to a specific 3D position. Returns True when reached."""
        self.target_position = [target_x, target_y, target_z]
        
        # Check if we've reached the target
        distance = math.sqrt(
            (target_x - self.current_pose[0])**2 + 
            (target_y - self.current_pose[1])**2 + 
            (target_z - self.current_pose[2])**2
        )
        
        return distance < self.target_precision

    def _calculate_flight_control(self) -> Tuple[float, float]:
        """Calculate yaw and pitch disturbances for flight control."""
        # Calculate heading to target (ignoring Z for now)
        target_yaw = math.atan2(
            self.target_position[1] - self.current_pose[1],
            self.target_position[0] - self.current_pose[0]
        )
        
        # Calculate turn angle
        angle_left = target_yaw - self.current_pose[5]
        angle_left = (angle_left + 2 * math.pi) % (2 * math.pi)
        if angle_left > math.pi:
            angle_left -= 2 * math.pi
        
        # Calculate disturbances
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * math.pi)
        pitch_disturbance = clamp(math.log10(abs(angle_left)) if abs(angle_left) > 0.1 else 0, 
                                self.MAX_PITCH_DISTURBANCE, 0.1)
        
        return yaw_disturbance, pitch_disturbance

    def _capture_photo_with_angle(self, camera_angle_deg: float):
        """Capture a photo at a specific camera angle and save metadata."""
        if not self.current_job:
            return
            
        try:
            # Set camera angle (convert degrees to radians)
            camera_angle_rad = math.radians(camera_angle_deg)
            self.camera_pitch_motor.setPosition(camera_angle_rad)
            
            # Wait a moment for camera to stabilize
            for _ in range(10):
                if self.step(self.time_step) == -1:
                    return
            
            # Capture image
            image = self.camera.getImage()
            if image:
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                
                # Generate filename
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = (f"{self.photos_dir}/job_{self.current_job.job_id}_"
                          f"pos_{self.current_position_index:02d}_"
                          f"angle_{camera_angle_deg:+03.0f}_{timestamp}.jpg")
                
                # Save image (simplified version without cv2)
                if cv2:
                    image_array = np.frombuffer(image, dtype=np.uint8)
                    image_array = image_array.reshape((height, width, 4))
                    rgb_image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
                    cv2.imwrite(filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                else:
                    # Fallback: save raw image data
                    with open(filename.replace('.jpg', '.raw'), 'wb') as f:
                        f.write(image)
                
                # Create metadata
                metadata = PhotoMetadata(
                    job_id=self.current_job.job_id,
                    drone_id=self.drone_id,
                    position_index=self.current_position_index,
                    camera_angle=camera_angle_deg,
                    camera_x=self.current_pose[0],
                    camera_y=self.current_pose[1],
                    camera_z=self.current_pose[2],
                    target_x=self.current_job.center_x,
                    target_y=self.current_job.center_y,
                    target_z=0,  # Ground level target
                    timestamp=time.time(),
                    filename=os.path.basename(filename)
                )
                
                self.photo_metadata.append(metadata)
                print(f"📸 {self.drone_id} captured photo at angle {camera_angle_deg}°: {filename}")
                
        except Exception as e:
            print(f"❌ Error capturing photo: {e}")

    def _execute_current_job(self):
        """Execute the circular photography pattern for the current job."""
        if not self.current_job or not self.current_circle_positions:
            return
            
        job = self.current_job
        
        # Get current target position
        if self.current_position_index < len(self.current_circle_positions):
            target_x, target_y = self.current_circle_positions[self.current_position_index]
            target_z = job.altitude
            
            # Move to position
            if self._move_to_position(target_x, target_y, target_z):
                # We've reached the position, now capture photos at different angles
                if self.current_angle_index < len(job.angles_per_position):
                    angle = job.angles_per_position[self.current_angle_index]
                    self._capture_photo_with_angle(angle)
                    self.current_angle_index += 1
                else:
                    # Finished all angles at this position, move to next position
                    self.current_position_index += 1
                    self.current_angle_index = 0
                    
            # Set target for flight control
            self.target_position = [target_x, target_y, target_z]
            
        else:
            # Completed all positions, finish the job
            self._complete_job()

    def _complete_job(self):
        """Complete the current job and report to coordinator."""
        if not self.current_job:
            return
            
        try:
            self.coordinator.complete_job(
                self.drone_id, 
                self.current_job.job_id, 
                self.photo_metadata
            )
            
            print(f"✅ {self.drone_id} completed job {self.current_job.job_id} with {len(self.photo_metadata)} photos")
            
            # Reset job state
            self.current_job = None
            self.current_circle_positions = []
            self.photo_metadata = []
            self.flight_phase = "seeking_job"
            
        except Exception as e:
            print(f"❌ Error completing job: {e}")
            self._fail_current_job(f"Error completing job: {e}")

    def _fail_current_job(self, error_message: str):
        """Fail the current job and report to coordinator."""
        if not self.current_job:
            return
            
        try:
            self.coordinator.fail_job(self.drone_id, self.current_job.job_id, error_message)
            print(f"❌ {self.drone_id} failed job {self.current_job.job_id}: {error_message}")
            
        except Exception as e:
            print(f"❌ Error failing job: {e}")
        
        # Reset job state
        self.current_job = None
        self.current_circle_positions = []
        self.photo_metadata = []
        self.flight_phase = "seeking_job"

    def run(self):
        """Main control loop."""
        print(f"🚀 Starting {self.drone_id}")
        
        # Mark job as started once we reach altitude
        job_started = False
        
        while self.step(self.time_step) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.current_pose = [x_pos, y_pos, altitude, roll, pitch, yaw]
            
            # Send heartbeat
            self._send_heartbeat()
            
            # State machine for flight phases
            if self.flight_phase == "initialization":
                # Rise to hover altitude
                self.target_position = [0, 0, self.hover_altitude]
                if altitude > self.hover_altitude - 2:
                    self.flight_phase = "seeking_job"
                    print(f"🚁 {self.drone_id} ready for missions at altitude {altitude:.1f}m")
            
            elif self.flight_phase == "seeking_job":
                # Request new job from coordinator
                if self._request_new_job():
                    job_started = False  # Reset for new job
                else:
                    # No jobs available, maintain hover
                    self.target_position = [x_pos, y_pos, self.hover_altitude]
            
            elif self.flight_phase == "traveling_to_job":
                # Fly to job location
                if self.current_job:
                    job_center_x, job_center_y = self.current_job.center_x, self.current_job.center_y
                    if self._move_to_position(job_center_x, job_center_y, self.current_job.altitude):
                        self.flight_phase = "executing_circle"
                        if not job_started:
                            self.coordinator.start_job(self.drone_id, self.current_job.job_id)
                            job_started = True
                        print(f"🎯 {self.drone_id} arrived at job location, starting circular photography")
            
            elif self.flight_phase == "executing_circle":
                # Execute circular photography
                self._execute_current_job()
            
            # Flight control calculations
            yaw_disturbance = 0
            pitch_disturbance = 0
            
            if altitude > 5:  # Only calculate movement when airborne
                yaw_disturbance, pitch_disturbance = self._calculate_flight_control()
            
            # PID control for stable flight
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            
            # Altitude control
            target_alt = self.target_position[2] if len(self.target_position) > 2 else self.hover_altitude
            clamped_difference_altitude = clamp(target_alt - altitude + self.K_VERTICAL_OFFSET, -1, 1)
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


# Create and run drone
if __name__ == "__main__":
    # You can specify a drone ID as a command line argument
    drone_id = None
    if len(sys.argv) > 1:
        drone_id = sys.argv[1]
    
    drone = CircularPhotographyDrone(drone_id)
    
    try:
        drone.run()
    except KeyboardInterrupt:
        print(f"\n🛑 {drone.drone_id} shutting down...")
    finally:
        # Clean shutdown
        coordinator = get_coordinator()
        coordinator.shutdown()