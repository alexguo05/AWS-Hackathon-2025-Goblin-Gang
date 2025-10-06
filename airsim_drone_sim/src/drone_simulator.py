"""
AirSim Drone Simulator
A comprehensive drone simulation system using AirSim for data collection and analysis.
"""

import airsim
import numpy as np
import pandas as pd
import cv2
import json
import os
import time
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import logging
from pathlib import Path


class DroneSimulator:
    """Main class for drone simulation and data collection."""
    
    def __init__(self, config_path: str = "config/settings.json"):
        """Initialize the drone simulator with configuration."""
        self.config = self._load_config(config_path)
        self.client = None
        self.data_log = []
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup logging
        self._setup_logging()
        
        # Create data directories
        self._create_directories()
    
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from JSON file."""
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.error(f"Configuration file not found: {config_path}")
            raise
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON in configuration file: {e}")
            raise
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_dir = Path(self.config['data_storage']['base_path']) / 'logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_dir / f'simulation_{self.session_id}.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
    
    def _create_directories(self):
        """Create necessary directories for data storage."""
        base_path = Path(self.config['data_storage']['base_path'])
        directories = ['images', 'telemetry', 'logs']
        
        for directory in directories:
            (base_path / directory).mkdir(parents=True, exist_ok=True)
    
    def connect(self) -> bool:
        """Connect to AirSim simulator."""
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            self.logger.info("Successfully connected to AirSim")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to AirSim: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from AirSim simulator."""
        if self.client:
            try:
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
                self.logger.info("Disconnected from AirSim")
            except Exception as e:
                self.logger.error(f"Error during disconnect: {e}")
    
    def takeoff(self, altitude: float = 20.0) -> bool:
        """Take off to specified altitude."""
        try:
            self.client.takeoffAsync().join()
            self.client.moveToZAsync(-altitude, 5).join()  # Negative Z is up in AirSim
            self.logger.info(f"Took off to altitude {altitude}m")
            return True
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            return False
    
    def land(self) -> bool:
        """Land the drone."""
        try:
            self.client.landAsync().join()
            self.logger.info("Landed successfully")
            return True
        except Exception as e:
            self.logger.error(f"Landing failed: {e}")
            return False
    
    def hover(self, duration: float, altitude: float = None) -> bool:
        """Hover at current position for specified duration."""
        try:
            if altitude:
                self.client.moveToZAsync(-altitude, 5).join()
            
            self.logger.info(f"Hovering for {duration} seconds")
            time.sleep(duration)
            return True
        except Exception as e:
            self.logger.error(f"Hover failed: {e}")
            return False
    
    def fly_circle(self, radius: float, altitude: float, speed: float) -> bool:
        """Fly in a circular pattern."""
        try:
            center = self.client.getMultirotorState().kinematics_estimated.position
            center = [center.x_val, center.y_val, center.z_val]
            
            num_points = 36  # 10-degree increments
            for i in range(num_points + 1):
                angle = 2 * np.pi * i / num_points
                x = center[0] + radius * np.cos(angle)
                y = center[1] + radius * np.sin(angle)
                z = -altitude  # Negative Z is up
                
                self.client.moveToPositionAsync(x, y, z, speed).join()
                self._collect_data()
                
            self.logger.info(f"Completed circular flight pattern (radius: {radius}m)")
            return True
        except Exception as e:
            self.logger.error(f"Circular flight failed: {e}")
            return False
    
    def fly_waypoints(self, waypoints: List[Dict], speed: float = 10.0) -> bool:
        """Fly to a series of waypoints."""
        try:
            for i, waypoint in enumerate(waypoints):
                x, y, z = waypoint['x'], waypoint['y'], -waypoint['z']  # Negative Z is up
                self.client.moveToPositionAsync(x, y, z, speed).join()
                self._collect_data()
                self.logger.info(f"Reached waypoint {i+1}/{len(waypoints)}")
            
            self.logger.info("Completed waypoint flight pattern")
            return True
        except Exception as e:
            self.logger.error(f"Waypoint flight failed: {e}")
            return False
    
    def _collect_data(self):
        """Collect sensor data and images."""
        try:
            # Get drone state
            state = self.client.getMultirotorState()
            position = state.kinematics_estimated.position
            orientation = state.kinematics_estimated.orientation
            
            # Get images
            images = self._capture_images()
            
            # Log telemetry data
            telemetry_data = {
                'timestamp': datetime.now().isoformat(),
                'position_x': position.x_val,
                'position_y': position.y_val,
                'position_z': position.z_val,
                'orientation_w': orientation.w_val,
                'orientation_x': orientation.x_val,
                'orientation_y': orientation.y_val,
                'orientation_z': orientation.z_val,
                'linear_velocity_x': state.kinematics_estimated.linear_velocity.x_val,
                'linear_velocity_y': state.kinematics_estimated.linear_velocity.y_val,
                'linear_velocity_z': state.kinematics_estimated.linear_velocity.z_val,
                'angular_velocity_x': state.kinematics_estimated.angular_velocity.x_val,
                'angular_velocity_y': state.kinematics_estimated.angular_velocity.y_val,
                'angular_velocity_z': state.kinematics_estimated.angular_velocity.z_val,
                'battery_level': state.battery_level if hasattr(state, 'battery_level') else None
            }
            
            self.data_log.append(telemetry_data)
            
            # Save images if enabled
            if self.config['data_storage']['save_images'] and images:
                self._save_images(images)
                
        except Exception as e:
            self.logger.error(f"Data collection failed: {e}")
    
    def _capture_images(self) -> Dict[str, np.ndarray]:
        """Capture images from all enabled cameras."""
        images = {}
        
        try:
            # Front camera
            if self.config['camera']['front_camera']['enabled']:
                responses = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                if responses[0]:
                    img1d = np.frombuffer(responses[0], dtype=np.uint8)
                    img_rgb = img1d.reshape(144, 256, 3)
                    images['front'] = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
            
            # Depth camera
            if self.config['camera']['depth_camera']['enabled']:
                responses = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False)
                ])
                if responses[0]:
                    img1d = np.frombuffer(responses[0], dtype=np.uint8)
                    img_depth = img1d.reshape(144, 256, 3)
                    images['depth'] = img_depth
                    
        except Exception as e:
            self.logger.error(f"Image capture failed: {e}")
        
        return images
    
    def _save_images(self, images: Dict[str, np.ndarray]):
        """Save captured images to disk."""
        try:
            image_dir = Path(self.config['data_storage']['base_path']) / 'images' / self.session_id
            image_dir.mkdir(parents=True, exist_ok=True)
            
            timestamp = datetime.now().strftime("%H%M%S_%f")[:-3]  # Include milliseconds
            
            for camera_name, image in images.items():
                filename = f"{camera_name}_{timestamp}.{self.config['data_storage']['image_format']}"
                filepath = image_dir / filename
                cv2.imwrite(str(filepath), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
        except Exception as e:
            self.logger.error(f"Image saving failed: {e}")
    
    def save_telemetry_data(self):
        """Save collected telemetry data to CSV."""
        try:
            if not self.data_log:
                self.logger.warning("No telemetry data to save")
                return
            
            telemetry_dir = Path(self.config['data_storage']['base_path']) / 'telemetry'
            telemetry_dir.mkdir(parents=True, exist_ok=True)
            
            df = pd.DataFrame(self.data_log)
            filename = f"telemetry_{self.session_id}.csv"
            filepath = telemetry_dir / filename
            df.to_csv(filepath, index=False)
            
            self.logger.info(f"Telemetry data saved to {filepath}")
            
        except Exception as e:
            self.logger.error(f"Telemetry saving failed: {e}")
    
    def run_simulation(self, flight_pattern: str = "waypoint") -> bool:
        """Run the complete simulation with specified flight pattern."""
        try:
            self.logger.info(f"Starting simulation with pattern: {flight_pattern}")
            
            if not self.connect():
                return False
            
            # Takeoff
            if not self.takeoff():
                return False
            
            # Execute flight pattern
            if flight_pattern == "hover":
                pattern_config = self.config['flight_patterns']['hover']
                self.hover(pattern_config['duration'], pattern_config['altitude'])
                
            elif flight_pattern == "circle":
                pattern_config = self.config['flight_patterns']['circle']
                self.fly_circle(
                    pattern_config['radius'],
                    pattern_config['altitude'],
                    pattern_config['speed']
                )
                
            elif flight_pattern == "waypoint":
                pattern_config = self.config['flight_patterns']['waypoint']
                self.fly_waypoints(pattern_config['points'])
            
            # Land
            self.land()
            
            # Save data
            self.save_telemetry_data()
            
            self.logger.info("Simulation completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Simulation failed: {e}")
            return False
        finally:
            self.disconnect()


if __name__ == "__main__":
    # Example usage
    simulator = DroneSimulator()
    
    # Run different flight patterns
    patterns = ["hover", "circle", "waypoint"]
    
    for pattern in patterns:
        print(f"\nRunning {pattern} pattern...")
        success = simulator.run_simulation(pattern)
        if success:
            print(f"{pattern} pattern completed successfully")
        else:
            print(f"{pattern} pattern failed")
        
        # Reset for next pattern
        simulator.data_log = []
        simulator.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
