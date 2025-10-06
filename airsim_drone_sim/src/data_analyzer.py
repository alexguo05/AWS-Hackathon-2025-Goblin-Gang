"""
Data Analysis Module for AirSim Drone Simulation
Provides tools for analyzing collected flight data and images.
"""

import pandas as pd
import numpy as np
import cv2
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import json
from typing import Dict, List, Tuple, Optional
import logging


class DataAnalyzer:
    """Analyze collected drone simulation data."""
    
    def __init__(self, data_path: str = "data"):
        """Initialize data analyzer with path to collected data."""
        self.data_path = Path(data_path)
        self.telemetry_data = None
        self.logger = logging.getLogger(__name__)
    
    def load_telemetry_data(self, session_id: str = None) -> pd.DataFrame:
        """Load telemetry data from CSV file."""
        try:
            telemetry_dir = self.data_path / 'telemetry'
            
            if session_id:
                filepath = telemetry_dir / f"telemetry_{session_id}.csv"
            else:
                # Load most recent file
                csv_files = list(telemetry_dir.glob("telemetry_*.csv"))
                if not csv_files:
                    raise FileNotFoundError("No telemetry files found")
                filepath = max(csv_files, key=lambda x: x.stat().st_mtime)
            
            self.telemetry_data = pd.read_csv(filepath)
            self.telemetry_data['timestamp'] = pd.to_datetime(self.telemetry_data['timestamp'])
            
            self.logger.info(f"Loaded telemetry data: {len(self.telemetry_data)} records")
            return self.telemetry_data
            
        except Exception as e:
            self.logger.error(f"Failed to load telemetry data: {e}")
            raise
    
    def analyze_flight_path(self) -> Dict:
        """Analyze the flight path and generate statistics."""
        if self.telemetry_data is None:
            raise ValueError("No telemetry data loaded")
        
        # Calculate flight statistics
        stats = {
            'total_distance': self._calculate_total_distance(),
            'max_altitude': abs(self.telemetry_data['position_z'].min()),  # Negative Z is up
            'min_altitude': abs(self.telemetry_data['position_z'].max()),
            'avg_speed': self._calculate_average_speed(),
            'max_speed': self._calculate_max_speed(),
            'flight_duration': self._calculate_flight_duration(),
            'total_points': len(self.telemetry_data)
        }
        
        return stats
    
    def _calculate_total_distance(self) -> float:
        """Calculate total distance traveled."""
        positions = self.telemetry_data[['position_x', 'position_y', 'position_z']].values
        distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
        return np.sum(distances)
    
    def _calculate_average_speed(self) -> float:
        """Calculate average speed during flight."""
        velocities = self.telemetry_data[['linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z']].values
        speeds = np.sqrt(np.sum(velocities**2, axis=1))
        return np.mean(speeds)
    
    def _calculate_max_speed(self) -> float:
        """Calculate maximum speed during flight."""
        velocities = self.telemetry_data[['linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z']].values
        speeds = np.sqrt(np.sum(velocities**2, axis=1))
        return np.max(speeds)
    
    def _calculate_flight_duration(self) -> float:
        """Calculate total flight duration in seconds."""
        if len(self.telemetry_data) < 2:
            return 0.0
        
        start_time = self.telemetry_data['timestamp'].iloc[0]
        end_time = self.telemetry_data['timestamp'].iloc[-1]
        return (end_time - start_time).total_seconds()
    
    def plot_flight_path(self, save_path: str = None) -> plt.Figure:
        """Plot the 3D flight path."""
        if self.telemetry_data is None:
            raise ValueError("No telemetry data loaded")
        
        fig = plt.figure(figsize=(12, 8))
        
        # 3D flight path
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.plot(self.telemetry_data['position_x'], 
                self.telemetry_data['position_y'], 
                self.telemetry_data['position_z'])
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_zlabel('Z Position (m)')
        ax1.set_title('3D Flight Path')
        
        # 2D top view
        ax2 = fig.add_subplot(222)
        ax2.plot(self.telemetry_data['position_x'], self.telemetry_data['position_y'])
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Top View Flight Path')
        ax2.grid(True)
        
        # Altitude over time
        ax3 = fig.add_subplot(223)
        ax3.plot(self.telemetry_data['timestamp'], abs(self.telemetry_data['position_z']))
        ax3.set_xlabel('Time')
        ax3.set_ylabel('Altitude (m)')
        ax3.set_title('Altitude Over Time')
        ax3.tick_params(axis='x', rotation=45)
        
        # Speed over time
        ax4 = fig.add_subplot(224)
        velocities = self.telemetry_data[['linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z']].values
        speeds = np.sqrt(np.sum(velocities**2, axis=1))
        ax4.plot(self.telemetry_data['timestamp'], speeds)
        ax4.set_xlabel('Time')
        ax4.set_ylabel('Speed (m/s)')
        ax4.set_title('Speed Over Time')
        ax4.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            self.logger.info(f"Flight path plot saved to {save_path}")
        
        return fig
    
    def analyze_images(self, session_id: str = None) -> Dict:
        """Analyze collected images and generate statistics."""
        try:
            image_dir = self.data_path / 'images'
            
            if session_id:
                session_dir = image_dir / session_id
            else:
                # Find most recent session
                session_dirs = [d for d in image_dir.iterdir() if d.is_dir()]
                if not session_dirs:
                    raise FileNotFoundError("No image directories found")
                session_dir = max(session_dirs, key=lambda x: x.stat().st_mtime)
            
            # Count images by camera type
            image_stats = {}
            for camera_type in ['front', 'depth']:
                camera_images = list(session_dir.glob(f"{camera_type}_*.png"))
                image_stats[camera_type] = {
                    'count': len(camera_images),
                    'files': [str(f) for f in camera_images]
                }
            
            # Analyze image properties (using first few images as sample)
            if image_stats['front']['count'] > 0:
                sample_image = cv2.imread(image_stats['front']['files'][0])
                if sample_image is not None:
                    image_stats['front']['resolution'] = sample_image.shape[:2]
                    image_stats['front']['channels'] = sample_image.shape[2] if len(sample_image.shape) > 2 else 1
            
            if image_stats['depth']['count'] > 0:
                sample_image = cv2.imread(image_stats['depth']['files'][0])
                if sample_image is not None:
                    image_stats['depth']['resolution'] = sample_image.shape[:2]
                    image_stats['depth']['channels'] = sample_image.shape[2] if len(sample_image.shape) > 2 else 1
            
            self.logger.info(f"Image analysis completed for session {session_dir.name}")
            return image_stats
            
        except Exception as e:
            self.logger.error(f"Image analysis failed: {e}")
            raise
    
    def create_flight_report(self, session_id: str = None, output_path: str = None) -> str:
        """Create a comprehensive flight report."""
        try:
            # Load data
            self.load_telemetry_data(session_id)
            
            # Analyze flight
            flight_stats = self.analyze_flight_path()
            image_stats = self.analyze_images(session_id)
            
            # Generate report
            report = f"""
# Flight Simulation Report
## Session ID: {session_id or 'Latest'}

## Flight Statistics
- **Total Distance**: {flight_stats['total_distance']:.2f} meters
- **Max Altitude**: {flight_stats['max_altitude']:.2f} meters
- **Min Altitude**: {flight_stats['min_altitude']:.2f} meters
- **Average Speed**: {flight_stats['avg_speed']:.2f} m/s
- **Max Speed**: {flight_stats['max_speed']:.2f} m/s
- **Flight Duration**: {flight_stats['flight_duration']:.2f} seconds
- **Data Points**: {flight_stats['total_points']}

## Image Collection
- **Front Camera Images**: {image_stats['front']['count']}
- **Depth Camera Images**: {image_stats['depth']['count']}
- **Front Camera Resolution**: {image_stats['front'].get('resolution', 'N/A')}
- **Depth Camera Resolution**: {image_stats['depth'].get('resolution', 'N/A')}

## Data Quality
- **Telemetry Completeness**: {self._calculate_data_completeness():.1f}%
- **Image Collection Rate**: {self._calculate_image_rate():.1f} images/second

---
*Report generated on {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}*
"""
            
            if output_path:
                with open(output_path, 'w') as f:
                    f.write(report)
                self.logger.info(f"Flight report saved to {output_path}")
            
            return report
            
        except Exception as e:
            self.logger.error(f"Failed to create flight report: {e}")
            raise
    
    def _calculate_data_completeness(self) -> float:
        """Calculate percentage of complete telemetry records."""
        if self.telemetry_data is None:
            return 0.0
        
        required_columns = ['position_x', 'position_y', 'position_z', 'timestamp']
        complete_records = self.telemetry_data[required_columns].notna().all(axis=1).sum()
        return (complete_records / len(self.telemetry_data)) * 100
    
    def _calculate_image_rate(self) -> float:
        """Calculate image collection rate."""
        if self.telemetry_data is None:
            return 0.0
        
        flight_duration = self._calculate_flight_duration()
        if flight_duration == 0:
            return 0.0
        
        # Estimate based on data collection interval
        expected_images = flight_duration / 0.1  # Assuming 0.1s interval
        return expected_images / flight_duration


if __name__ == "__main__":
    # Example usage
    analyzer = DataAnalyzer()
    
    try:
        # Load and analyze data
        analyzer.load_telemetry_data()
        
        # Generate statistics
        stats = analyzer.analyze_flight_path()
        print("Flight Statistics:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        # Create plots
        analyzer.plot_flight_path("flight_analysis.png")
        
        # Generate report
        report = analyzer.create_flight_report(output_path="flight_report.md")
        print("\nFlight report generated!")
        
    except Exception as e:
        print(f"Analysis failed: {e}")
