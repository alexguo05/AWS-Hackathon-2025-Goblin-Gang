# AirSim Drone Simulation Project

A comprehensive drone simulation system using Microsoft AirSim for data collection, analysis, and visualization.

## 🚁 Features

- **Multiple Flight Patterns**: Hover, circular, and waypoint-based flight patterns
- **Data Collection**: Telemetry data and camera images (front and depth)
- **Real-time Analysis**: Live data processing and visualization
- **Comprehensive Logging**: Detailed flight logs and session tracking
- **Jupyter Integration**: Interactive analysis notebooks

## 📁 Project Structure

```
airsim_drone_sim/
├── src/                    # Source code
│   ├── drone_simulator.py  # Main simulation class
│   └── data_analyzer.py    # Data analysis tools
├── config/                 # Configuration files
│   └── settings.json       # Simulation parameters
├── scripts/                # Executable scripts
│   └── run_simulation.py   # Main simulation runner
├── notebooks/              # Jupyter notebooks
│   └── analysis_demo.ipynb # Data analysis demo
├── data/                   # Collected data (created at runtime)
│   ├── images/            # Camera images
│   ├── telemetry/         # Flight data CSV files
│   └── logs/              # Simulation logs
├── requirements.txt        # Python dependencies
└── README.md              # This file
```

## 🚀 Quick Start

### Prerequisites

1. **Install AirSim**: Download and install Microsoft AirSim from [GitHub](https://github.com/Microsoft/AirSim)
2. **Python 3.8+**: Ensure Python 3.8 or higher is installed
3. **AirSim Simulator**: Start the AirSim simulator with a drone-enabled environment

### Installation

1. **Clone and navigate to the project**:
   ```bash
   cd airsim_drone_sim
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Start AirSim**: Launch AirSim simulator and load a drone-enabled environment

### Running Simulations

#### Basic Simulation
```bash
python scripts/run_simulation.py
```

#### Specific Flight Pattern
```bash
python scripts/run_simulation.py --pattern circle
```

#### With Analysis and Report
```bash
python scripts/run_simulation.py --pattern waypoint --analyze --report
```

#### Available Flight Patterns
- `hover`: Hover at a fixed altitude
- `circle`: Fly in a circular pattern
- `waypoint`: Follow predefined waypoints

## 📊 Data Collection

The simulator automatically collects:

### Telemetry Data
- Position (x, y, z coordinates)
- Orientation (quaternion)
- Linear and angular velocities
- Timestamp for each data point

### Camera Data
- **Front Camera**: RGB images from the drone's front camera
- **Depth Camera**: Depth visualization images

### Data Storage
- **CSV Files**: Telemetry data stored in `data/telemetry/`
- **Images**: Camera images stored in `data/images/`
- **Logs**: Simulation logs stored in `data/logs/`

## 🔧 Configuration

Edit `config/settings.json` to customize:

### Simulation Parameters
```json
{
  "simulation": {
    "drone_name": "Drone1",
    "simulation_time": 60,
    "data_collection_interval": 0.1,
    "max_altitude": 100,
    "min_altitude": 10,
    "max_speed": 20
  }
}
```

### Camera Settings
```json
{
  "camera": {
    "front_camera": {
      "enabled": true,
      "image_type": "Scene"
    },
    "depth_camera": {
      "enabled": true,
      "image_type": "DepthVis"
    }
  }
}
```

### Flight Patterns
```json
{
  "flight_patterns": {
    "waypoint": {
      "points": [
        {"x": 0, "y": 0, "z": -20},
        {"x": 50, "y": 0, "z": -20},
        {"x": 50, "y": 50, "z": -20}
      ]
    }
  }
}
```

## 📈 Data Analysis

### Using the Analysis Tools

```python
from src.data_analyzer import DataAnalyzer

# Initialize analyzer
analyzer = DataAnalyzer('data')

# Load telemetry data
telemetry_data = analyzer.load_telemetry_data()

# Analyze flight statistics
stats = analyzer.analyze_flight_path()

# Create visualizations
analyzer.plot_flight_path('flight_analysis.png')

# Generate report
report = analyzer.create_flight_report('flight_report.md')
```

### Jupyter Notebook Analysis

Open `notebooks/analysis_demo.ipynb` for interactive data analysis:

```bash
jupyter notebook notebooks/analysis_demo.ipynb
```

## 🛠️ API Reference

### DroneSimulator Class

Main simulation class for controlling drone flights.

#### Methods
- `connect()`: Connect to AirSim simulator
- `takeoff(altitude)`: Take off to specified altitude
- `land()`: Land the drone
- `hover(duration, altitude)`: Hover for specified duration
- `fly_circle(radius, altitude, speed)`: Fly in circular pattern
- `fly_waypoints(waypoints, speed)`: Fly to waypoints
- `run_simulation(pattern)`: Run complete simulation

### DataAnalyzer Class

Tools for analyzing collected flight data.

#### Methods
- `load_telemetry_data(session_id)`: Load telemetry data
- `analyze_flight_path()`: Calculate flight statistics
- `plot_flight_path(save_path)`: Create flight path visualization
- `analyze_images(session_id)`: Analyze collected images
- `create_flight_report(session_id, output_path)`: Generate report

## 🐛 Troubleshooting

### Common Issues

1. **Connection Failed**: Ensure AirSim is running and accessible
2. **No Data Collected**: Check camera settings in configuration
3. **Import Errors**: Verify all dependencies are installed
4. **Permission Errors**: Ensure write permissions for data directory

### Debug Mode

Enable detailed logging by modifying the log level in the configuration:

```json
{
  "logging": {
    "level": "DEBUG"
  }
}
```

## 📝 License

This project is part of the AWS Hackathon 2025 - Goblin Gang submission.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## 📞 Support

For issues and questions:
- Check the troubleshooting section
- Review AirSim documentation
- Open an issue in the repository
