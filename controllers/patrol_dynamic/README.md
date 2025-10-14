# Simple Patrol Dynamic Controller

## 🎯 Overview

A simplified drone controller that demonstrates basic takeoff and landing sequences. Perfect for testing the supervisor's multi-drone spawning.

## 🚁 Flight Sequence

```
1. TAKEOFF   → Rise to 10 meters
2. HOVER     → Stay at 10m for 5 seconds
3. LANDING   → Descend to ground
4. LANDED    → Motors off, mission complete
```

## ⚙️ Configuration

Edit these values in `patrol_dynamic.py`:

```python
self.target_altitude = 10.0    # Target altitude (meters)
self.hover_duration = 5.0      # How long to hover (seconds)
self.landing_altitude = 0.5    # When to consider "landed" (meters)
```

## 📊 Console Output

```
🚁 Drone 0 initialized
   Target altitude: 10.0m
   Hover duration: 5.0s

🚀 Drone 0: Starting mission...
   Phase: TAKEOFF

📊 Drone 0: Phase=TAKEOFF, Alt=2.3m, Target=10.0m
📊 Drone 0: Phase=TAKEOFF, Alt=5.8m, Target=10.0m
📊 Drone 0: Phase=TAKEOFF, Alt=9.2m, Target=10.0m

✅ Drone 0: Reached 10.0m - HOVERING

📊 Drone 0: Phase=HOVER, Alt=10.1m, Target=10.0m
📊 Drone 0: Phase=HOVER, Alt=10.0m, Target=10.0m

🛬 Drone 0: Hover complete - LANDING

📊 Drone 0: Phase=LANDING, Alt=7.5m, Target=0.0m
📊 Drone 0: Phase=LANDING, Alt=3.2m, Target=0.0m

✅ Drone 0: LANDED - Mission complete!
```

## 🎮 Using with Supervisor

The supervisor will spawn multiple drones, each running this controller:

```
Supervisor spawns:
├─ drone_0 → patrol_dynamic.py → Takeoff, hover, land
├─ drone_1 → patrol_dynamic.py → Takeoff, hover, land
└─ drone_2 → patrol_dynamic.py → Takeoff, hover, land

Each drone operates independently!
```

## 🔧 Flight Phases Explained

### 1. **TAKEOFF Phase**
- **Goal**: Reach target altitude (10m)
- **Control**: `target_alt = 10.0`
- **Exit condition**: `altitude > 10.0 - 1` (within 1m)

### 2. **HOVER Phase**
- **Goal**: Maintain altitude for set duration
- **Control**: `target_alt = 10.0`
- **Timer**: Starts when TAKEOFF complete
- **Exit condition**: `current_time - hover_start >= 5.0s`

### 3. **LANDING Phase**
- **Goal**: Descend to ground safely
- **Control**: `target_alt = 0.0`
- **Exit condition**: `altitude < 0.5m`

### 4. **LANDED Phase**
- **Goal**: Stay on ground, mission complete
- **Control**: Motors set to 0
- **Exit condition**: None (final state)

## 📐 Control System

Same PID-like control as `patrol_with_images.py`:

```python
# Altitude control (cubic for smooth approach)
altitude_error = target_alt - current_alt
vertical_input = K_VERTICAL_P * pow(altitude_error, 3.0)

# Stabilization (proportional + damping)
roll_input = K_ROLL_P * roll + roll_acceleration
pitch_input = K_PITCH_P * pitch + pitch_acceleration

# Motor mixing (quadcopter configuration)
FL = base_thrust + vertical - yaw + pitch - roll
FR = base_thrust + vertical + yaw + pitch + roll
RL = base_thrust + vertical + yaw - pitch - roll
RR = base_thrust + vertical - yaw - pitch + roll
```

## 🎯 Key Features

- ✅ **Simple**: Only ~160 lines of code
- ✅ **Autonomous**: Each drone independent
- ✅ **Safe**: Smooth takeoff and landing
- ✅ **Configurable**: Easy to adjust parameters
- ✅ **Multi-drone ready**: Works with supervisor

## 🚀 Next Steps

To add more functionality:

1. **Add waypoint navigation**: Copy from `patrol_with_images.py`
2. **Add image capture**: Copy camera code
3. **Add grid mapping**: Implement grid coverage algorithm
4. **Add coordination**: Use shared JSON files for multi-drone

Current version is intentionally simple for testing the supervisor spawning system!

## 📝 Technical Notes

- **Time step**: 32ms (from `getBasicTimeStep()`)
- **Control rate**: Every simulation step
- **Status updates**: Every 5 seconds
- **Motor control**: Standard quadcopter mixing
- **Coordinate system**: Webots standard (X, Y, Z/altitude)

## 🔍 Comparison with patrol_with_images.py

| Feature | patrol_dynamic | patrol_with_images |
|---------|----------------|-------------------|
| Takeoff | ✅ Simple | ✅ Same logic |
| Hover | ✅ Timed | ❌ No hover |
| Landing | ✅ Automatic | ❌ No landing |
| Waypoints | ❌ None | ✅ Circle patrol |
| Images | ❌ None | ✅ Periodic capture |
| Complexity | Simple | Full-featured |

This controller is a **simplified version** focused on demonstrating the basic flight sequence for multi-drone testing.
