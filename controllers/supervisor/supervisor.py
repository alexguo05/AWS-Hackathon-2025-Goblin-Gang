"""
Supervisor Controller for Lawnmower Mapping Mission

This supervisor:
1. Spawns a single drone
2. Generates lawnmower pattern tasks (systematic grid scanning)
3. Assigns tasks to the drone via JSON file
4. Monitors mission progress
"""

from controller import Supervisor
import sys
import os
import json
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

class LawnmowerMissionSupervisor(Supervisor):
    """Supervisor that manages lawnmower pattern mapping missions."""
    
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # Mission configuration
        self.num_drones = 3  # Number of drones to spawn
        self.spawn_position = [0, 0, 0.3]  # Spawn location (always 0, 0)
        self.drones = []  # List to store drone references
        
        # Grid bounds: x: -15 to -50, y: -10 to 30
        self.grid_x_min = -50
        self.grid_x_max = -15
        self.grid_y_min = -10
        self.grid_y_max = 30
        
        # Calculate first task start position (corner closest to 0,0, offset by 10m outside)
        # Corner closest to (0,0) is (grid_x_max, grid_y_min) = (-15, -10)
        self.first_task_start_x = self.grid_x_max + 10  # -15 + 10 = -5
        self.first_task_start_y = self.grid_y_min + 10  # -10 + 10 = 0
        
        # Lawnmower pattern parameters
        self.scan_y_increment = 20  # Y spacing between scan lines
        self.camera_yaw = 90  # Camera faces 90 degrees right
        self.camera_pitch = 45  # Camera tilted down 45 degrees
        
        # MQTT client (optional; used to publish tasks to drones)
        self.mqtt_client = None
        if mqtt is not None:
            try:
                self.mqtt_client = mqtt.Client()
                self.mqtt_client.connect("localhost", 1883, 60)
                self.mqtt_client.loop_start()
                print("🔌 MQTT: Connected to localhost:1883")
            except Exception as e:
                print(f"⚠️  MQTT connect failed: {e}")
                self.mqtt_client = None

        print("=" * 70)
        print("🎯 LAWNMOWER MAPPING MISSION SUPERVISOR")
        print("=" * 70)
        print(f"📊 Configuration:")
        print(f"   Number of drones: {self.num_drones}")
        print(f"   Spawn position: {self.spawn_position}")
        print(f"   First task start: ({self.first_task_start_x}, {self.first_task_start_y})")
        print(f"   Grid bounds: X[{self.grid_x_min}, {self.grid_x_max}], Y[{self.grid_y_min}, {self.grid_y_max}]")
        print(f"   Camera orientation: yaw={self.camera_yaw}°, pitch={self.camera_pitch}°")
        print("=" * 70)
        
    def spawn_drones(self):
        """Spawn multiple drones at/near the spawn position."""
        print(f"\n🚁 Spawning {self.num_drones} drones...")
        
        try:
            # Get the root node
            root = self.getRoot()
            if root is None:
                print("   ❌ ERROR: Could not get root node!")
                print("   Make sure the supervisor has 'supervisor TRUE' field set")
                return False
            
            children_field = root.getField('children')
            if children_field is None:
                print("   ❌ ERROR: Could not get children field!")
                return False
            
            # Spawn multiple drones with slight Y offsets to avoid overlap
            x0, y0, z0 = self.spawn_position
            any_spawned = False
            for i in range(self.num_drones):
                x = x0
                y = y0 + i * 1.0
                z = z0
                
                drone_def = f"""DEF DRONE_{i} Mavic2Pro {{
  translation {x} {y} {z}
  rotation 0 0 1 0
  name "drone_{i}"
  controller "patrol_dynamic"
  controllerArgs []
  cameraSlot [
    Camera {{
      width 1920
      height 1080
      antiAliasing TRUE
      motionBlur 0
      noise 0
      lens Lens {{
        radialCoefficients 0 0
        tangentialCoefficients 0 0
      }}
    }}
  ]
}}"""
                
                print(f"   📝 Creating drone_{i} at spawn vicinity...")
                try:
                    children_field.importMFNodeFromString(-1, drone_def)
                    print(f"   ✅ Spawned drone_{i} at ({x:.1f}, {y:.1f}, {z:.1f})")
                    print(f"   📍 Drone will fly to first task start: ({self.first_task_start_x:.1f}, {self.first_task_start_y:.1f})")
                    any_spawned = True
                except Exception as e:
                    print(f"   ❌ ERROR spawning drone_{i}: {e}")
                    continue
            
            if any_spawned:
                print(f"\n✅ Drones spawned successfully!")
                return True
            else:
                return False
            
        except Exception as e:
            print(f"   ❌ CRITICAL ERROR in spawn_drones: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def monitor_mission(self):
        """Monitor the mission progress."""
        # Simplified monitoring - just check if drone exists
        try:
            drone_node = self.getFromDef("DRONE_0")
            if drone_node:
                # Drone exists, mission is running (details printed by drone itself)
                pass
        except Exception as e:
            pass
    
    def run(self):
        """Main supervisor control loop."""
        print("\n🚀 Starting lawnmower mapping mission...")
        
        # Now spawn the drones (they will load tasks on startup)
        print("\n🚁 Spawning drones (will load tasks on startup)...")
        spawn_success = self.spawn_drones()
        
        if not spawn_success:
            print("\n❌ Failed to spawn drone. Exiting...")
            print("\n🔧 Troubleshooting:")
            print("   1. Make sure supervisor robot has 'supervisor TRUE' field")
            print("   2. Check that Mavic2Pro PROTO is in IMPORTABLE EXTERNPROTO list")
            print("   3. Verify patrol_dynamic controller exists")
            print("   4. Check console for specific error messages")
            return
        
        # Wait for drone to initialize
        print("\n⏳ Waiting for drone to initialize...")
        for _ in range(30):
            self.step(self.time_step)
        
        print("\n✅ Mission started!")
        print(f"⏱️  Monitoring every 5s")
        print("\n" + "=" * 70)

        # After drones initialized, publish circle task to each via MQTT
        if self.mqtt_client is not None:
            try:
                centers = {
                    0: [-30.0, -9.0],
                    1: [-30.0, 19.0],
                    2: [ -6.0,  4.0],
                }
                radius = 30.0
                base_altitude = 35.0
                for drone_id in range(self.num_drones):
                    topic = f"tasks/drone_{drone_id}"
                    center = centers.get(drone_id, [0.0, 0.0])
                    altitude = base_altitude + 2.0 * float(drone_id)
                    payload = {
                        "id": f"circle-r{int(radius)}-{drone_id}",
                        "type": "nav.circle",
                        "args": {"center": center, "radius": radius, "altitude": altitude}
                    }
                    self.mqtt_client.publish(topic, json.dumps(payload), qos=1)
                    print(f"📤 MQTT task -> {topic}: {payload}")
            except Exception as e:
                print(f"⚠️  MQTT publish failed: {e}")
        
        # Main monitoring loop
        last_monitor_time = 0
        monitor_interval = 5.0  # Monitor every 5 seconds
        
        while self.step(self.time_step) != -1:
            current_time = self.getTime()
            
            # Periodic status monitoring
            if current_time - last_monitor_time >= monitor_interval:
                self.monitor_mission()
                last_monitor_time = current_time
            
            # Check if drone has landed (mission complete)
            try:
                drone_node = self.getFromDef("DRONE_0")
                if drone_node:
                    pos = drone_node.getPosition()
                    altitude = pos[2]
                    if altitude < 0.5 and current_time > 60.0:  # After at least 60 seconds
                        print("\n" + "=" * 70)
                        print("🎉 MISSION COMPLETE!")
                        print("=" * 70)
                        print(f"✅ Drone completed lawnmower mapping mission")
                        print(f"⏱️  Total mission time: {current_time:.1f} seconds")
                        print("=" * 70)
                        break
            except:
                pass

if __name__ == "__main__":
    supervisor = LawnmowerMissionSupervisor()
    supervisor.run()
