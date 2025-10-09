"""
Supervisor Controller for Circle Patrol Mission

This supervisor:
1. Spawns a single drone
2. Generates circle patrol tasks (waypoint-based circular flight)
3. Assigns tasks to the drone via JSON file
4. Monitors mission progress
"""

from controller import Supervisor
import sys
import os
import json

class CirclePatrolMissionSupervisor(Supervisor):
    """Supervisor that manages circle patrol missions."""
    
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # Mission configuration
        self.num_drones = 1  # Single drone for now
        self.spawn_position = [0, 0, 0.3]  # Spawn location (always 0, 0)
        self.drones = []  # List to store drone references
        
        # Circle patrol parameters
        self.target_center = [-25.35, 0]  # Center of main area
        self.circle_radius = 25.0  # Radius of each sub-circle (reduced from 50m)
        self.num_waypoints = 8  # Number of waypoints per circle
        self.num_circles = 4  # Number of overlapping sub-circles
        self.overlap_percentage = 0.5  # 50% overlap between circles
        self.camera_yaw = 90  # Camera faces 90 degrees right
        self.camera_pitch = 45  # Camera tilted down 45 degrees
        
        # Task directory (use absolute path relative to project root)
        # Controllers run from their own directories, so we need to go up to project root
        controller_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(controller_dir))
        self.task_dir = os.path.join(project_root, "lawnmower_tasks")
        os.makedirs(self.task_dir, exist_ok=True)
        
        print("=" * 70)
        print("🎯 CIRCLE PATROL MISSION SUPERVISOR")
        print("=" * 70)
        print(f"📊 Configuration:")
        print(f"   Number of drones: {self.num_drones}")
        print(f"   Spawn position: {self.spawn_position}")
        print(f"   Main area center: {self.target_center}")
        print(f"   Sub-circle radius: {self.circle_radius}m")
        print(f"   Number of sub-circles: {self.num_circles}")
        print(f"   Waypoints per circle: {self.num_waypoints}")
        print(f"   Overlap percentage: {self.overlap_percentage*100}%")
        print(f"   Camera orientation: yaw={self.camera_yaw}°, pitch={self.camera_pitch}°")
        print(f"   Task directory: {self.task_dir}")
        print("=" * 70)
    
    def generate_circle_tasks(self, drone_id):
        """Generate multiple overlapping sub-circle patrol tasks."""
        import math
        
        tasks = []
        
        print(f"\n📋 Generating multi-circle patrol tasks for drone {drone_id}:")
        print(f"   Main area center: {self.target_center}")
        print(f"   Sub-circle radius: {self.circle_radius}m")
        print(f"   Number of sub-circles: {self.num_circles}")
        print(f"   Overlap: {self.overlap_percentage*100}%")
        
        # Calculate circle centers with 50% overlap
        # For 50% overlap: distance between centers = radius (not 2*radius)
        circle_spacing = self.circle_radius * (1 - self.overlap_percentage)  # 12.5m for 50% overlap
        
        # Generate circle centers in a grid pattern
        circle_centers = []
        grid_size = int(math.ceil(math.sqrt(self.num_circles)))
        
        for i in range(self.num_circles):
            row = i // grid_size
            col = i % grid_size
            
            # Calculate center position relative to main target
            center_x = self.target_center[0] + (col - grid_size/2 + 0.5) * circle_spacing
            center_y = self.target_center[1] + (row - grid_size/2 + 0.5) * circle_spacing
            
            circle_centers.append([center_x, center_y])
        
        print(f"   Circle centers: {circle_centers}")
        
        # Generate waypoints for each circle
        for circle_idx, circle_center in enumerate(circle_centers):
            print(f"\n   🎯 Generating Circle {circle_idx + 1}/{self.num_circles} at ({circle_center[0]:.1f}, {circle_center[1]:.1f})")
            
            # Add transition task to move to circle center first
            if circle_idx == 0:
                # First circle - go directly to first waypoint
                first_angle = 0
            else:
                # Subsequent circles - add transition waypoint to circle center
                tasks.append({
                    "type": "transition",
                    "position": circle_center,
                    "camera_yaw": self.camera_yaw,
                    "camera_pitch": self.camera_pitch,
                    "description": f"Transition to Circle {circle_idx + 1} center",
                    "circle_id": circle_idx + 1
                })
                print(f"      Transition task: Move to Circle {circle_idx + 1} center")
                first_angle = 0
            
            # Generate waypoints around this circle
            for waypoint_idx in range(self.num_waypoints):
                angle = 2 * math.pi * waypoint_idx / self.num_waypoints
                x = circle_center[0] + self.circle_radius * math.cos(angle)
                y = circle_center[1] + self.circle_radius * math.sin(angle)
                
                # Create waypoint task
                tasks.append({
                    "type": "circle_waypoint",
                    "position": [x, y],
                    "camera_yaw": self.camera_yaw,
                    "camera_pitch": self.camera_pitch,
                    "description": f"Circle {circle_idx + 1} - Waypoint {waypoint_idx + 1}/{self.num_waypoints}: ({x:.1f}, {y:.1f})",
                    "circle_id": circle_idx + 1,
                    "waypoint_id": waypoint_idx + 1
                })
                print(f"      Waypoint {waypoint_idx + 1}: ({x:.1f}, {y:.1f})")
        
        print(f"\n✅ Generated {len(tasks)} total tasks across {self.num_circles} circles")
        print(f"   - Transition tasks: {self.num_circles - 1}")
        print(f"   - Circle waypoint tasks: {self.num_circles * self.num_waypoints}")
        return tasks
    
    def assign_tasks_to_drone(self, drone_id, tasks):
        """Write tasks to JSON file for drone to read."""
        task_file = os.path.join(self.task_dir, f"drone_{drone_id}_tasks.json")
        
        data = {
            "drone_id": drone_id,
            "mission_type": "circle_patrol",
            "num_tasks": len(tasks),
            "tasks": tasks
        }
        
        try:
            with open(task_file, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"✅ Tasks assigned to drone {drone_id} via {task_file}")
            return True
        except Exception as e:
            print(f"❌ Error assigning tasks: {e}")
            return False
        
    def spawn_drones(self):
        """Spawn single drone at spawn position."""
        print("\n🚁 Spawning drone...")
        
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
            
            # Spawn single drone at spawn_position
            x, y, z = self.spawn_position
            
            # Create drone definition string with high-quality camera settings
            drone_def = f"""DEF DRONE_0 Mavic2Pro {{
  translation {x} {y} {z}
  rotation 0 0 1 0
  name "drone_0"
  controller "patrol_with_images"
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
            
            print(f"   📝 Creating drone_0 at spawn position...")
            
            # Import drone into the world
            try:
                children_field.importMFNodeFromString(-1, drone_def)
                print(f"   ✅ Spawned drone_0 at ({x:.1f}, {y:.1f}, {z:.1f})")
                print(f"   📍 Drone will fly to first task start: ({self.first_task_start_x:.1f}, {self.first_task_start_y:.1f})")
            except Exception as e:
                print(f"   ❌ ERROR spawning drone_0: {e}")
                return False
            
            print(f"\n✅ Drone spawned successfully!")
            return True
            
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
        print("\n🚀 Starting multi-circle patrol mission...")
        
        # Generate and assign tasks BEFORE spawning drone
        print("\n📋 Generating circle patrol tasks...")
        tasks = self.generate_circle_tasks(drone_id=0)
        
        print("\n📤 Writing tasks to file...")
        if not self.assign_tasks_to_drone(drone_id=0, tasks=tasks):
            print("❌ Failed to assign tasks!")
            return
        
        print(f"✅ Tasks ready: {len(tasks)} tasks written to file")
        
        # Now spawn the drone (it will load tasks immediately on init)
        print("\n🚁 Spawning drone (will load tasks on startup)...")
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
        print(f"📋 Total tasks: {len(tasks)}")
        print(f"⏱️  Monitoring every 5s")
        print("\n" + "=" * 70)
        
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
                        print(f"✅ Drone completed multi-circle patrol mission")
                        print(f"⏱️  Total mission time: {current_time:.1f} seconds")
                        print("=" * 70)
                        break
            except:
                pass

if __name__ == "__main__":
    supervisor = CirclePatrolMissionSupervisor()
    supervisor.run()
