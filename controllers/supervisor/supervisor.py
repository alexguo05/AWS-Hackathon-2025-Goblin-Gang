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

class LawnmowerMissionSupervisor(Supervisor):
    """Supervisor that manages lawnmower pattern mapping missions."""
    
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # Mission configuration
        self.num_drones = 1  # Single drone for now
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
        
        # Task directory (use absolute path relative to project root)
        # Controllers run from their own directories, so we need to go up to project root
        controller_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(controller_dir))
        self.task_dir = os.path.join(project_root, "lawnmower_tasks")
        os.makedirs(self.task_dir, exist_ok=True)
        
        print("=" * 70)
        print("🎯 LAWNMOWER MAPPING MISSION SUPERVISOR")
        print("=" * 70)
        print(f"📊 Configuration:")
        print(f"   Number of drones: {self.num_drones}")
        print(f"   Spawn position: {self.spawn_position}")
        print(f"   First task start: ({self.first_task_start_x}, {self.first_task_start_y})")
        print(f"   Grid bounds: X[{self.grid_x_min}, {self.grid_x_max}], Y[{self.grid_y_min}, {self.grid_y_max}]")
        print(f"   Camera orientation: yaw={self.camera_yaw}°, pitch={self.camera_pitch}°")
        print(f"   Task directory: {self.task_dir}")
        print("=" * 70)
    
    def generate_lawnmower_tasks(self, drone_id):
        """Generate lawnmower pattern scanning tasks (both X and Y axis)."""
        tasks = []
        
        # X-axis scan positions (horizontal lines at different Y values)
        y_positions = []
        current_y = self.grid_y_min
        while current_y <= self.grid_y_max:
            y_positions.append(current_y)
            current_y += self.scan_y_increment
        
        # Y-axis scan positions (vertical lines at different X values)
        x_positions = []
        current_x = self.grid_x_min
        while current_x <= self.grid_x_max:
            x_positions.append(current_x)
            current_x += self.scan_y_increment  # Use same spacing
        
        print(f"\n📋 Generating lawnmower tasks for drone {drone_id}:")
        print(f"   Grid: X[{self.grid_x_min}, {self.grid_x_max}], Y[{self.grid_y_min}, {self.grid_y_max}]")
        print(f"   Horizontal scan lines (Y positions): {y_positions}")
        print(f"   Vertical scan lines (X positions): {x_positions}")
        
        # ========== HORIZONTAL SCANS (X-axis movement) ==========
        print(f"\n   📍 Generating horizontal scans (moving along X-axis)...")
        for i, y in enumerate(y_positions):
            # We start outside the grid, scan left past the grid
            start_x = self.grid_x_max + 10  # -15 + 10 = -5 (outside, near side)
            end_x = self.grid_x_min - 10     # -50 - 10 = -60 (outside, far side)
            
            # Task: Scan from x=-5 to x=-60 (left, through the grid)
            tasks.append({
                "type": "scan",
                "start": [start_x, y],
                "end": [end_x, y],
                "camera_yaw": self.camera_yaw,
                "camera_pitch": self.camera_pitch,
                "description": f"Horizontal scan {i+1}: x={start_x} to {end_x}, y={y}"
            })
            print(f"      Task {len(tasks)}: scan from ({start_x}, {y}) to ({end_x}, {y})")
            
            # Task: Scan back from x=-60 to x=-5 (right, back through grid)
            tasks.append({
                "type": "scan",
                "start": [end_x, y],
                "end": [start_x, y],
                "camera_yaw": self.camera_yaw,
                "camera_pitch": self.camera_pitch,
                "description": f"Horizontal scan {i+1} return: x={end_x} to {start_x}, y={y}"
            })
            print(f"      Task {len(tasks)}: scan from ({end_x}, {y}) to ({start_x}, {y})")
        
        # ========== VERTICAL SCANS (Y-axis movement) ==========
        print(f"\n   📍 Generating vertical scans (moving along Y-axis)...")
        for i, x in enumerate(x_positions):
            # We start outside the grid, scan from bottom to top
            start_y = self.grid_y_min - 10  # -10 - 10 = -20 (outside, bottom)
            end_y = self.grid_y_max + 10    # 30 + 10 = 40 (outside, top)
            
            # Task: Scan from y=-20 to y=40 (upward, through the grid)
            tasks.append({
                "type": "scan",
                "start": [x, start_y],
                "end": [x, end_y],
                "camera_yaw": self.camera_yaw,
                "camera_pitch": self.camera_pitch,
                "description": f"Vertical scan {i+1}: y={start_y} to {end_y}, x={x}"
            })
            print(f"      Task {len(tasks)}: scan from ({x}, {start_y}) to ({x}, {end_y})")
            
            # Task: Scan back from y=40 to y=-20 (downward, back through grid)
            tasks.append({
                "type": "scan",
                "start": [x, end_y],
                "end": [x, start_y],
                "camera_yaw": self.camera_yaw,
                "camera_pitch": self.camera_pitch,
                "description": f"Vertical scan {i+1} return: y={end_y} to {start_y}, x={x}"
            })
            print(f"      Task {len(tasks)}: scan from ({x}, {end_y}) to ({x}, {start_y})")
        
        print(f"\n✅ Generated {len(tasks)} tasks total")
        print(f"   - Horizontal scans: {len(y_positions) * 2} tasks")
        print(f"   - Vertical scans: {len(x_positions) * 2} tasks")
        return tasks
    
    def assign_tasks_to_drone(self, drone_id, tasks):
        """Write tasks to JSON file for drone to read."""
        task_file = os.path.join(self.task_dir, f"drone_{drone_id}_tasks.json")
        
        data = {
            "drone_id": drone_id,
            "mission_type": "lawnmower",
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
            
            # Create drone definition string
            drone_def = f"""DEF DRONE_0 Mavic2Pro {{
  translation {x} {y} {z}
  rotation 0 0 1 0
  name "drone_0"
  controller "patrol_dynamic"
  controllerArgs []
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
        print("\n🚀 Starting lawnmower mapping mission...")
        
        # Generate and assign tasks BEFORE spawning drone
        print("\n📋 Generating lawnmower tasks...")
        tasks = self.generate_lawnmower_tasks(drone_id=0)
        
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
                        print(f"✅ Drone completed lawnmower mapping mission")
                        print(f"⏱️  Total mission time: {current_time:.1f} seconds")
                        print("=" * 70)
                        break
            except:
                pass

if __name__ == "__main__":
    supervisor = LawnmowerMissionSupervisor()
    supervisor.run()
