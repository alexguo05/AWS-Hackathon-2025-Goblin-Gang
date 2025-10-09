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
        self.num_drones = 3  # Three drones for efficient coverage
        self.spawn_positions = [
            [-10, 0, 0.3],   # Drone 0: Left spawn
            [0, 0, 0.3],     # Drone 1: Center spawn  
            [10, 0, 0.3]     # Drone 2: Right spawn
        ]
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
        
        # Centralized task management
        self.all_tasks = []  # Pool of all available tasks
        self.assigned_tasks = {}  # drone_id -> task_id mapping
        self.completed_tasks = set()  # Set of completed task IDs
        self.task_lock_file = os.path.join(self.task_dir, "task_lock.json")
        self.task_pool_file = os.path.join(self.task_dir, "task_pool.json")
        
        print("=" * 70)
        print("🎯 CIRCLE PATROL MISSION SUPERVISOR")
        print("=" * 70)
        print(f"📊 Configuration:")
        print(f"   Number of drones: {self.num_drones}")
        print(f"   Spawn positions: {self.spawn_positions}")
        print(f"   Main area center: {self.target_center}")
        print(f"   Sub-circle radius: {self.circle_radius}m")
        print(f"   Number of sub-circles: {self.num_circles}")
        print(f"   Waypoints per circle: {self.num_waypoints}")
        print(f"   Overlap percentage: {self.overlap_percentage*100}%")
        print(f"   Camera orientation: yaw={self.camera_yaw}°, pitch={self.camera_pitch}°")
        print(f"   Task directory: {self.task_dir}")
        print("=" * 70)
    
    def generate_all_tasks(self):
        """Generate circle-based tasks for the centralized pool."""
        import math
        
        print(f"\n📋 Generating centralized circle-based task pool:")
        print(f"   Main area center: {self.target_center}")
        print(f"   Sub-circle radius: {self.circle_radius}m")
        print(f"   Total sub-circles: {self.num_circles}")
        print(f"   Waypoints per circle: {self.num_waypoints}")
        print(f"   Overlap: {self.overlap_percentage*100}%")
        
        # Calculate circle centers with 50% overlap
        circle_spacing = self.circle_radius * (1 - self.overlap_percentage)
        
        # Generate all circle centers in a grid pattern
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
        
        # Generate one task per circle (each drone gets assigned an entire circle)
        task_id = 0
        for circle_idx, circle_center in enumerate(circle_centers):
            print(f"\n   🎯 Generating Circle {circle_idx + 1} task at ({circle_center[0]:.1f}, {circle_center[1]:.1f})")
            
            # Generate all waypoints for this circle
            waypoints = []
            for waypoint_idx in range(self.num_waypoints):
                angle = 2 * math.pi * waypoint_idx / self.num_waypoints
                x = circle_center[0] + self.circle_radius * math.cos(angle)
                y = circle_center[1] + self.circle_radius * math.sin(angle)
                waypoints.append([x, y])
            
            # Create single circle task
            self.all_tasks.append({
                "task_id": task_id,
                "type": "circle_patrol",
                "circle_center": circle_center,
                "waypoints": waypoints,
                "camera_yaw": self.camera_yaw,
                "camera_pitch": self.camera_pitch,
                "description": f"Complete Circle {circle_idx + 1} patrol ({self.num_waypoints} waypoints)",
                "circle_id": circle_idx + 1,
                "num_waypoints": self.num_waypoints,
                "status": "available",  # available, assigned, completed
                "assigned_to": None,
                "completed_by": None
            })
            print(f"      Circle task {task_id}: {self.num_waypoints} waypoints around ({circle_center[0]:.1f}, {circle_center[1]:.1f})")
            task_id += 1
        
        print(f"\n✅ Generated {len(self.all_tasks)} circle-based tasks in centralized pool")
        print(f"   - Each task represents a complete circle patrol")
        print(f"   - Maximum {self.num_drones} tasks can be assigned simultaneously")
        
        # Save task pool to file
        self.save_task_pool()
        
        return self.all_tasks
    
    def save_task_pool(self):
        """Save the task pool to file for drones to read."""
        try:
            with open(self.task_pool_file, 'w') as f:
                json.dump({
                    "total_tasks": len(self.all_tasks),
                    "available_tasks": len([t for t in self.all_tasks if t["status"] == "available"]),
                    "assigned_tasks": len([t for t in self.all_tasks if t["status"] == "assigned"]),
                    "completed_tasks": len([t for t in self.all_tasks if t["status"] == "completed"]),
                    "tasks": self.all_tasks
                }, f, indent=2)
            print(f"✅ Task pool saved to {self.task_pool_file}")
        except Exception as e:
            print(f"❌ Error saving task pool: {e}")
    
    def assign_task_to_drone(self, drone_id):
        """Assign an available task to a drone (atomic operation)."""
        # Try to acquire lock
        lock_acquired = False
        max_attempts = 5
        attempt = 0
        
        while not lock_acquired and attempt < max_attempts:
            try:
                # Check if lock file exists
                if os.path.exists(self.task_lock_file):
                    # Read lock file to check if it's stale
                    with open(self.task_lock_file, 'r') as f:
                        lock_data = json.load(f)
                        lock_time = lock_data.get('timestamp', 0)
                        current_time = self.getTime()
                        
                        # If lock is older than 2 seconds, consider it stale
                        if current_time - lock_time > 2.0:
                            os.remove(self.task_lock_file)
                
                # Try to create lock file
                if not os.path.exists(self.task_lock_file):
                    with open(self.task_lock_file, 'w') as f:
                        json.dump({
                            'locked_by': drone_id,
                            'timestamp': self.getTime()
                        }, f)
                    lock_acquired = True
                else:
                    time.sleep(0.05)  # Wait 50ms before retry
                    attempt += 1
                    
            except Exception as e:
                time.sleep(0.05)
                attempt += 1
        
        if not lock_acquired:
            print(f"❌ Drone {drone_id}: Could not acquire task lock after {max_attempts} attempts")
            return None
        
        try:
            # Find an available task
            available_task = None
            for task in self.all_tasks:
                if task["status"] == "available":
                    available_task = task
                    break
            
            if available_task is None:
                print(f"📋 Drone {drone_id}: No available tasks")
                return None
            
            # Double-check task is still available (prevent race condition)
            if available_task["status"] != "available":
                print(f"⚠️  Drone {drone_id}: Task {available_task['task_id']} was assigned to another drone")
                return None
            
            # Assign the task
            available_task["status"] = "assigned"
            available_task["assigned_to"] = drone_id
            
            print(f"🔒 Drone {drone_id}: Successfully assigned task {available_task['task_id']} - {available_task['description']}")
            
            # Save updated task pool
            self.save_task_pool()
            
            return available_task
            
        finally:
            # Always release the lock
            try:
                if os.path.exists(self.task_lock_file):
                    os.remove(self.task_lock_file)
            except:
                pass
    
    def complete_task(self, drone_id, task_id):
        """Mark a task as completed by a drone."""
        import time
        
        # Try to acquire lock
        lock_acquired = False
        max_attempts = 10
        attempt = 0
        
        while not lock_acquired and attempt < max_attempts:
            try:
                if not os.path.exists(self.task_lock_file):
                    with open(self.task_lock_file, 'w') as f:
                        json.dump({
                            'locked_by': drone_id,
                            'timestamp': time.time()
                        }, f)
                    lock_acquired = True
                else:
                    time.sleep(0.1)
                    attempt += 1
            except:
                time.sleep(0.1)
                attempt += 1
        
        if not lock_acquired:
            print(f"❌ Drone {drone_id}: Could not acquire lock to complete task {task_id}")
            return False
        
        try:
            # Find and complete the task
            for task in self.all_tasks:
                if task["task_id"] == task_id and task["assigned_to"] == drone_id:
                    task["status"] = "completed"
                    task["completed_by"] = drone_id
                    self.completed_tasks.add(task_id)
                    
                    print(f"✅ Drone {drone_id}: Completed task {task_id} - {task['description']}")
                    
                    # Save updated task pool
                    self.save_task_pool()
                    return True
            
            print(f"❌ Drone {drone_id}: Task {task_id} not found or not assigned to this drone")
            return False
            
        finally:
            # Release the lock
            try:
                if os.path.exists(self.task_lock_file):
                    os.remove(self.task_lock_file)
            except:
                pass
    
    def get_task_status(self):
        """Get current task status for monitoring."""
        available = len([t for t in self.all_tasks if t["status"] == "available"])
        assigned = len([t for t in self.all_tasks if t["status"] == "assigned"])
        completed = len([t for t in self.all_tasks if t["status"] == "completed"])
        total = len(self.all_tasks)
        
        return {
            "total": total,
            "available": available,
            "assigned": assigned,
            "completed": completed,
            "progress": f"{completed}/{total} ({completed/total*100:.1f}%)"
        }
    
    def process_task_requests(self):
        """Process task requests from drones."""
        for drone_id in range(self.num_drones):
            request_file = os.path.join(self.task_dir, f"drone_{drone_id}_request.json")
            response_file = os.path.join(self.task_dir, f"drone_{drone_id}_response.json")
            
            # Skip if response file already exists (request already processed)
            if os.path.exists(response_file):
                continue
                
            if os.path.exists(request_file):
                try:
                    # Read the request
                    with open(request_file, 'r') as f:
                        request_data = json.load(f)
                    
                    # Check if it's a valid request
                    if request_data.get('status') == 'requesting':
                        print(f"📋 Processing task request from Drone {drone_id}")
                        
                        # Assign a task to this drone
                        task = self.assign_task_to_drone(drone_id)
                        
                        if task:
                            print(f"🔒 Assigned task {task['task_id']} to Drone {drone_id}: {task['description']}")
                            
                            # Write response file for drone to read
                            with open(response_file, 'w') as f:
                                json.dump({
                                    "drone_id": drone_id,
                                    "task": task,
                                    "status": "assigned",
                                    "timestamp": self.getTime()
                                }, f, indent=2)
                        else:
                            print(f"⚠️  No tasks available for Drone {drone_id}")
                            
                            # No tasks available
                            with open(response_file, 'w') as f:
                                json.dump({
                                    "drone_id": drone_id,
                                    "task": None,
                                    "status": "no_tasks",
                                    "timestamp": self.getTime()
                                }, f, indent=2)
                    
                    # Remove the request file immediately after processing
                    os.remove(request_file)
                    
                except Exception as e:
                    print(f"❌ Error processing request from drone {drone_id}: {e}")
                    # Remove request file even on error to prevent infinite processing
                    try:
                        os.remove(request_file)
                    except:
                        pass
    
    def process_task_completions(self):
        """Process task completion notifications from drones."""
        for drone_id in range(self.num_drones):
            complete_file = os.path.join(self.task_dir, f"drone_{drone_id}_complete.json")
            
            if os.path.exists(complete_file):
                try:
                    # Read the completion
                    with open(complete_file, 'r') as f:
                        complete_data = json.load(f)
                    
                    # Mark task as completed
                    task_id = complete_data.get('task_id')
                    if task_id is not None:
                        print(f"✅ Processing task completion from Drone {drone_id}: Task {task_id}")
                        self.complete_task(drone_id, task_id)
                    
                    # Remove the completion file
                    os.remove(complete_file)
                    
                except Exception as e:
                    print(f"❌ Error processing completion from drone {drone_id}: {e}")
    
    def spawn_drones(self):
        """Spawn multiple drones at different spawn positions."""
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
            
            # Spawn each drone at its assigned position
            for drone_id in range(self.num_drones):
                x, y, z = self.spawn_positions[drone_id]
                
                # Create drone definition string with high-quality camera settings
                drone_def = f"""DEF DRONE_{drone_id} Mavic2Pro {{
  translation {x} {y} {z}
  rotation 0 0 1 0
  name "drone_{drone_id}"
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
                
                print(f"   📝 Creating drone_{drone_id} at spawn position...")
                
                # Import drone into the world
                try:
                    children_field.importMFNodeFromString(-1, drone_def)
                    print(f"   ✅ Spawned drone_{drone_id} at ({x:.1f}, {y:.1f}, {z:.1f})")
                except Exception as e:
                    print(f"   ❌ ERROR spawning drone_{drone_id}: {e}")
                    return False
            
            print(f"\n✅ All {self.num_drones} drones spawned successfully!")
            return True
            
        except Exception as e:
            print(f"   ❌ CRITICAL ERROR in spawn_drones: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def monitor_mission(self):
        """Monitor the mission progress for all drones."""
        # Check if all drones exist
        active_drones = 0
        for drone_id in range(self.num_drones):
            try:
                drone_node = self.getFromDef(f"DRONE_{drone_id}")
                if drone_node:
                    active_drones += 1
            except Exception as e:
                pass
        
        # Mission is running if at least one drone is active
        return active_drones > 0
    
    def run(self):
        """Main supervisor control loop."""
        print("\n🚀 Starting centralized multi-drone circle patrol mission...")
        
        # Generate centralized task pool
        print("\n📋 Generating centralized task pool...")
        self.generate_all_tasks()
        
        # Spawn all drones (they will request tasks dynamically)
        print(f"\n🚁 Spawning {self.num_drones} drones (will request tasks dynamically)...")
        spawn_success = self.spawn_drones()
        
        if not spawn_success:
            print(f"\n❌ Failed to spawn drones. Exiting...")
            print("\n🔧 Troubleshooting:")
            print("   1. Check if Mavic2Pro.wbo file exists in the project")
            print("   2. Verify supervisor has 'supervisor TRUE' field")
            print("   3. Check Webots console for detailed error messages")
            return
        
        print(f"\n🎯 Mission started! {self.num_drones} drones deployed.")
        print("📊 Drones will request tasks from centralized pool dynamically.")
        print("⏱️  Monitor progress in individual drone console outputs.")
        
        # Mission monitoring loop
        mission_start_time = self.getTime()
        print(f"\n⏰ Mission monitoring started at t={mission_start_time:.1f}s")
        
        last_status = None  # Track last status to only print when it changes
        
        while self.step(self.time_step) != -1:
            current_time = self.getTime()
            
            # Process task requests from drones
            self.process_task_requests()
            
            # Process task completions from drones
            self.process_task_completions()
            
            # Check mission status every 5 seconds, but only print if it changed
            if int(current_time) % 5 == 0 and current_time > mission_start_time + 1:
                status = self.get_task_status()
                
                # Only print if status changed
                if last_status != status['progress']:
                    print(f"\n📊 Mission Status: {status['progress']}")
                    print(f"   Available: {status['available']}, Assigned: {status['assigned']}, Completed: {status['completed']}")
                    last_status = status['progress']
                
                # Check if all tasks are completed
                if status['available'] == 0 and status['assigned'] == 0:
                    print("\n" + "=" * 70)
                    print("🎉 MISSION COMPLETE!")
                    print("=" * 70)
                    print(f"✅ All {self.num_drones} drones completed centralized circle patrol mission")
                    print(f"⏱️  Total mission time: {current_time:.1f} seconds")
                    print(f"📊 Final status: {status['progress']}")
                    print("=" * 70)
                    break

if __name__ == "__main__":
    supervisor = CirclePatrolMissionSupervisor()
    supervisor.run()
