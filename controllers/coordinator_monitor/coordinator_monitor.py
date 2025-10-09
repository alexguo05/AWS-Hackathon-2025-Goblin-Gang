"""Multi-drone coordinator for distributed 3D modeling photography missions.
   Implements job distribution, failure recovery, and dead letter queue management."""

import json
import time
import math
import threading
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
import queue

class JobStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    DEAD_LETTER = "dead_letter"

@dataclass
class CirclePhotographyJob:
    """Represents a circular photography job at a specific location."""
    job_id: str
    center_x: float
    center_y: float
    radius: float
    altitude: float
    num_positions: int  # Number of positions around the circle
    angles_per_position: List[float]  # Camera angles to capture at each position
    assigned_drone_id: Optional[str] = None
    status: JobStatus = JobStatus.PENDING
    created_time: float = 0
    assigned_time: float = 0
    started_time: float = 0
    completed_time: float = 0
    retry_count: int = 0
    max_retries: int = 3

@dataclass
class DroneStatus:
    """Represents the current status of a drone."""
    drone_id: str
    x: float
    y: float
    z: float
    is_healthy: bool
    last_heartbeat: float
    current_job_id: Optional[str] = None
    is_available: bool = True

@dataclass
class PhotoMetadata:
    """Metadata for captured photos for 3D modeling."""
    job_id: str
    drone_id: str
    position_index: int
    camera_angle: float
    camera_x: float
    camera_y: float
    camera_z: float
    target_x: float
    target_y: float
    target_z: float
    timestamp: float
    filename: str

class JobCoordinator:
    """Coordinates job distribution and failure handling across multiple drones."""
    
    def __init__(self, world_size: int = 400, circle_radius: float = 15.0, 
                 circle_spacing: float = 40.0, heartbeat_timeout: float = 30.0):
        self.world_size = world_size
        self.circle_radius = circle_radius
        self.circle_spacing = circle_spacing
        self.heartbeat_timeout = heartbeat_timeout
        
        # Job management
        self.jobs: Dict[str, CirclePhotographyJob] = {}
        self.job_queue = queue.PriorityQueue()
        self.dead_letter_queue = queue.Queue()
        
        # Drone management
        self.drones: Dict[str, DroneStatus] = {}
        
        # Photo metadata storage
        self.photo_metadata: List[PhotoMetadata] = []
        
        # Threading
        self.coordinator_lock = threading.Lock()
        self.running = True
        
        # Generate initial job grid
        self._generate_circle_jobs()
        
        # Start background threads
        self.health_monitor_thread = threading.Thread(target=self._monitor_drone_health)
        self.dead_letter_processor_thread = threading.Thread(target=self._process_dead_letter_queue)
        self.health_monitor_thread.start()
        self.dead_letter_processor_thread.start()
        
        print(f"🎯 JobCoordinator initialized with {len(self.jobs)} circle photography jobs")

    def _generate_circle_jobs(self):
        """Generate circular photography jobs in a grid pattern."""
        job_counter = 0
        
        # Calculate grid boundaries
        start_x = -self.world_size // 2 + self.circle_spacing // 2
        end_x = self.world_size // 2 - self.circle_spacing // 2
        start_y = -self.world_size // 2 + self.circle_spacing // 2
        end_y = self.world_size // 2 - self.circle_spacing // 2
        
        y = start_y
        while y <= end_y:
            x = start_x
            while x <= end_x:
                job_id = f"circle_job_{job_counter:03d}"
                
                # Define camera angles for 3D reconstruction
                # Multiple angles provide better depth information
                camera_angles = [-45, -30, -15, 0, 15, 30, 45]  # degrees from horizontal
                
                job = CirclePhotographyJob(
                    job_id=job_id,
                    center_x=float(x),
                    center_y=float(y),
                    radius=self.circle_radius,
                    altitude=25.0,  # Fixed altitude for now
                    num_positions=8,  # 8 positions around the circle (45° apart)
                    angles_per_position=camera_angles,
                    created_time=time.time()
                )
                
                self.jobs[job_id] = job
                # Priority based on distance from origin (process center jobs first)
                distance_from_center = math.sqrt(x*x + y*y)
                self.job_queue.put((distance_from_center, job_id))
                
                job_counter += 1
                x += self.circle_spacing
            y += self.circle_spacing
        
        print(f"📋 Generated {job_counter} circular photography jobs")

    def register_drone(self, drone_id: str, x: float = 0, y: float = 0, z: float = 0):
        """Register a new drone with the coordinator."""
        with self.coordinator_lock:
            self.drones[drone_id] = DroneStatus(
                drone_id=drone_id,
                x=x, y=y, z=z,
                is_healthy=True,
                last_heartbeat=time.time(),
                is_available=True
            )
        print(f"🤖 Registered drone: {drone_id}")

    def update_drone_heartbeat(self, drone_id: str, x: float, y: float, z: float):
        """Update drone position and heartbeat."""
        with self.coordinator_lock:
            if drone_id in self.drones:
                drone = self.drones[drone_id]
                drone.x, drone.y, drone.z = x, y, z
                drone.last_heartbeat = time.time()
                drone.is_healthy = True

    def request_job(self, drone_id: str) -> Optional[CirclePhotographyJob]:
        """Request the next available job for a drone."""
        with self.coordinator_lock:
            if drone_id not in self.drones or not self.drones[drone_id].is_available:
                return None
            
            try:
                # Get the highest priority job
                _, job_id = self.job_queue.get_nowait()
                job = self.jobs[job_id]
                
                if job.status == JobStatus.PENDING:
                    # Assign job to drone
                    job.assigned_drone_id = drone_id
                    job.status = JobStatus.ASSIGNED
                    job.assigned_time = time.time()
                    
                    drone = self.drones[drone_id]
                    drone.current_job_id = job_id
                    drone.is_available = False
                    
                    print(f"📝 Assigned job {job_id} to drone {drone_id}")
                    return job
                    
            except queue.Empty:
                pass
            
            return None

    def start_job(self, drone_id: str, job_id: str):
        """Mark a job as started."""
        with self.coordinator_lock:
            if job_id in self.jobs and self.jobs[job_id].assigned_drone_id == drone_id:
                self.jobs[job_id].status = JobStatus.IN_PROGRESS
                self.jobs[job_id].started_time = time.time()
                print(f"▶️ Job {job_id} started by drone {drone_id}")

    def complete_job(self, drone_id: str, job_id: str, photo_metadata_list: List[PhotoMetadata]):
        """Mark a job as completed and store photo metadata."""
        with self.coordinator_lock:
            if job_id in self.jobs and self.jobs[job_id].assigned_drone_id == drone_id:
                job = self.jobs[job_id]
                job.status = JobStatus.COMPLETED
                job.completed_time = time.time()
                
                # Store photo metadata
                self.photo_metadata.extend(photo_metadata_list)
                
                # Free up the drone
                if drone_id in self.drones:
                    drone = self.drones[drone_id]
                    drone.current_job_id = None
                    drone.is_available = True
                
                print(f"✅ Job {job_id} completed by drone {drone_id} with {len(photo_metadata_list)} photos")

    def fail_job(self, drone_id: str, job_id: str, error_message: str = ""):
        """Mark a job as failed and potentially retry or send to dead letter queue."""
        with self.coordinator_lock:
            if job_id in self.jobs and self.jobs[job_id].assigned_drone_id == drone_id:
                job = self.jobs[job_id]
                job.retry_count += 1
                
                # Free up the drone
                if drone_id in self.drones:
                    drone = self.drones[drone_id]
                    drone.current_job_id = None
                    drone.is_available = True
                
                if job.retry_count <= job.max_retries:
                    # Retry the job
                    job.status = JobStatus.PENDING
                    job.assigned_drone_id = None
                    # Re-add to queue with higher priority (lower number)
                    priority = math.sqrt(job.center_x**2 + job.center_y**2) - job.retry_count * 100
                    self.job_queue.put((priority, job_id))
                    print(f"🔄 Job {job_id} failed, retry {job.retry_count}/{job.max_retries}")
                else:
                    # Send to dead letter queue
                    job.status = JobStatus.DEAD_LETTER
                    self.dead_letter_queue.put((job_id, error_message))
                    print(f"💀 Job {job_id} sent to dead letter queue after {job.retry_count} retries")

    def _monitor_drone_health(self):
        """Background thread to monitor drone health and reassign stuck jobs."""
        while self.running:
            current_time = time.time()
            
            with self.coordinator_lock:
                for drone_id, drone in list(self.drones.items()):
                    # Check if drone has timed out
                    if current_time - drone.last_heartbeat > self.heartbeat_timeout:
                        if drone.is_healthy:
                            print(f"⚠️ Drone {drone_id} health timeout detected")
                            drone.is_healthy = False
                            
                            # If drone has a job, fail it
                            if drone.current_job_id:
                                self.fail_job(drone_id, drone.current_job_id, "Drone health timeout")
            
            time.sleep(5)  # Check every 5 seconds

    def _process_dead_letter_queue(self):
        """Background thread to process dead letter queue and attempt recovery."""
        while self.running:
            try:
                job_id, error_message = self.dead_letter_queue.get(timeout=10)
                
                with self.coordinator_lock:
                    if job_id in self.jobs:
                        job = self.jobs[job_id]
                        
                        # Check if we have healthy drones available
                        healthy_drones = [d for d in self.drones.values() 
                                        if d.is_healthy and d.is_available]
                        
                        if healthy_drones:
                            # Reset job for retry with a healthy drone
                            job.status = JobStatus.PENDING
                            job.assigned_drone_id = None
                            job.retry_count = 0  # Reset retry count for dead letter recovery
                            
                            # Add back to job queue
                            priority = math.sqrt(job.center_x**2 + job.center_y**2)
                            self.job_queue.put((priority, job_id))
                            
                            print(f"🔄 Recovered job {job_id} from dead letter queue")
                        else:
                            # No healthy drones, put back in dead letter queue
                            self.dead_letter_queue.put((job_id, error_message))
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Error processing dead letter queue: {e}")

    def get_status_summary(self) -> Dict:
        """Get a summary of the coordinator status."""
        with self.coordinator_lock:
            job_counts = {status.value: 0 for status in JobStatus}
            for job in self.jobs.values():
                job_counts[job.status.value] += 1
            
            healthy_drones = sum(1 for d in self.drones.values() if d.is_healthy)
            available_drones = sum(1 for d in self.drones.values() if d.is_available and d.is_healthy)
            
            return {
                "total_jobs": len(self.jobs),
                "job_status_counts": job_counts,
                "total_drones": len(self.drones),
                "healthy_drones": healthy_drones,
                "available_drones": available_drones,
                "photos_captured": len(self.photo_metadata),
                "dead_letter_queue_size": self.dead_letter_queue.qsize()
            }

    def export_photo_metadata(self, filename: str = "photo_metadata.json"):
        """Export all photo metadata for 3D modeling pipeline."""
        with self.coordinator_lock:
            metadata_dicts = [asdict(meta) for meta in self.photo_metadata]
            
            export_data = {
                "export_timestamp": datetime.now().isoformat(),
                "total_photos": len(metadata_dicts),
                "world_size": self.world_size,
                "circle_radius": self.circle_radius,
                "photos": metadata_dicts
            }
            
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            print(f"📄 Exported {len(metadata_dicts)} photo metadata entries to {filename}")

    def shutdown(self):
        """Gracefully shutdown the coordinator."""
        self.running = False
        if hasattr(self, 'health_monitor_thread'):
            self.health_monitor_thread.join()
        if hasattr(self, 'dead_letter_processor_thread'):
            self.dead_letter_processor_thread.join()
        print("🛑 JobCoordinator shutdown complete")

# Global coordinator instance
job_coordinator = None

def get_coordinator() -> JobCoordinator:
    """Get the global job coordinator instance."""
    global job_coordinator
    if job_coordinator is None:
        job_coordinator = JobCoordinator()
    return job_coordinator