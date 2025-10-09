"""Monitoring and management interface for the multi-drone coordinator system."""

import time
import json
import sys
import os
from datetime import datetime

# Import our coordination system
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from multi_drone_coordinator.multi_drone_coordinator import get_coordinator

class CoordinatorMonitor:
    """Provides monitoring and management capabilities for the drone coordination system."""
    
    def __init__(self):
        self.coordinator = get_coordinator()
        
    def print_status_dashboard(self):
        """Print a real-time status dashboard."""
        status = self.coordinator.get_status_summary()
        
        print("\n" + "="*80)
        print("🎯 MULTI-DRONE 3D MODELING COORDINATION DASHBOARD")
        print("="*80)
        print(f"📊 Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"")
        print(f"🏗️  Job Statistics:")
        print(f"   📋 Total Jobs: {status['total_jobs']}")
        print(f"   ⏳ Pending: {status['job_status_counts']['pending']}")
        print(f"   📝 Assigned: {status['job_status_counts']['assigned']}")
        print(f"   ▶️  In Progress: {status['job_status_counts']['in_progress']}")
        print(f"   ✅ Completed: {status['job_status_counts']['completed']}")
        print(f"   ❌ Failed: {status['job_status_counts']['failed']}")
        print(f"   💀 Dead Letter: {status['job_status_counts']['dead_letter']}")
        print(f"")
        print(f"🤖 Drone Fleet:")
        print(f"   🚁 Total Drones: {status['total_drones']}")
        print(f"   💚 Healthy Drones: {status['healthy_drones']}")
        print(f"   🔄 Available Drones: {status['available_drones']}")
        print(f"")
        print(f"📸 Photography Progress:")
        print(f"   📷 Photos Captured: {status['photos_captured']}")
        print(f"   📦 Dead Letter Queue Size: {status['dead_letter_queue_size']}")
        
        # Calculate completion percentage
        completed = status['job_status_counts']['completed']
        total = status['total_jobs']
        completion_pct = (completed / total * 100) if total > 0 else 0
        print(f"   📈 Overall Progress: {completion_pct:.1f}% ({completed}/{total})")
        
        # Progress bar
        bar_length = 50
        filled_length = int(bar_length * completion_pct / 100)
        bar = "█" * filled_length + "░" * (bar_length - filled_length)
        print(f"   [{bar}] {completion_pct:.1f}%")
        print("="*80)

    def export_all_data(self):
        """Export all coordination data for analysis."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Export photo metadata
        photo_filename = f"photo_metadata_{timestamp}.json"
        self.coordinator.export_photo_metadata(photo_filename)
        
        # Export status summary
        status_filename = f"status_summary_{timestamp}.json"
        status = self.coordinator.get_status_summary()
        
        with open(status_filename, 'w') as f:
            json.dump(status, f, indent=2)
        
        print(f"📄 Exported coordination data:")
        print(f"   📸 Photo metadata: {photo_filename}")
        print(f"   📊 Status summary: {status_filename}")

    def run_monitoring_loop(self, update_interval: float = 10.0):
        """Run continuous monitoring loop."""
        print("🖥️  Starting coordination monitoring...")
        print("   Press Ctrl+C to stop and export data")
        
        try:
            while True:
                self.print_status_dashboard()
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
            self.export_all_data()
            self.coordinator.shutdown()

if __name__ == "__main__":
    monitor = CoordinatorMonitor()
    monitor.run_monitoring_loop()