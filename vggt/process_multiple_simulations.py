#!/usr/bin/env python3
"""
Process Multiple Drone Simulations in Batch

This script processes multiple simulation folders at once.
Great for processing a whole day's worth of simulation runs!
"""

from batch_process_simulations import process_simulation

# ============================================================================
# Configure Your Simulation Runs Here
# ============================================================================

SIMULATIONS = [
    {
        "input_folder": "path/to/simulation_run_1",
        "name": "Morning_Flight_01",
        "upload": True
    },
    {
        "input_folder": "path/to/simulation_run_2", 
        "name": "Morning_Flight_02",
        "upload": True
    },
    # Add more simulations here...
]

# Or automatically find all folders in a directory:
AUTO_PROCESS_DIRECTORY = None  # e.g., "C:/DroneSimulations/Runs"
# If set, will process all subfolders in this directory


def process_all_simulations():
    """Process all configured simulations."""
    import os
    
    simulations_to_process = SIMULATIONS.copy()
    
    # Auto-discover folders if configured
    if AUTO_PROCESS_DIRECTORY and os.path.exists(AUTO_PROCESS_DIRECTORY):
        print(f"Auto-discovering simulations in: {AUTO_PROCESS_DIRECTORY}")
        for folder_name in os.listdir(AUTO_PROCESS_DIRECTORY):
            folder_path = os.path.join(AUTO_PROCESS_DIRECTORY, folder_name)
            if os.path.isdir(folder_path):
                # Check if it has images
                image_extensions = ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']
                has_images = any(
                    any(f.endswith(ext) for ext in image_extensions)
                    for f in os.listdir(folder_path)
                )
                if has_images:
                    simulations_to_process.append({
                        "input_folder": folder_path,
                        "name": folder_name,
                        "upload": True
                    })
    
    if not simulations_to_process:
        print("❌ No simulations configured. Edit SIMULATIONS list or set AUTO_PROCESS_DIRECTORY")
        return
    
    print(f"\n🚀 Processing {len(simulations_to_process)} simulations...")
    print("="*70)
    
    results = []
    for i, sim in enumerate(simulations_to_process, 1):
        print(f"\n[{i}/{len(simulations_to_process)}] Processing: {sim.get('name', sim['input_folder'])}")
        
        success = process_simulation(
            input_folder=sim["input_folder"],
            simulation_name=sim.get("name"),
            upload_to_s3_flag=sim.get("upload", True)
        )
        
        results.append({
            "name": sim.get("name", sim["input_folder"]),
            "success": success
        })
    
    # Print summary
    print("\n" + "="*70)
    print("BATCH PROCESSING SUMMARY")
    print("="*70)
    
    successful = sum(1 for r in results if r["success"])
    failed = len(results) - successful
    
    for result in results:
        status = "✅" if result["success"] else "❌"
        print(f"{status} {result['name']}")
    
    print(f"\nTotal: {len(results)} | Successful: {successful} | Failed: {failed}")
    print("="*70)


if __name__ == "__main__":
    process_all_simulations()

