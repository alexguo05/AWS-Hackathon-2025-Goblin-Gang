#!/usr/bin/env python3
"""
Setup verification script for AirSim Drone Simulation
Checks if all dependencies are properly installed and configured.
"""

import sys
import importlib
from pathlib import Path

def check_import(module_name, package_name=None):
    """Check if a module can be imported."""
    try:
        importlib.import_module(module_name)
        print(f"✅ {package_name or module_name} - OK")
        return True
    except ImportError as e:
        print(f"❌ {package_name or module_name} - FAILED: {e}")
        return False

def check_file_exists(file_path, description):
    """Check if a file exists."""
    if Path(file_path).exists():
        print(f"✅ {description} - OK")
        return True
    else:
        print(f"❌ {description} - MISSING")
        return False

def main():
    """Main verification function."""
    print("🔍 Verifying AirSim Drone Simulation Setup...")
    print("=" * 50)
    
    # Check Python version
    python_version = sys.version_info
    if python_version >= (3, 8):
        print(f"✅ Python {python_version.major}.{python_version.minor}.{python_version.micro} - OK")
    else:
        print(f"❌ Python {python_version.major}.{python_version.minor}.{python_version.micro} - FAILED (requires 3.8+)")
        return False
    
    print("\n📦 Checking Dependencies...")
    print("-" * 30)
    
    # Core dependencies
    dependencies = [
        ("airsim", "AirSim"),
        ("numpy", "NumPy"),
        ("pandas", "Pandas"),
        ("cv2", "OpenCV"),
        ("PIL", "Pillow"),
        ("matplotlib", "Matplotlib"),
        ("seaborn", "Seaborn"),
        ("tqdm", "TQDM"),
        ("yaml", "PyYAML"),
        ("dotenv", "python-dotenv"),
        ("h5py", "H5Py"),
        ("pytz", "Pytz"),
    ]
    
    all_deps_ok = True
    for module, name in dependencies:
        if not check_import(module, name):
            all_deps_ok = False
    
    print("\n📁 Checking Project Structure...")
    print("-" * 30)
    
    # Check project structure
    project_files = [
        ("src/drone_simulator.py", "Drone Simulator"),
        ("src/data_analyzer.py", "Data Analyzer"),
        ("config/settings.json", "Configuration"),
        ("scripts/run_simulation.py", "Simulation Script"),
        ("notebooks/analysis_demo.ipynb", "Analysis Notebook"),
        ("requirements.txt", "Requirements"),
        ("README.md", "Documentation"),
    ]
    
    all_files_ok = True
    for file_path, description in project_files:
        if not check_file_exists(file_path, description):
            all_files_ok = False
    
    print("\n🔧 Checking Data Directories...")
    print("-" * 30)
    
    # Check data directories
    data_dirs = [
        ("data/images", "Images Directory"),
        ("data/telemetry", "Telemetry Directory"),
        ("data/logs", "Logs Directory"),
    ]
    
    all_dirs_ok = True
    for dir_path, description in data_dirs:
        if not check_file_exists(dir_path, description):
            all_dirs_ok = False
    
    print("\n" + "=" * 50)
    
    if all_deps_ok and all_files_ok and all_dirs_ok:
        print("🎉 Setup verification PASSED!")
        print("\nNext steps:")
        print("1. Start AirSim simulator with a drone environment")
        print("2. Run: python scripts/run_simulation.py --pattern waypoint")
        print("3. Analyze results with: jupyter notebook notebooks/analysis_demo.ipynb")
        return True
    else:
        print("❌ Setup verification FAILED!")
        print("\nPlease fix the issues above before running simulations.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
