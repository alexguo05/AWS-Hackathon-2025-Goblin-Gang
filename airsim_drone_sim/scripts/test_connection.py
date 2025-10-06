#!/usr/bin/env python3
"""
Test AirSim connection without running a full simulation
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent.parent / 'src'))

import airsim
import time

def test_connection():
    """Test connection to AirSim simulator."""
    print("🔌 Testing AirSim connection...")
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("✅ Successfully connected to AirSim!")
        
        # Get drone state
        state = client.getMultirotorState()
        print(f"📡 Drone state retrieved: {state.landed_state}")
        
        # Test API control
        client.enableApiControl(True)
        print("🎮 API control enabled")
        
        # Disable API control
        client.enableApiControl(False)
        print("🔒 API control disabled")
        
        print("\n🎉 AirSim connection test PASSED!")
        print("You're ready to run simulations!")
        
        return True
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure AirSim is running")
        print("2. Load a drone-enabled environment")
        print("3. Check that AirSim is accessible on localhost:41451")
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
