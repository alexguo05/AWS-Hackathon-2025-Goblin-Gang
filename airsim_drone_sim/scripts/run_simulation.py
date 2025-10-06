#!/usr/bin/env python3
"""
Main script to run AirSim drone simulation
Usage: python run_simulation.py [--pattern PATTERN] [--config CONFIG]
"""

import argparse
import sys
import os
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent.parent / 'src'))

from drone_simulator import DroneSimulator
from data_analyzer import DataAnalyzer


def main():
    """Main function to run drone simulation."""
    parser = argparse.ArgumentParser(description='Run AirSim drone simulation')
    parser.add_argument('--pattern', 
                       choices=['hover', 'circle', 'waypoint'], 
                       default='waypoint',
                       help='Flight pattern to execute (default: waypoint)')
    parser.add_argument('--config', 
                       default='config/settings.json',
                       help='Path to configuration file (default: config/settings.json)')
    parser.add_argument('--analyze', 
                       action='store_true',
                       help='Run data analysis after simulation')
    parser.add_argument('--report', 
                       action='store_true',
                       help='Generate flight report after simulation')
    
    args = parser.parse_args()
    
    print(f"Starting AirSim drone simulation...")
    print(f"Flight pattern: {args.pattern}")
    print(f"Configuration: {args.config}")
    
    # Initialize simulator
    simulator = DroneSimulator(args.config)
    
    try:
        # Run simulation
        success = simulator.run_simulation(args.pattern)
        
        if success:
            print("✅ Simulation completed successfully!")
            
            if args.analyze or args.report:
                print("\nAnalyzing collected data...")
                analyzer = DataAnalyzer()
                
                try:
                    # Load and analyze data
                    analyzer.load_telemetry_data()
                    stats = analyzer.analyze_flight_path()
                    
                    print("\n📊 Flight Statistics:")
                    for key, value in stats.items():
                        print(f"  {key}: {value}")
                    
                    if args.analyze:
                        # Create visualization
                        plot_path = f"flight_analysis_{args.pattern}.png"
                        analyzer.plot_flight_path(plot_path)
                        print(f"📈 Flight path plot saved to: {plot_path}")
                    
                    if args.report:
                        # Generate report
                        report_path = f"flight_report_{args.pattern}.md"
                        analyzer.create_flight_report(output_path=report_path)
                        print(f"📄 Flight report saved to: {report_path}")
                        
                except Exception as e:
                    print(f"❌ Data analysis failed: {e}")
        else:
            print("❌ Simulation failed!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️  Simulation interrupted by user")
        simulator.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
