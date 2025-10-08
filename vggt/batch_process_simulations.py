#!/usr/bin/env python3
"""
Batch Processing Script for Drone Simulation Reconstructions

This script automatically:
1. Takes a folder of drone simulation images
2. Runs VGGT 3D reconstruction
3. Uploads results (GLB + images) to AWS S3
4. Tracks multiple simulation runs

Usage:
    python batch_process_simulations.py --input-folder /path/to/drone/images
    
    Or set defaults in the script and just run:
    python batch_process_simulations.py
"""

import os
import sys
import glob
import argparse
import torch
import numpy as np
from datetime import datetime
import boto3
from pathlib import Path
import shutil

# Add vggt to path
sys.path.append("vggt/")

from visual_util import predictions_to_glb
from vggt.models.vggt import VGGT
from vggt.utils.load_fn import load_and_preprocess_images
from vggt.utils.pose_enc import pose_encoding_to_extri_intri
from vggt.utils.geometry import unproject_depth_map_to_point_map

# ============================================================================
# CONFIGURATION - Edit these settings
# ============================================================================

# AWS S3 Configuration
S3_BUCKET_NAME = "vandyawshackathon2025"  # Your bucket name
S3_BASE_PREFIX = "SimulationRuns"  # Base folder in S3 for all simulations
AWS_REGION = "us-east-1"

# Processing Configuration
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
CONFIDENCE_THRESHOLD = 3.0
PREDICTION_MODE = "Pointmap Regression"  # or "Depthmap and Camera Branch"

# ============================================================================
# Helper Functions
# ============================================================================

def setup_s3_client():
    """Create S3 client with credentials from environment or AWS CLI config."""
    try:
        # This will use credentials from AWS CLI config or environment variables
        return boto3.client('s3', region_name=AWS_REGION)
    except Exception as e:
        print(f"Error creating S3 client: {e}")
        print("Make sure AWS credentials are configured (run 'aws configure')")
        return None


def load_model():
    """Load the VGGT model."""
    print("Loading VGGT model...")
    model = VGGT()
    _URL = "https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt"
    model.load_state_dict(torch.hub.load_state_dict_from_url(_URL))
    model.eval()
    model = model.to(DEVICE)
    print(f"Model loaded on {DEVICE}")
    return model


def run_reconstruction(image_folder, model, output_dir):
    """
    Run VGGT reconstruction on images in the folder.
    
    Args:
        image_folder: Path to folder containing images
        model: VGGT model
        output_dir: Where to save results
        
    Returns:
        Path to generated GLB file, or None if failed
    """
    print(f"\nProcessing images from: {image_folder}")
    
    # Find all image files
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
    image_paths = []
    for ext in image_extensions:
        image_paths.extend(glob.glob(os.path.join(image_folder, ext)))
    
    image_paths = sorted(image_paths)
    
    if len(image_paths) == 0:
        print(f"❌ No images found in {image_folder}")
        return None
    
    print(f"Found {len(image_paths)} images")
    
    # Load and preprocess images
    print("Loading and preprocessing images...")
    images = load_and_preprocess_images(image_paths).to(DEVICE)
    print(f"Preprocessed images shape: {images.shape}")
    
    # Run inference
    print("Running VGGT inference...")
    dtype = torch.bfloat16 if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    
    with torch.no_grad():
        with torch.cuda.amp.autocast(dtype=dtype):
            predictions = model(images)
    
    # Convert pose encoding to extrinsic and intrinsic matrices
    print("Converting to camera parameters...")
    extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"], images.shape[-2:])
    predictions["extrinsic"] = extrinsic
    predictions["intrinsic"] = intrinsic
    
    # Convert tensors to numpy
    for key in predictions.keys():
        if isinstance(predictions[key], torch.Tensor):
            predictions[key] = predictions[key].cpu().numpy().squeeze(0)
    predictions['pose_enc_list'] = None
    
    # Generate world points from depth map
    print("Computing 3D world points...")
    depth_map = predictions["depth"]
    world_points = unproject_depth_map_to_point_map(depth_map, predictions["extrinsic"], predictions["intrinsic"])
    predictions["world_points_from_depth"] = world_points
    
    # Save predictions
    os.makedirs(output_dir, exist_ok=True)
    predictions_path = os.path.join(output_dir, "predictions.npz")
    np.savez(predictions_path, **predictions)
    print(f"Saved predictions to: {predictions_path}")
    
    # Generate GLB file
    print("Generating 3D model (GLB)...")
    glb_path = os.path.join(output_dir, "reconstruction.glb")
    
    glbscene = predictions_to_glb(
        predictions,
        conf_thres=CONFIDENCE_THRESHOLD,
        filter_by_frames="All",
        mask_black_bg=False,
        mask_white_bg=False,
        show_cam=True,
        mask_sky=False,
        target_dir=output_dir,
        prediction_mode=PREDICTION_MODE,
    )
    glbscene.export(file_obj=glb_path)
    
    print(f"✅ GLB file created: {glb_path}")
    
    # Clean up GPU memory
    torch.cuda.empty_cache()
    
    return glb_path


def upload_raw_images_to_s3(images_folder, simulation_name, s3_client):
    """
    Upload ONLY raw images to S3 immediately (before processing).
    
    Args:
        images_folder: Local folder containing raw images
        simulation_name: Name for this simulation run
        s3_client: Boto3 S3 client
        
    Returns:
        True if successful, False otherwise
    """
    if s3_client is None:
        print("⚠️  S3 client not available. Images will be uploaded after reconstruction.")
        return False
    
    print(f"\n📸 Uploading raw images to S3 (backup before processing)...")
    
    # S3 path structure: SimulationRuns/{simulation_name}/images/
    s3_prefix = f"{S3_BASE_PREFIX}/{simulation_name}/images"
    
    try:
        # Find all image files
        image_extensions = ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']
        image_files = []
        
        for file in os.listdir(images_folder):
            if any(file.endswith(ext) for ext in image_extensions):
                local_path = os.path.join(images_folder, file)
                s3_key = f"{s3_prefix}/{file}"
                image_files.append((local_path, s3_key))
        
        if not image_files:
            print("  ⚠️  No images found to upload")
            return False
        
        print(f"  Uploading {len(image_files)} images to S3...")
        for i, (local_path, s3_key) in enumerate(image_files, 1):
            filename = os.path.basename(local_path)
            # Show progress every 10 images or for small sets
            if i % 10 == 0 or len(image_files) <= 20:
                print(f"    [{i}/{len(image_files)}] {filename}")
            s3_client.upload_file(local_path, S3_BUCKET_NAME, s3_key)
        
        print(f"  ✅ All {len(image_files)} images uploaded to S3!")
        print(f"     S3 Location: s3://{S3_BUCKET_NAME}/{s3_prefix}/")
        return True
        
    except Exception as e:
        print(f"  ❌ Error uploading images: {e}")
        return False


def upload_reconstruction_to_s3(local_folder, simulation_name, s3_client):
    """
    Upload reconstruction results to S3 (GLB, predictions, metadata).
    Images are already uploaded, so we skip them here.
    
    Args:
        local_folder: Local folder containing results
        simulation_name: Name for this simulation run
        s3_client: Boto3 S3 client
        
    Returns:
        True if successful, False otherwise
    """
    if s3_client is None:
        print("❌ S3 client not available. Skipping upload.")
        return False
    
    print(f"\n📤 Uploading reconstruction results to S3...")
    
    # S3 path structure: SimulationRuns/{simulation_name}/
    s3_prefix = f"{S3_BASE_PREFIX}/{simulation_name}"
    
    try:
        # Find all files to upload (excluding images folder)
        files_to_upload = []
        
        for root, dirs, files in os.walk(local_folder):
            # Skip the images folder (already uploaded)
            if 'images' in dirs:
                dirs.remove('images')
            
            for file in files:
                local_path = os.path.join(root, file)
                # Calculate relative path for S3 key
                rel_path = os.path.relpath(local_path, local_folder)
                s3_key = f"{s3_prefix}/{rel_path}".replace("\\", "/")
                files_to_upload.append((local_path, s3_key))
        
        # Upload reconstruction files
        print(f"  Uploading {len(files_to_upload)} reconstruction files...")
        for local_path, s3_key in files_to_upload:
            filename = os.path.basename(local_path)
            print(f"    ✓ {filename}")
            s3_client.upload_file(local_path, S3_BUCKET_NAME, s3_key)
        
        print(f"\n✅ Successfully uploaded reconstruction to S3")
        print(f"   🌐 S3 Location: s3://{S3_BUCKET_NAME}/{s3_prefix}/")
        return True
        
    except Exception as e:
        print(f"❌ Error uploading reconstruction: {e}")
        return False


def create_metadata_file(output_dir, simulation_name, image_count, processing_time):
    """Create a metadata file with simulation info."""
    metadata = {
        "simulation_name": simulation_name,
        "image_count": image_count,
        "processing_time_seconds": processing_time,
        "timestamp": datetime.now().isoformat(),
        "device": DEVICE,
        "confidence_threshold": CONFIDENCE_THRESHOLD,
        "prediction_mode": PREDICTION_MODE
    }
    
    metadata_path = os.path.join(output_dir, "metadata.txt")
    with open(metadata_path, 'w') as f:
        for key, value in metadata.items():
            f.write(f"{key}: {value}\n")
    
    return metadata_path


# ============================================================================
# Main Processing Function
# ============================================================================

def process_simulation(input_folder, simulation_name=None, upload_to_s3_flag=True):
    """
    Process a single simulation: upload images, reconstruct, upload results.
    
    WORKFLOW:
    1. Upload raw images to S3 immediately (backup)
    2. Run VGGT reconstruction (may take 10-30 minutes)
    3. Upload reconstruction results to S3
    
    Args:
        input_folder: Path to folder with drone images
        simulation_name: Name for this simulation (auto-generated if None)
        upload_to_s3_flag: Whether to upload to S3
        
    Returns:
        True if successful, False otherwise
    """
    import time
    start_time = time.time()
    
    # Generate simulation name if not provided
    if simulation_name is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = os.path.basename(os.path.normpath(input_folder))
        simulation_name = f"{folder_name}_{timestamp}"
    
    print("="*70)
    print(f"Processing Simulation: {simulation_name}")
    print("="*70)
    
    # Verify input folder exists
    if not os.path.exists(input_folder) or not os.path.isdir(input_folder):
        print(f"❌ Input folder not found: {input_folder}")
        return False
    
    # Count images
    image_extensions = ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']
    image_count = len([f for f in os.listdir(input_folder) if any(f.endswith(ext) for ext in image_extensions)])
    print(f"Found {image_count} images in input folder")
    
    # STEP 1: Upload raw images to S3 FIRST (immediate backup)
    s3_client = None
    if upload_to_s3_flag:
        s3_client = setup_s3_client()
        if s3_client:
            print("\n" + "="*70)
            print("STEP 1: Uploading raw images to S3 (backup before processing)")
            print("="*70)
            upload_raw_images_to_s3(input_folder, simulation_name, s3_client)
        else:
            print("⚠️  S3 upload skipped - credentials not available")
    
    # STEP 2: Create local output directory and copy images
    print("\n" + "="*70)
    print("STEP 2: Running VGGT Reconstruction")
    print("="*70)
    
    output_dir = os.path.join("simulation_outputs", simulation_name)
    os.makedirs(output_dir, exist_ok=True)
    
    # Copy input images to output directory for local archival
    images_output = os.path.join(output_dir, "images")
    print(f"Copying images to local output: {images_output}")
    shutil.copytree(input_folder, images_output, dirs_exist_ok=True)
    
    # Load model
    model = load_model()
    
    # Run reconstruction
    glb_path = run_reconstruction(images_output, model, output_dir)
    
    if glb_path is None:
        print("❌ Reconstruction failed")
        print("💡 Note: Raw images are already backed up in S3!")
        return False
    
    processing_time = time.time() - start_time
    
    # Create metadata
    print("\nCreating metadata...")
    create_metadata_file(output_dir, simulation_name, image_count, processing_time)
    
    print(f"\n⏱️  Total processing time: {processing_time:.2f} seconds")
    print(f"📁 Local results saved to: {output_dir}")
    
    # STEP 3: Upload reconstruction results to S3
    if upload_to_s3_flag and s3_client:
        print("\n" + "="*70)
        print("STEP 3: Uploading reconstruction results to S3")
        print("="*70)
        upload_reconstruction_to_s3(output_dir, simulation_name, s3_client)
    
    print("\n" + "="*70)
    print("✅ PROCESSING COMPLETE!")
    print("="*70)
    print(f"📸 Raw images: Already in S3!")
    print(f"🎮 Reconstruction: s3://{S3_BUCKET_NAME}/{S3_BASE_PREFIX}/{simulation_name}/")
    
    return True


# ============================================================================
# Command Line Interface
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Batch process drone simulation images with VGGT and upload to S3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python batch_process_simulations.py --input-folder ./my_drone_run
  python batch_process_simulations.py --input-folder ./run1 --name "Test_Flight_01"
  python batch_process_simulations.py --input-folder ./run1 --no-upload
        """
    )
    
    parser.add_argument(
        '--input-folder', '-i',
        type=str,
        required=True,
        help='Path to folder containing drone simulation images'
    )
    
    parser.add_argument(
        '--name', '-n',
        type=str,
        default=None,
        help='Name for this simulation run (auto-generated if not provided)'
    )
    
    parser.add_argument(
        '--no-upload',
        action='store_true',
        help='Skip uploading to S3 (only process locally)'
    )
    
    parser.add_argument(
        '--bucket',
        type=str,
        default=None,
        help=f'S3 bucket name (default: {S3_BUCKET_NAME})'
    )
    
    args = parser.parse_args()
    
    # Override bucket if provided
    if args.bucket:
        global S3_BUCKET_NAME
        S3_BUCKET_NAME = args.bucket
    
    # Process the simulation
    success = process_simulation(
        input_folder=args.input_folder,
        simulation_name=args.name,
        upload_to_s3_flag=not args.no_upload
    )
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

