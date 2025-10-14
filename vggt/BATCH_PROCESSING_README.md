# Drone Simulation Batch Processing

Automatically process drone simulation images with VGGT and upload to AWS S3!

## 🚀 Quick Start

### 1. Setup AWS Credentials (One-time)

```bash
# Install AWS CLI if you haven't already
pip install awscli

# Configure your credentials
aws configure
# Enter your AWS Access Key ID
# Enter your AWS Secret Access Key
# Enter region: us-east-1
# Enter output format: json
```

### 2. Process a Single Simulation

```bash
# Basic usage - process and upload
python batch_process_simulations.py --input-folder /path/to/your/drone/images

# With custom name
python batch_process_simulations.py --input-folder ./my_flight --name "Test_Flight_01"

# Process locally only (no upload)
python batch_process_simulations.py --input-folder ./my_flight --no-upload

# Use different S3 bucket
python batch_process_simulations.py --input-folder ./my_flight --bucket my-other-bucket
```

### 3. Process Multiple Simulations

Edit `process_multiple_simulations.py`:

```python
SIMULATIONS = [
    {
        "input_folder": "C:/DroneData/Run1",
        "name": "Morning_Flight_01",
        "upload": True
    },
    {
        "input_folder": "C:/DroneData/Run2",
        "name": "Afternoon_Flight_01",
        "upload": True
    },
]
```

Then run:
```bash
python process_multiple_simulations.py
```

## 📋 How It Works

**New Workflow - Images Uploaded FIRST!**

1. **📸 Step 1: Upload raw images to S3** (immediate backup!)
2. **🔨 Step 2: Run VGGT reconstruction** (10-30 minutes for ~100 images)
3. **📤 Step 3: Upload reconstruction results to S3**

**Key Benefit:** Your raw images are safely backed up in S3 immediately, even if reconstruction takes a long time or fails!

## 📁 What Gets Uploaded to S3

**✅ IMPORTANT: The script uploads BOTH the 3D reconstruction AND all raw images!**

Your S3 bucket will be organized like this:

```
vandyawshackathon2025/
└── SimulationRuns/
    ├── Morning_Flight_01_20251007_143022/
    │   ├── images/                     # 📸 ALL RAW IMAGES (preserved!)
    │   │   ├── image_0001.jpg
    │   │   ├── image_0002.jpg
    │   │   ├── image_0003.jpg
    │   │   └── ... (all your original images)
    │   ├── reconstruction.glb          # 3D model file
    │   ├── predictions.npz             # Raw predictions
    │   └── metadata.txt                # Processing info
    │
    ├── Afternoon_Flight_01_20251007_153045/
    │   ├── images/                     # 📸 More raw images
    │   └── ...
    └── ...
```

**Every simulation run preserves:**
- 📸 All original drone images (in `images/` folder)
- 🎮 3D reconstruction GLB file
- 📊 Raw prediction data
- 📝 Metadata (timestamp, image count, processing time)

## ⚙️ Configuration

Edit these settings in `batch_process_simulations.py`:

```python
# AWS S3 Configuration
S3_BUCKET_NAME = "vandyawshackathon2025"  # Your bucket
S3_BASE_PREFIX = "SimulationRuns"        # Folder in S3
AWS_REGION = "us-east-1"

# Processing Configuration
CONFIDENCE_THRESHOLD = 3.0
PREDICTION_MODE = "Pointmap Regression"
```

## 🎯 Use Cases

### Case 1: Single Flight Processing
You just finished a drone simulation and want to process it:

```bash
python batch_process_simulations.py -i ./circle_patrol_images
```

### Case 2: Batch Process All Today's Flights
Edit `process_multiple_simulations.py`:

```python
AUTO_PROCESS_DIRECTORY = "C:/DroneSimulations/2025-10-07"
```

Run:
```bash
python process_multiple_simulations.py
```

### Case 3: Process Without Uploading (Testing)
```bash
python batch_process_simulations.py -i ./test_images --no-upload
```

## 📊 Output

Each run creates:
- **Local folder**: `simulation_outputs/{simulation_name}/`
- **S3 location**: `s3://your-bucket/SimulationRuns/{simulation_name}/`

The script prints:
```
======================================================================
Processing Simulation: Test_Flight_01
======================================================================
Found 99 images in input folder

======================================================================
STEP 1: Uploading raw images to S3 (backup before processing)
======================================================================

📸 Uploading raw images to S3 (backup before processing)...
  Uploading 99 images to S3...
    [10/99] image_0010.jpg
    [20/99] image_0020.jpg
    [30/99] image_0030.jpg
    ...
    [90/99] image_0090.jpg
  ✅ All 99 images uploaded to S3!
     S3 Location: s3://vandyawshackathon2025/SimulationRuns/Test_Flight_01/images/

======================================================================
STEP 2: Running VGGT Reconstruction
======================================================================
Copying images to local output: simulation_outputs/Test_Flight_01/images
Loading VGGT model...
Found 99 images
Running VGGT inference...
Generating 3D model (GLB)...
✅ GLB file created: simulation_outputs/Test_Flight_01/reconstruction.glb

Creating metadata...

⏱️  Total processing time: 245.32 seconds
📁 Local results saved to: simulation_outputs/Test_Flight_01

======================================================================
STEP 3: Uploading reconstruction results to S3
======================================================================

📤 Uploading reconstruction results to S3...
  Uploading 3 reconstruction files...
    ✓ reconstruction.glb
    ✓ predictions.npz
    ✓ metadata.txt

✅ Successfully uploaded reconstruction to S3
   🌐 S3 Location: s3://vandyawshackathon2025/SimulationRuns/Test_Flight_01/

======================================================================
✅ PROCESSING COMPLETE!
======================================================================
📸 Raw images: Already in S3!
🎮 Reconstruction: s3://vandyawshackathon2025/SimulationRuns/Test_Flight_01/
```

## 🔧 Troubleshooting

### Error: AWS credentials not found
```bash
aws configure  # Set up your credentials
```

### Error: CUDA out of memory
Reduce the number of images or use CPU mode:
```python
DEVICE = "cpu"  # in batch_process_simulations.py
```

### Want to speed up processing?
- Use fewer images (select every Nth image)
- Reduce image resolution before processing
- Use GPU instead of CPU

### Check what's in your S3 bucket
```bash
aws s3 ls s3://vandyawshackathon2025/SimulationRuns/ --recursive
```

## 🎮 Example Workflow

```bash
# 1. Run your drone simulation → saves images to folder

# 2. Process and upload
python batch_process_simulations.py \
    --input-folder "C:/Images/TestImages/circle_patrol_images" \
    --name "Circle_Patrol_Run_01"

# 3. View results
# - Locally: simulation_outputs/Circle_Patrol_Run_01/reconstruction.glb
# - S3: s3://vandyawshackathon2025/SimulationRuns/Circle_Patrol_Run_01/

# 4. Download any run later
aws s3 sync s3://vandyawshackathon2025/SimulationRuns/Circle_Patrol_Run_01/ ./downloaded_run/
```

## 💡 Tips

- **Images are backed up FIRST** - Even if VGGT crashes, your images are safe in S3!
- Use descriptive simulation names: `"Morning_Obstacle_Course_Run_03"`
- Keep original image folder structure organized by date/run
- The script automatically tracks processing time and image count
- GLB files can be viewed in most 3D viewers or web browsers
- All original images are preserved in S3 for reproducibility
- You can safely queue multiple runs - images upload immediately

## 🆘 Need Help?

Check:
1. AWS credentials are configured: `aws s3 ls`
2. Input folder has images: `ls /path/to/images`
3. CUDA available (if using GPU): `python -c "import torch; print(torch.cuda.is_available())"`

