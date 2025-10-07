# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import os
import cv2
import torch
import numpy as np
import gradio as gr
import sys
import shutil
from datetime import datetime
import glob
import gc
import time
import tempfile

sys.path.append("vggt/")

from visual_util import predictions_to_glb
from vggt.models.vggt import VGGT
from vggt.utils.load_fn import load_and_preprocess_images
from vggt.utils.pose_enc import pose_encoding_to_extri_intri
from vggt.utils.geometry import unproject_depth_map_to_point_map
import boto3
from botocore import UNSIGNED
from botocore.config import Config

device = "cuda" if torch.cuda.is_available() else "cpu"

print("Initializing and loading VGGT model...")
# model = VGGT.from_pretrained("facebook/VGGT-1B")  # another way to load the model

model = VGGT()
_URL = "https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt"
model.load_state_dict(torch.hub.load_state_dict_from_url(_URL))

model.eval()
model = model.to(device)

# -------------------------------------------------------------------------
# Public S3 Helper Functions (No Credentials Required)
# -------------------------------------------------------------------------
def create_s3_client(region='us-east-1'):
    """Create S3 client for public buckets (no credentials needed)."""
    return boto3.client('s3', config=Config(signature_version=UNSIGNED), region_name=region)

def list_public_bucket_images(bucket_name, prefix=''):
    """List images in S3 bucket (tries credentials first, then public access)."""
    try:
        s3_client = create_s3_client()
        response = s3_client.list_objects_v2(Bucket=bucket_name, Prefix=prefix)
        
        image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'}
        images = []
        
        if 'Contents' in response:
            for obj in response['Contents']:
                key = obj['Key']
                if any(key.lower().endswith(ext) for ext in image_extensions):
                    images.append(key)
        
        return sorted(images)
    except Exception as e:
        print(f"Error listing images: {e}")
        return []

def download_public_s3_images(bucket_name, image_keys, local_dir):
    """Download images from S3 bucket (tries credentials first, then public access)."""
    try:
        s3_client = create_s3_client()
        os.makedirs(local_dir, exist_ok=True)
        downloaded_files = []
        
        for i, key in enumerate(image_keys):
            filename = os.path.basename(key)
            if not filename:
                filename = f"image_{i:04d}.jpg"
            
            local_path = os.path.join(local_dir, filename)
            s3_client.download_file(bucket_name, key, local_path)
            downloaded_files.append(local_path)
            print(f"Downloaded: {key} -> {local_path}")
        
        return downloaded_files
    except Exception as e:
        print(f"Error downloading images: {e}")
        return []

def upload_glb_to_public_s3(bucket_name, glb_file_path, s3_key):
    """Upload GLB file to S3 bucket (tries credentials first, then public access)."""
    try:
        s3_client = create_s3_client()
        s3_client.upload_file(glb_file_path, bucket_name, s3_key)
        print(f"Uploaded: {glb_file_path} -> s3://{bucket_name}/{s3_key}")
        return True
    except Exception as e:
        print(f"Error uploading GLB: {e}")
        return False

def list_public_glb_files(bucket_name, prefix='reconstructions/'):
    """List GLB files in S3 bucket (tries credentials first, then public access)."""
    try:
        s3_client = create_s3_client()
        response = s3_client.list_objects_v2(Bucket=bucket_name, Prefix=prefix)
        
        glb_files = []
        if 'Contents' in response:
            for obj in response['Contents']:
                key = obj['Key']
                if key.lower().endswith('.glb'):
                    glb_files.append(key)
        
        return sorted(glb_files, reverse=True)
    except Exception as e:
        print(f"Error listing GLB files: {e}")
        return []

# -------------------------------------------------------------------------
# 1) Core model inference
# -------------------------------------------------------------------------
def run_model(target_dir, model) -> dict:
    """
    Run the VGGT model on images in the 'target_dir/images' folder and return predictions.
    """
    print(f"Processing images from {target_dir}")

    # Device check
    if device == "cpu":
        print("Warning: Running on CPU. This will be slow.")

    # Load and preprocess images
    image_names = glob.glob(os.path.join(target_dir, "images", "*"))
    image_names = sorted(image_names)
    print(f"Found {len(image_names)} images")
    if len(image_names) == 0:
        raise ValueError("No images found. Check your upload.")

    images = load_and_preprocess_images(image_names).to(device)
    print(f"Preprocessed images shape: {images.shape}")

    # Run inference
    print("Running inference...")
    dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16

    with torch.no_grad():
        with torch.cuda.amp.autocast(dtype=dtype):
            images = images[None]  # add batch dimension
            aggregated_tokens_list, ps_idx = model.aggregator(images)

        # Predict Cameras
        pose_enc = model.camera_head(aggregated_tokens_list)[-1]
        # Extrinsic and intrinsic matrices, following OpenCV convention (camera from world)
        extrinsic, intrinsic = pose_encoding_to_extri_intri(pose_enc, images.shape[-2:])

        # Predict Depth Maps
        depth_map, depth_conf = model.depth_head(aggregated_tokens_list, images, ps_idx)

        # Predict Point Maps
        point_map, point_conf = model.point_head(aggregated_tokens_list, images, ps_idx)

        # Construct 3D Points from Depth Maps and Cameras
        # which usually leads to more accurate 3D points than point map branch
        point_map_by_unprojection = unproject_depth_map_to_point_map(
            depth_map.squeeze(0), extrinsic.squeeze(0), intrinsic.squeeze(0)
        )

    # Save predictions
    predictions = {
        "pose_enc": pose_enc.cpu().numpy(),
        "depth": depth_map.cpu().numpy(),
        "depth_conf": depth_conf.cpu().numpy(),
        "point_map": point_map.cpu().numpy(),
        "point_conf": point_conf.cpu().numpy(),
        "point_map_by_unprojection": point_map_by_unprojection.cpu().numpy(),
        "extrinsic": extrinsic.cpu().numpy(),
        "intrinsic": intrinsic.cpu().numpy(),
    }

    # Save to file
    np.savez(os.path.join(target_dir, "predictions.npz"), **predictions)
    print(f"Predictions saved to {target_dir}/predictions.npz")

    return predictions

# -------------------------------------------------------------------------
# 2) Gradio interface functions
# -------------------------------------------------------------------------
def handle_uploads(input_video, input_images):
    """
    Create a new 'target_dir' + 'images' subfolder, and place user-uploaded
    images or extracted frames from video into it. Return (target_dir, image_paths).
    """
    if input_video is None and input_images is None:
        return "None", []

    # Create a new target directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    target_dir = f"input_images_{timestamp}"
    os.makedirs(target_dir, exist_ok=True)
    os.makedirs(os.path.join(target_dir, "images"), exist_ok=True)

    image_paths = []

    if input_video is not None:
        # Extract frames from video
        cap = cv2.VideoCapture(input_video)
        frame_count = 0
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_interval = max(1, int(fps))  # Extract one frame per second

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if frame_count % frame_interval == 0:
                frame_path = os.path.join(target_dir, "images", f"frame_{frame_count:04d}.jpg")
                cv2.imwrite(frame_path, frame)
                image_paths.append(frame_path)

            frame_count += 1

        cap.release()
        print(f"Extracted {len(image_paths)} frames from video")

    if input_images is not None:
        # Copy uploaded images
        for i, image_file in enumerate(input_images):
            image_path = os.path.join(target_dir, "images", f"image_{i:04d}.jpg")
            shutil.copy2(image_file, image_path)
            image_paths.append(image_path)

    return target_dir, image_paths

def handle_s3_uploads(bucket_name, image_keys):
    """
    Handle S3 bucket image selection and download (public bucket - no credentials needed).
    """
    if not bucket_name:
        return "None", [], "Please select a bucket"
    
    if not image_keys:
        return "None", [], "Please select images from the bucket"
    
    # Create a new target directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    target_dir = f"s3_images_{timestamp}"
    images_dir = os.path.join(target_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    
    try:
        # Download images from public S3
        downloaded = download_public_s3_images(bucket_name, image_keys, images_dir)
        
        if not downloaded:
            return "None", [], "Failed to download images from S3"
        
        return target_dir, downloaded, f"Downloaded {len(downloaded)} images from S3 bucket '{bucket_name}'"
    
    except Exception as e:
        return "None", [], f"Error processing S3 images: {str(e)}"

def update_gallery_on_upload(input_video, input_images):
    """
    Whenever user uploads or changes files, immediately handle them
    and show in the gallery. Return (target_dir, image_paths).
    """
    target_dir, image_paths = handle_uploads(input_video, input_images)
    return None, target_dir, image_paths, ""

def update_gallery_on_s3_upload(bucket_name, image_keys):
    """
    Handle S3 upload and update gallery (public bucket - no credentials needed).
    """
    target_dir, image_paths, message = handle_s3_uploads(bucket_name, image_keys)
    return None, target_dir, image_paths, message

def clear_fields():
    """
    Clears the 3D viewer, the stored target_dir, and empties the gallery.
    """
    return None, "None", [], ""

def update_log():
    """
    Display a quick log message while waiting.
    """
    return "🔄 Processing... Please wait."

def gradio_demo(
    target_dir,
    conf_thres=3.0,
    frame_filter="All",
    mask_black_bg=False,
    mask_white_bg=False,
    show_cam=True,
    mask_sky=False,
    prediction_mode="Pointmap Regression",
):
    """
    Perform reconstruction using the already-created target_dir/images.
    """
    if not os.path.isdir(target_dir) or target_dir == "None":
        return None, "No valid target directory found. Please upload first.", None, None

    start_time = time.time()
    gc.collect()
    torch.cuda.empty_cache()

    # Prepare frame_filter dropdown
    image_names = glob.glob(os.path.join(target_dir, "images", "*"))
    image_names = sorted(image_names)
    frame_options = ["All"] + [f"Frame {i}" for i in range(len(image_names))]

    try:
        # Run the model
        predictions = run_model(target_dir, model)

        # Generate GLB file
        glb_path = os.path.join(target_dir, "reconstruction.glb")
        predictions_to_glb(
            predictions,
            glb_path,
            conf_thres=conf_thres,
            frame_filter=frame_filter,
            mask_black_bg=mask_black_bg,
            mask_white_bg=mask_white_bg,
            show_cam=show_cam,
            mask_sky=mask_sky,
            prediction_mode=prediction_mode,
        )

        end_time = time.time()
        processing_time = end_time - start_time

        return (
            glb_path,
            f"✅ Reconstruction completed in {processing_time:.2f} seconds! GLB file generated.",
            frame_options,
            image_names,
        )

    except Exception as e:
        return None, f"❌ Error during reconstruction: {str(e)}", None, None

def gradio_demo_with_s3_upload(
    target_dir,
    bucket_name,
    scene_name,
    conf_thres=3.0,
    frame_filter="All",
    mask_black_bg=False,
    mask_white_bg=False,
    show_cam=True,
    mask_sky=False,
    prediction_mode="Pointmap Regression",
):
    """
    Perform reconstruction and upload GLB to S3.
    """
    # First run the normal reconstruction
    glb_path, message, frame_options, image_names = gradio_demo(
        target_dir, conf_thres, frame_filter, mask_black_bg, 
        mask_white_bg, show_cam, mask_sky, prediction_mode
    )
    
    # If reconstruction was successful, upload to S3
    if glb_path and os.path.exists(glb_path) and bucket_name:
        try:
            # Generate S3 key
            if not scene_name:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                scene_name = f"vggt_reconstruction_{timestamp}"
            
            s3_key = f"reconstructions/{scene_name}.glb"
            success = upload_glb_to_public_s3(bucket_name, glb_path, s3_key)
            
            if success:
                message += f"\n📤 Successfully uploaded GLB to s3://{bucket_name}/{s3_key}"
            else:
                message += "\n❌ Failed to upload GLB file to S3"
        except Exception as e:
            message += f"\n❌ S3 upload failed: {str(e)}"
    
    return glb_path, message, frame_options, image_names

def update_visualization(
    target_dir,
    conf_thres,
    frame_filter,
    mask_black_bg,
    mask_white_bg,
    show_cam,
    mask_sky,
    prediction_mode,
):
    """
    Update the 3D visualization based on the current settings.
    """
    if not target_dir or target_dir == "None" or not os.path.isdir(target_dir):
        return None, "No reconstruction available. Please click the Reconstruct button first."

    predictions_path = os.path.join(target_dir, "predictions.npz")
    if not os.path.exists(predictions_path):
        return None, f"No reconstruction available at {predictions_path}. Please run 'Reconstruct' first."

    key_list = [
        "pose_enc",
        "depth",
        "depth_conf",
        "point_map",
        "point_conf",
        "point_map_by_unprojection",
        "extrinsic",
        "intrinsic",
    ]

    try:
        predictions = {}
        with np.load(predictions_path) as data:
            for key in key_list:
                if key in data:
                    predictions[key] = data[key]

        # Generate new GLB with updated settings
        glb_path = os.path.join(target_dir, "reconstruction.glb")
        predictions_to_glb(
            predictions,
            glb_path,
            conf_thres=conf_thres,
            frame_filter=frame_filter,
            mask_black_bg=mask_black_bg,
            mask_white_bg=mask_white_bg,
            show_cam=show_cam,
            mask_sky=mask_sky,
            prediction_mode=prediction_mode,
        )

        return glb_path, "✅ Visualization updated!"

    except Exception as e:
        return None, f"❌ Error updating visualization: {str(e)}"

def load_existing_glb_from_s3(bucket_name, glb_key):
    """
    Load existing GLB file from public S3 (no credentials needed).
    """
    if not bucket_name or not glb_key:
        return None, "Please select a bucket and GLB file"
    
    try:
        # Create temporary directory
        temp_dir = tempfile.mkdtemp()
        local_glb_path = os.path.join(temp_dir, os.path.basename(glb_key))
        
        # Download GLB from S3 (tries credentials first, then public access)
        s3_client = create_s3_client()
        s3_client.download_file(bucket_name, glb_key, local_glb_path)
        
        return local_glb_path, f"✅ Loaded GLB file: {glb_key}"
    
    except Exception as e:
        return None, f"❌ Error loading GLB from S3: {str(e)}"

# -------------------------------------------------------------------------
# 3) Gradio interface
# -------------------------------------------------------------------------
theme = gr.themes.Soft(
    primary_hue="blue",
    secondary_hue="green",
    neutral_hue="slate",
)

with gr.Blocks(
    theme=theme,
    css="""
    .custom-log * {
        font-style: italic;
        font-size: 22px !important;
        background-image: linear-gradient(120deg, #0ea5e9 0%, #6ee7b7 60%, #34d399 100%);
        -webkit-background-clip: text;
        background-clip: text;
        font-weight: bold !important;
        color: transparent !important;
        text-align: center !important;
    }
    
    .example-log * {
        font-style: italic;
        font-size: 16px !important;
        background-image: linear-gradient(120deg, #0ea5e9 0%, #6ee7b7 60%, #34d399 100%);
        -webkit-background-clip: text;
        background-clip: text;
        color: transparent !important;
    }
    
    #my_radio .wrap {
        display: flex;
        flex-wrap: nowrap;
        justify-content: center;
        align-items: center;
    }

    #my_radio .wrap label {
        display: flex;
        width: 50%;
        justify-content: center;
        align-items: center;
        margin: 0;
        padding: 10px 0;
        box-sizing: border-box;
    }
    """,
) as demo:
    # State variables
    is_example = gr.Textbox(label="is_example", visible=False, value="None")
    num_images = gr.Textbox(label="num_images", visible=False, value="None")
    target_dir_output = gr.Textbox(label="Target Dir", visible=False, value="None")

    gr.HTML(
        """
    <h1>🏛️ VGGT: Visual Geometry Grounded Transformer with Public S3 Integration</h1>
    <p>
    <a href="https://github.com/facebookresearch/vggt">🐙 GitHub Repository</a> |
    <a href="#">Project Page</a>
    </p>

    <div style="font-size: 16px; line-height: 1.5;">
    <p>Upload images/videos locally OR select images from your <strong>public AWS S3 bucket</strong> to create 3D reconstructions. <strong>No AWS credentials required!</strong></p>

    <h3>Getting Started:</h3>
    <ol>
        <li><strong>Choose Input Method:</strong> Upload files locally or enter your public S3 bucket name</li>
        <li><strong>Preview:</strong> Your images will appear in the gallery</li>
        <li><strong>Reconstruct:</strong> Click "Reconstruct" to start 3D reconstruction</li>
        <li><strong>Upload to S3:</strong> Optionally upload the GLB file to your public S3 bucket</li>
        <li><strong>Load Existing:</strong> Load previously generated GLB files from S3</li>
    </ol>
    <p><strong style="color: #0ea5e9;">Please note:</strong> <span style="color: #0ea5e9; font-weight: bold;">VGGT typically reconstructs a scene in less than 1 second. However, visualizing 3D points may take tens of seconds due to third-party rendering, which are independent of VGGT's processing time. </span></p>
    <p><strong style="color: #22c55e;">Public S3 Mode:</strong> <span style="color: #22c55e; font-weight: bold;">This demo works with public S3 buckets - no AWS credentials needed! Just enter your bucket name.</span></p>
    </div>
    """
    )

    with gr.Tabs():
        # Tab 1: Local Upload
        with gr.Tab("📁 Local Upload"):
            with gr.Row():
                with gr.Column(scale=2):
                    input_video = gr.Video(label="Upload Video", interactive=True)
                    input_images = gr.File(file_count="multiple", label="Upload Images", interactive=True)

                    image_gallery = gr.Gallery(
                        label="Preview",
                        columns=4,
                        height="300px",
                        show_download_button=True,
                        object_fit="contain",
                        preview=True,
                    )

                with gr.Column(scale=3):
                    reconstruction_output = gr.Model3D(height=520, zoom_speed=0.5, pan_speed=0.5)

                with gr.Row():
                    submit_btn = gr.Button("Reconstruct", scale=1, variant="primary")
                    clear_btn = gr.ClearButton(
                        [input_video, input_images, reconstruction_output, image_gallery],
                        scale=1,
                    )

        # Tab 2: S3 Integration
        with gr.Tab("☁️ AWS S3 Integration"):
            with gr.Row():
                with gr.Column(scale=1):
                    # Public S3 Configuration (No Credentials Needed)
                    gr.Markdown("### Public S3 Bucket Configuration")
                    gr.Markdown("*No AWS credentials required - using public bucket access*")
                    
                    # S3 Bucket Selection
                    gr.Markdown("### S3 Bucket Selection")
                    bucket_name = gr.Textbox(
                        label="Enter S3 Bucket Name",
                        placeholder="Enter your public bucket name (e.g., 64722)",
                        value=""
                    )
                    
                    # Image Selection
                    gr.Markdown("### Image Selection")
                    image_keys = gr.CheckboxGroup(
                        label="Select Images from Bucket",
                        choices=[],
                        interactive=True
                    )
                    refresh_images_btn = gr.Button("🔄 Refresh Images", size="sm")
                    
                    # Scene Configuration
                    gr.Markdown("### Scene Configuration")
                    scene_name = gr.Textbox(
                        label="Scene Name (for GLB file)",
                        placeholder="Enter a name for your reconstruction",
                        value=""
                    )
                    
                    # Action Buttons
                    with gr.Row():
                        s3_submit_btn = gr.Button("📥 Load from S3", variant="primary")
                        s3_clear_btn = gr.Button("🗑️ Clear", variant="secondary")
                
                with gr.Column(scale=2):
                    s3_image_gallery = gr.Gallery(
                        label="S3 Images Preview",
                        columns=4,
                        height="300px",
                        show_download_button=True,
                        object_fit="contain",
                        preview=True,
                    )
                
                with gr.Column(scale=2):
                    s3_reconstruction_output = gr.Model3D(height=520, zoom_speed=0.5, pan_speed=0.5)
                    
                    # S3 Reconstruction Button
                    s3_reconstruct_btn = gr.Button("🔨 Reconstruct & Upload to S3", variant="primary", size="lg")

        # Tab 3: Load Existing GLB
        with gr.Tab("📦 Load Existing GLB from S3"):
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### Load Previous Reconstructions")
                    existing_bucket = gr.Textbox(
                        label="Enter S3 Bucket Name",
                        placeholder="Enter your public bucket name (e.g., 64722)",
                        value=""
                    )
                    existing_glb_files = gr.Dropdown(
                        label="Select GLB File",
                        choices=[],
                        interactive=True
                    )
                    load_glb_btn = gr.Button("📥 Load GLB from S3", variant="primary")
                    refresh_existing_btn = gr.Button("🔄 Refresh Files", size="sm")
                
                with gr.Column(scale=2):
                    existing_glb_output = gr.Model3D(height=520, zoom_speed=0.5, pan_speed=0.5)
                    existing_glb_log = gr.Textbox(label="Status", interactive=False)

    # Visualization Controls (shared across tabs)
    with gr.Row():
        with gr.Column():
            gr.Markdown("### Visualization Controls")
            conf_thres = gr.Slider(
                minimum=0.0,
                maximum=10.0,
                value=3.0,
                step=0.1,
                label="Confidence Threshold",
                info="Higher values show fewer, more confident points",
            )
            frame_filter = gr.Dropdown(
                label="Show Points from Frame",
                choices=["All"],
                value="All",
                interactive=True,
            )
            show_cam = gr.Checkbox(label="Show Camera", value=True)
            mask_black_bg = gr.Checkbox(label="Filter Black Background", value=False)
            mask_white_bg = gr.Checkbox(label="Filter White Background", value=False)
            mask_sky = gr.Checkbox(label="Filter Sky", value=False)
            prediction_mode = gr.Radio(
                label="Select a Prediction Mode",
                choices=["Depthmap and Camera Branch", "Pointmap Regression"],
                value="Pointmap Regression",
                info="Choose between depth-based or point-based reconstruction",
            )

    log_output = gr.Textbox(label="Status", interactive=False)

    # -------------------------------------------------------------------------
    # Event Handlers
    # -------------------------------------------------------------------------
    
    # Local upload handlers
    submit_btn.click(fn=clear_fields, inputs=[], outputs=[reconstruction_output]).then(
        fn=update_log, inputs=[], outputs=[log_output]
    ).then(
        fn=gradio_demo,
        inputs=[
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        outputs=[reconstruction_output, log_output, frame_filter, image_gallery],
    )

    # S3 images refresh
    def refresh_s3_images(bucket):
        if not bucket:
            return gr.CheckboxGroup(choices=[]), "Please enter a bucket name first"
        try:
            images = list_public_bucket_images(bucket)
            if not images:
                return gr.CheckboxGroup(choices=[]), f"No images found in bucket '{bucket}'"
            return gr.CheckboxGroup(choices=images), f"Found {len(images)} images in bucket '{bucket}'"
        except Exception as e:
            return gr.CheckboxGroup(choices=[]), f"Error: {str(e)}"
    
    refresh_images_btn.click(
        fn=refresh_s3_images,
        inputs=[bucket_name],
        outputs=[image_keys, log_output]
    )
    
    # S3 image loading
    s3_submit_btn.click(
        fn=update_gallery_on_s3_upload,
        inputs=[bucket_name, image_keys],
        outputs=[s3_reconstruction_output, target_dir_output, s3_image_gallery, log_output]
    )
    
    # S3 reconstruction and upload
    s3_reconstruct_btn.click(
        fn=gradio_demo_with_s3_upload,
        inputs=[
            target_dir_output,
            bucket_name,
            scene_name,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        outputs=[s3_reconstruction_output, log_output, frame_filter, s3_image_gallery],
    )
    
    # Load existing GLB files
    def refresh_existing_files(bucket):
        if not bucket:
            return gr.Dropdown(choices=[]), "Please enter a bucket name first"
        try:
            glb_files = list_public_glb_files(bucket)
            if not glb_files:
                return gr.Dropdown(choices=[]), f"No GLB files found in bucket '{bucket}'"
            return gr.Dropdown(choices=glb_files), f"Found {len(glb_files)} GLB files in bucket '{bucket}'"
        except Exception as e:
            return gr.Dropdown(choices=[]), f"Error: {str(e)}"
    
    refresh_existing_btn.click(
        fn=refresh_existing_files,
        inputs=[existing_bucket],
        outputs=[existing_glb_files, existing_glb_log]
    )
    
    load_glb_btn.click(
        fn=load_existing_glb_from_s3,
        inputs=[existing_bucket, existing_glb_files],
        outputs=[existing_glb_output, existing_glb_log]
    )

    # Visualization update handlers
    conf_thres.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    frame_filter.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    mask_black_bg.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    mask_white_bg.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    show_cam.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    mask_sky.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )
    prediction_mode.change(
        update_visualization,
        [
            target_dir_output,
            conf_thres,
            frame_filter,
            mask_black_bg,
            mask_white_bg,
            show_cam,
            mask_sky,
            prediction_mode,
        ],
        [reconstruction_output, log_output],
    )

    # Auto-update gallery for local uploads
    input_video.change(
        fn=update_gallery_on_upload,
        inputs=[input_video, input_images],
        outputs=[reconstruction_output, target_dir_output, image_gallery, log_output],
    )
    input_images.change(
        fn=update_gallery_on_upload,
        inputs=[input_video, input_images],
        outputs=[reconstruction_output, target_dir_output, image_gallery, log_output],
    )

    demo.queue(max_size=20).launch(show_error=True, share=True)
