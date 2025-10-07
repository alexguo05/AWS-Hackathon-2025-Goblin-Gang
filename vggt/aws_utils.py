# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import boto3
import os
import tempfile
import json
from datetime import datetime
from typing import List, Optional, Tuple
import gradio as gr

class S3Manager:
    def __init__(self, aws_access_key_id: str = None, aws_secret_access_key: str = None, region_name: str = 'us-east-1', access_point_arn: str = None):
        """
        Initialize S3 manager with AWS credentials.
        If credentials are None, will use default AWS credential chain.
        If access_point_arn is provided, will use S3 Access Point.
        """
        self.access_point_arn = access_point_arn
        self.s3_client = boto3.client(
            's3',
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            region_name=region_name
        )
        
        # If using access point, create access point client
        if access_point_arn:
            self.s3_ap_client = boto3.client(
                's3',
                aws_access_key_id=aws_access_key_id,
                aws_secret_access_key=aws_secret_access_key,
                region_name=region_name,
                endpoint_url=f"https://{self._extract_access_point_id()}.s3-accesspoint.{region_name}.amazonaws.com"
            )
        else:
            self.s3_ap_client = None
    
    def _extract_access_point_id(self) -> str:
        """Extract access point ID from ARN."""
        if self.access_point_arn:
            return self.access_point_arn.split('/')[-1]
        return None
    
    def _get_client(self):
        """Get the appropriate S3 client (access point or regular)."""
        return self.s3_ap_client if self.s3_ap_client else self.s3_client
    
    def list_buckets(self) -> List[str]:
        """List all available S3 buckets."""
        try:
            response = self.s3_client.list_buckets()
            return [bucket['Name'] for bucket in response['Buckets']]
        except Exception as e:
            print(f"Error listing buckets: {e}")
            return []
    
    def list_images_in_bucket(self, bucket_name: str, prefix: str = "") -> List[str]:
        """List all image files in a bucket with given prefix."""
        try:
            client = self._get_client()
            response = client.list_objects_v2(
                Bucket=bucket_name,
                Prefix=prefix
            )
            
            image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'}
            images = []
            
            if 'Contents' in response:
                for obj in response['Contents']:
                    key = obj['Key']
                    if any(key.lower().endswith(ext) for ext in image_extensions):
                        images.append(key)
            
            return sorted(images)
        except Exception as e:
            print(f"Error listing images in bucket {bucket_name}: {e}")
            return []
    
    def download_images_from_s3(self, bucket_name: str, image_keys: List[str], local_dir: str) -> List[str]:
        """Download images from S3 to local directory."""
        downloaded_files = []
        
        try:
            os.makedirs(local_dir, exist_ok=True)
            client = self._get_client()
            
            for i, key in enumerate(image_keys):
                # Create local filename
                filename = os.path.basename(key)
                if not filename:
                    filename = f"image_{i:04d}.jpg"
                
                local_path = os.path.join(local_dir, filename)
                
                # Download from S3
                client.download_file(bucket_name, key, local_path)
                downloaded_files.append(local_path)
                print(f"Downloaded: {key} -> {local_path}")
            
            return downloaded_files
        except Exception as e:
            print(f"Error downloading images: {e}")
            return []
    
    def upload_glb_to_s3(self, bucket_name: str, glb_file_path: str, s3_key: str) -> bool:
        """Upload GLB file to S3."""
        try:
            client = self._get_client()
            client.upload_file(glb_file_path, bucket_name, s3_key)
            print(f"Uploaded GLB file: {glb_file_path} -> s3://{bucket_name}/{s3_key}")
            return True
        except Exception as e:
            print(f"Error uploading GLB file: {e}")
            return False
    
    def list_glb_files_in_bucket(self, bucket_name: str, prefix: str = "") -> List[dict]:
        """List all GLB files in a bucket with metadata."""
        try:
            client = self._get_client()
            response = client.list_objects_v2(
                Bucket=bucket_name,
                Prefix=prefix
            )
            
            glb_files = []
            
            if 'Contents' in response:
                for obj in response['Contents']:
                    key = obj['Key']
                    if key.lower().endswith('.glb'):
                        glb_files.append({
                            'key': key,
                            'last_modified': obj['LastModified'],
                            'size': obj['Size']
                        })
            
            return sorted(glb_files, key=lambda x: x['last_modified'], reverse=True)
        except Exception as e:
            print(f"Error listing GLB files in bucket {bucket_name}: {e}")
            return []
    
    def download_glb_from_s3(self, bucket_name: str, s3_key: str, local_path: str) -> bool:
        """Download GLB file from S3."""
        try:
            client = self._get_client()
            client.download_file(bucket_name, s3_key, local_path)
            print(f"Downloaded GLB: s3://{bucket_name}/{s3_key} -> {local_path}")
            return True
        except Exception as e:
            print(f"Error downloading GLB file: {e}")
            return False
    
    def save_reconstruction_metadata(self, bucket_name: str, metadata: dict, s3_key: str) -> bool:
        """Save reconstruction metadata to S3."""
        try:
            client = self._get_client()
            metadata_json = json.dumps(metadata, indent=2)
            client.put_object(
                Bucket=bucket_name,
                Key=s3_key,
                Body=metadata_json,
                ContentType='application/json'
            )
            print(f"Saved metadata: {s3_key}")
            return True
        except Exception as e:
            print(f"Error saving metadata: {e}")
            return False

def create_s3_manager(aws_access_key_id: str = None, aws_secret_access_key: str = None, region_name: str = 'us-east-1', access_point_arn: str = None) -> S3Manager:
    """Create S3Manager instance with credentials and optional access point."""
    return S3Manager(aws_access_key_id, aws_secret_access_key, region_name, access_point_arn)

def get_bucket_images_gradio(bucket_name: str, prefix: str = "") -> Tuple[List[str], str]:
    """Gradio-compatible function to get images from S3 bucket."""
    if not bucket_name:
        return [], "Please select a bucket"
    
    try:
        s3_manager = create_s3_manager()
        images = s3_manager.list_images_in_bucket(bucket_name, prefix)
        
        if not images:
            return [], f"No images found in bucket '{bucket_name}' with prefix '{prefix}'"
        
        return images, f"Found {len(images)} images in bucket '{bucket_name}'"
    except Exception as e:
        return [], f"Error accessing bucket: {str(e)}"

def download_and_process_s3_images(bucket_name: str, image_keys: List[str], target_dir: str) -> Tuple[str, str]:
    """Download images from S3 and prepare for VGGT processing."""
    if not bucket_name or not image_keys:
        return "None", "No bucket or images selected"
    
    try:
        # Create images directory
        images_dir = os.path.join(target_dir, "images")
        os.makedirs(images_dir, exist_ok=True)
        
        # Download images
        s3_manager = create_s3_manager()
        downloaded_files = s3_manager.download_images_from_s3(bucket_name, image_keys, images_dir)
        
        if not downloaded_files:
            return "None", "Failed to download images from S3"
        
        return target_dir, f"Downloaded {len(downloaded_files)} images from S3 bucket '{bucket_name}'"
    
    except Exception as e:
        return "None", f"Error processing S3 images: {str(e)}"

def upload_glb_to_s3_gradio(bucket_name: str, glb_file_path: str, scene_name: str = None) -> str:
    """Upload GLB file to S3 with automatic naming."""
    if not bucket_name or not glb_file_path or not os.path.exists(glb_file_path):
        return "Error: Invalid bucket, file path, or file not found"
    
    try:
        # Generate S3 key
        if not scene_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            scene_name = f"vggt_reconstruction_{timestamp}"
        
        s3_key = f"reconstructions/{scene_name}.glb"
        
        # Upload to S3
        s3_manager = create_s3_manager()
        success = s3_manager.upload_glb_to_s3(bucket_name, glb_file_path, s3_key)
        
        if success:
            return f"Successfully uploaded GLB to s3://{bucket_name}/{s3_key}"
        else:
            return "Failed to upload GLB file to S3"
    
    except Exception as e:
        return f"Error uploading GLB: {str(e)}"

def list_available_glb_files(bucket_name: str) -> Tuple[List[str], str]:
    """List available GLB files in S3 bucket."""
    if not bucket_name:
        return [], "Please select a bucket"
    
    try:
        s3_manager = create_s3_manager()
        glb_files = s3_manager.list_glb_files_in_bucket(bucket_name, "reconstructions/")
        
        if not glb_files:
            return [], f"No GLB files found in bucket '{bucket_name}'"
        
        # Return just the keys for Gradio dropdown
        keys = [f["key"] for f in glb_files]
        return keys, f"Found {len(keys)} GLB files in bucket '{bucket_name}'"
    
    except Exception as e:
        return [], f"Error listing GLB files: {str(e)}"
