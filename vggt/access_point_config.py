# S3 Access Point Configuration for VGGT
# This file helps manage S3 Access Points for secure VGGT operations

import os
from typing import Optional, Dict, Any

class AccessPointConfig:
    def __init__(self):
        self.access_point_arn = os.getenv('VGGT_ACCESS_POINT_ARN')
        self.bucket_name = os.getenv('VGGT_BUCKET_NAME')
        self.region = os.getenv('AWS_DEFAULT_REGION', 'us-east-1')
    
    def get_access_point_arn(self) -> Optional[str]:
        """Get the access point ARN."""
        return self.access_point_arn
    
    def get_bucket_name(self) -> Optional[str]:
        """Get the bucket name."""
        return self.bucket_name
    
    def get_region(self) -> str:
        """Get the AWS region."""
        return self.region
    
    def set_access_point(self, access_point_arn: str, bucket_name: str, region: str = 'us-east-1'):
        """Set access point configuration."""
        self.access_point_arn = access_point_arn
        self.bucket_name = bucket_name
        self.region = region
    
    def has_access_point(self) -> bool:
        """Check if access point is configured."""
        return bool(self.access_point_arn and self.bucket_name)
    
    def get_endpoint_url(self) -> Optional[str]:
        """Get the access point endpoint URL."""
        if not self.access_point_arn:
            return None
        
        # Extract access point ID from ARN
        access_point_id = self.access_point_arn.split('/')[-1]
        return f"https://{access_point_id}.s3-accesspoint.{self.region}.amazonaws.com"
    
    def get_config_dict(self) -> Dict[str, Any]:
        """Get configuration as dictionary."""
        return {
            'access_point_arn': self.access_point_arn,
            'bucket_name': self.bucket_name,
            'region': self.region,
            'endpoint_url': self.get_endpoint_url()
        }

# Global access point configuration
access_point_config = AccessPointConfig()

def get_access_point_config() -> AccessPointConfig:
    """Get the global access point configuration."""
    return access_point_config

def setup_access_point(access_point_arn: str, bucket_name: str, region: str = 'us-east-1'):
    """Setup access point configuration."""
    access_point_config.set_access_point(access_point_arn, bucket_name, region)
    return access_point_config.has_access_point()

def get_access_point_arn_from_env() -> Optional[str]:
    """Get access point ARN from environment variables."""
    return os.getenv('VGGT_ACCESS_POINT_ARN')

def get_bucket_name_from_env() -> Optional[str]:
    """Get bucket name from environment variables."""
    return os.getenv('VGGT_BUCKET_NAME')

# Example access point ARN format:
# arn:aws:s3:us-east-1:123456789012:accesspoint/vggt-demo-access-point
