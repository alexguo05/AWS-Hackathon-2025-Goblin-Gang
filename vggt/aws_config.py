# AWS Configuration for VGGT Demo
# This file helps manage AWS credentials and settings

import os
from typing import Optional

class AWSConfig:
    def __init__(self):
        self.aws_access_key_id = os.getenv('AWS_ACCESS_KEY_ID')
        self.aws_secret_access_key = os.getenv('AWS_SECRET_ACCESS_KEY')
        self.aws_session_token = os.getenv('AWS_SESSION_TOKEN')
        self.region_name = os.getenv('AWS_DEFAULT_REGION', 'us-east-1')
    
    def get_credentials(self) -> dict:
        """Get AWS credentials as a dictionary."""
        creds = {}
        if self.aws_access_key_id:
            creds['aws_access_key_id'] = self.aws_access_key_id
        if self.aws_secret_access_key:
            creds['aws_secret_access_key'] = self.aws_secret_access_key
        if self.aws_session_token:
            creds['aws_session_token'] = self.aws_session_token
        return creds
    
    def set_credentials(self, access_key: str, secret_key: str, region: str = 'us-east-1'):
        """Set AWS credentials programmatically."""
        self.aws_access_key_id = access_key
        self.aws_secret_access_key = secret_key
        self.region_name = region
    
    def has_credentials(self) -> bool:
        """Check if AWS credentials are available."""
        return bool(self.aws_access_key_id and self.aws_secret_access_key)

# Global configuration instance
aws_config = AWSConfig()

def get_aws_config() -> AWSConfig:
    """Get the global AWS configuration."""
    return aws_config

def setup_aws_credentials(access_key: str, secret_key: str, region: str = 'us-east-1'):
    """Setup AWS credentials for the application."""
    aws_config.set_credentials(access_key, secret_key, region)
    return aws_config.has_credentials()
