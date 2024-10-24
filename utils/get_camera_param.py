import pyrealsense2 as rs
import os

# Create a pipeline
pipeline = rs.pipeline()
config = rs.config()

# Start the pipeline
pipeline_profile = pipeline.start(config)

# Retrieve the device from the pipeline
device = pipeline_profile.get_device()
import pdb; pdb.set_trace()

# Get advanced device to access controls
advanced_mode = rs.rs400_advanced_mode(device)

# Serialize the current settings to JSON
json_settings = advanced_mode.serialize_json()

# Save the JSON file
save_dir = '/home/robopil/baoyu/trackdlo_ws/src/trackdlo/config'
save_path = os.path.join(save_dir, 'realsense_d455.json')
with open(save_path, 'w') as f:
    f.write(json_settings)

# Stop the pipeline
pipeline.stop()
