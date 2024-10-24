import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline without a specific configuration (uses default camera settings)
pipeline_profile = pipeline.start()

# Get the active device profile
active_profile = pipeline_profile.get_stream(rs.stream.depth)  # Select the depth stream

# Retrieve depth stream resolution and FPS
depth_resolution_width = active_profile.as_video_stream_profile().width()
depth_resolution_height = active_profile.as_video_stream_profile().height()
fps = active_profile.fps()

# Print the depth stream resolution and FPS
print(f"Depth Resolution: {depth_resolution_width}x{depth_resolution_height}")
print(f"Frames Per Second (FPS): {fps}")

# Stop the pipeline
pipeline.stop()
