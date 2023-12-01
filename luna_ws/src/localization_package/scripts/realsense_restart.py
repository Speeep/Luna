import pyrealsense2 as rs
import time

def restart_camera():
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    # Wait for a short duration
    time.sleep(2)

    # Stop streaming
    pipeline.stop()

if __name__ == "__main__":
    restart_camera()