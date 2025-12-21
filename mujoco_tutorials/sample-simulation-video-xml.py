import mujoco
import numpy as np
import imageio.v2 as imageio # Use imageio-v2 for compatibility if needed (pip install imageio) 
import os

# Set the desired backend for headless rendering
# Use 'egl' for hardware acceleration (recommended with GPU access)
# or 'osmesa' for software rendering
os.environ['MUJOCO_GL'] = 'egl' 

# Define the path to your MuJoCo XML model file
MODEL_PATH = './simple-model.xml' # Replace with your actual path

# Simulation parameters
DURATION = 3          # seconds
FRAMERATE = 60        # frames per second
TIMESTEP = 0.002      # MuJoCo timestep
#WIDTH, HEIGHT = 640, 480
WIDTH, HEIGHT = 480, 480

# Load the model and data
try:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# Initialize the renderer
renderer = mujoco.Renderer(model, WIDTH, HEIGHT)
camera_name = None # Use default camera or specify a name from your XML

frames = []

# Run the simulation
while data.time < DURATION:
    # mj_step advances the physics simulation
    mujoco.mj_step(model, data) 

    # Render the current frame
    #renderer.update_scene(data, camera=camera_name)
    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)

# Save frames to a file using imageio
output_video_path = 'simulation_output.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=FRAMERATE) 

print(f"Simulation finished. Video saved to {output_video_path}")
