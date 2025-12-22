import mujoco
import numpy as np
import imageio.v2 as imageio # Use imageio-v2 for compatibility if needed (pip install imageio) 
import os

# Set the desired backend for headless rendering
# Use 'egl' for hardware acceleration (recommended with GPU access)
# or 'osmesa' for software rendering
os.environ['MUJOCO_GL'] = 'egl' 

# Define the path to your MuJoCo XML model file
#MODEL_PATH = './models/humanoid.xml' 
MODEL_PATH = './models/humanoid-mod.xml' 

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
output_video_path = 'simulation_humanoid_mod_output.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=FRAMERATE) 

print(f"Simulation finished. Video saved to {output_video_path}")

# read some data
print("positions: ", data.qpos)
print("velocities: ", data.qvel)

'''
21 element tuples returned. Do these correspond to 
21 actuators as defined in the XML?

positions:  [-1.51694072e-18 -9.93104121e-03 -2.31998066e-18 -1.23655526e-02
  4.43190428e-04  9.37606251e-03 -3.70567596e-02  5.17926734e-02
  3.12280701e-02 -1.23655526e-02  4.43190428e-04  9.37606251e-03
 -3.70567596e-02  5.17926734e-02 -3.12280701e-02  6.88007926e-01
 -5.18862221e-01 -1.57633831e+00 -6.88007926e-01  5.18862221e-01
 -1.57633831e+00]
velocities:  [ 4.47188711e-18 -1.19750377e-02 -2.91529275e-17 -8.90043276e-04
  4.85193365e-05 -3.63659222e-03  5.75708195e-03 -2.30275476e-04
 -2.12101260e-04 -8.90043276e-04  4.85193365e-05 -3.63659222e-03
  5.75708195e-03 -2.30275476e-04  2.12101260e-04  5.74229970e-01
 -1.25563848e+00 -1.01865879e-03 -5.74229970e-01  1.25563848e+00
 -1.01865879e-03]

'''
