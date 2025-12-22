import mujoco
import numpy as np
import imageio.v2 as imageio # Use imageio-v2 for compatibility if needed (pip install imageio) 
import os

# Set the desired backend for headless rendering
# Use 'egl' for hardware acceleration (recommended with GPU access)
# or 'osmesa' for software rendering
os.environ['MUJOCO_GL'] = 'egl' 

# Define the path to your MuJoCo XML model file
MODEL_PATH = './models/humanoid-spin.xml' 

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
mujoco.mj_resetDataKeyframe(model, data, 0)  # Reset the state to keyframe 0
while data.time < DURATION:
    # mj_step advances the physics simulation
    mujoco.mj_step(model, data) 

    # Render the current frame

    #renderer.update_scene(data, camera=camera_name)

    #renderer.update_scene(data)
    #renderer.update_scene(data, camera="egocentric")
    #renderer.update_scene(data, camera="back")
    renderer.update_scene(data, camera="side")

    pixels = renderer.render()
    frames.append(pixels)

# Save frames to a file using imageio
output_video_path = 'test6.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=FRAMERATE) 

print(f"Simulation finished. Video saved to {output_video_path}")

# read some data
print("positions: (len): ", len(data.qpos), "\n", data.qpos)
print("velocities:(len): ", len(data.qvel), "\n", data.qvel) 

'''
positions: (len):  28 
 [ 7.97647593e-01 -2.47586575e-13  2.78628690e-01  4.15656584e-02
 -6.93402184e-14  9.99135775e-01  4.56234143e-13 -3.50380234e-13
 -1.31750353e+00  1.98156061e-15  9.36636398e-03 -3.53294890e-03
 -7.42463958e-01  4.28900397e-02 -8.81513924e-01  2.94688320e-04
  9.36636398e-03 -3.53294890e-03 -7.42463958e-01  4.28900397e-02
 -8.81513924e-01 -2.94688323e-04 -4.64790363e-01  6.11471875e-01
 -1.16884479e+00  4.64790363e-01 -6.11471875e-01 -1.16884479e+00]
velocities:(len):  27 
 [ 5.22773474e-03 -6.91687420e-13 -7.15365132e-04 -2.55015081e-12
 -4.57032905e-02 -2.79059064e-13 -9.53561692e-13  5.32210966e-05
  2.81653846e-14 -1.94478104e-03  1.54415505e-03  7.45346085e-02
 -5.52499372e-05 -1.19511364e-05 -2.52012340e-03 -1.94478105e-03
  1.54415505e-03  7.45346085e-02 -5.52499372e-05 -1.19511364e-05
  2.52012339e-03  8.98457917e-02  2.75320960e-02 -8.47427923e-02
 -8.98457917e-02 -2.75320960e-02 -8.47427924e-02]

'''
