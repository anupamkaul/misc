import mujoco
import numpy as np
import imageio.v2 as imageio # Use imageio-v2 for compatibility if needed (pip install imageio) 
import os

'''
Here I start playing with actuators.. or, I introduce
actuator perturbations, instead of external keyframe
perturbations

(Tests 5 and 6, apart from modifying humanoids, were 
about initial and continuous perturbations caused by
different keyframes)

'''

os.environ['MUJOCO_GL'] = 'egl' 

# Define the path to your MuJoCo XML model file
#MODEL_PATH = './models/humanoid-spin.xml' 
MODEL_PATH = './models/humanoid-push.xml' 

# Simulation parameters
DURATION = 10          # seconds
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

# let's find some actuator IDs based on actuator names..
print("number of actuators in this model: ", model.nu)

'''
Search for these:

  <actuator>
    <motor name="abdomen_y"       gear="40"  joint="abdomen_y"/>
    <motor name="abdomen_z"       gear="40"  joint="abdomen_z"/>
    <motor name="abdomen_x"       gear="40"  joint="abdomen_x"/>
    <motor name="right_hip_x"     gear="40"  joint="right_hip_x"/>
    <motor name="right_hip_z"     gear="40"  joint="right_hip_z"/>
    <motor name="right_hip_y"     gear="120" joint="right_hip_y"/>
    <motor name="right_knee"      gear="80"  joint="right_knee"/>
    <motor name="right_ankle_x"   gear="20"  joint="right_ankle_x"/>
    <motor name="right_ankle_y"   gear="20"  joint="right_ankle_y"/>
    <motor name="left_hip_x"      gear="40"  joint="left_hip_x"/>
    <motor name="left_hip_z"      gear="40"  joint="left_hip_z"/>
    <motor name="left_hip_y"      gear="120" joint="left_hip_y"/>
    <motor name="left_knee"       gear="80"  joint="left_knee"/>
    <motor name="left_ankle_x"    gear="20"  joint="left_ankle_x"/>
    <motor name="left_ankle_y"    gear="20"  joint="left_ankle_y"/>
    <motor name="right_shoulder1" gear="20"  joint="right_shoulder1"/>
    <motor name="right_shoulder2" gear="20"  joint="right_shoulder2"/>
    <motor name="right_elbow"     gear="40"  joint="right_elbow"/>
    <motor name="left_shoulder1"  gear="20"  joint="left_shoulder1"/>
    <motor name="left_shoulder2"  gear="20"  joint="left_shoulder2"/>
    <motor name="left_elbow"      gear="40"  joint="left_elbow"/>
  </actuator>

'''

right_knee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_knee")
print("right knee actuator id: ", right_knee_id)

left_knee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_knee")
print("left knee actuator id: ", left_knee_id)

right_ankle_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_ankle_x")
print("right ankle_x actuator id: ", right_ankle_x_id)

right_ankle_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_ankle_y")
print("right ankle_y actuator id: ", right_ankle_y_id)

right_elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_elbow")
print("right elbow actuator id: ", right_elbow_id)

left_elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_elbow")
print("left elbow actuator id: ", left_elbow_id)

# by default if freebody is disabled in humanoid-push.xml, the humanoid will stand on the floor,
# gently swinging its arms.. (thus non-langragian and a subtractive approach applies constraints)

actuator_input_val = 0.5
actuator_input_step = 0.5
actuator_max_val = 3
sign = -1

reset_count=0
while data.time < DURATION:

    # mj_step advances the physics simulation
    mujoco.mj_step(model, data) 
    reset_count += 1

    # in this loop I know data.time advances to 2000 steps
    # just (grep|wc) a print(data.time), so let me introduce
    # actuator perturbations roughly 8 times in the simulation: 

    if (reset_count % 25) == 0:

        # 1. store current time
        current_time = data.time
 
        # 2. provide inputs to some actuators of the humanoid

        print("actuator val for right knee: ", data.ctrl[right_knee_id])
        print("actuator val for left knee: ", data.ctrl[left_knee_id])
        print("actuator val for right ankle x: ", data.ctrl[right_ankle_x_id])
        print("actuator val for right ankle y: ", data.ctrl[right_ankle_y_id])
        print("actuator val for right elbow: ", data.ctrl[right_elbow_id])
        print("actuator val for left elbow: ",  data.ctrl[left_elbow_id])

        data.ctrl[right_knee_id] = sign*actuator_input_val
        data.ctrl[left_knee_id] =  actuator_input_val
        data.ctrl[right_ankle_x_id] = actuator_input_val
        data.ctrl[right_ankle_y_id] = actuator_input_val
        data.ctrl[right_elbow_id] = actuator_input_val
        data.ctrl[left_elbow_id] = actuator_input_val

        if (abs(actuator_input_val) > abs(actuator_max_val)) :
            actuator_input_step *= sign  

        actuator_input_val += actuator_input_step

        # 3. restore current time
        data.time = current_time

        # 4. ensure synchronicity (update all dependent data fields)
        mujoco.mj_forward(model, data)

    # Render the current frame
    #renderer.update_scene(data, camera=camera_name)

    #renderer.update_scene(data)
    #renderer.update_scene(data, camera="egocentric")
    #renderer.update_scene(data, camera="back")
    renderer.update_scene(data, camera="side")

    pixels = renderer.render()
    frames.append(pixels)

# Save frames to a file using imageio
output_video_path = 'test7.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=FRAMERATE) 

print(f"Simulation finished. Video saved to {output_video_path}")

# read some data
print("positions: (len): ", len(data.qpos), "\n", data.qpos)
print("velocities:(len): ", len(data.qvel), "\n", data.qvel) 
print("actuators:(len): ", len(data.ctrl), "\n", data.ctrl) 

'''

28,27 are for freebody (freejoint)
else it is 21,21

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
