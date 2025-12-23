import mujoco
import numpy as np
import imageio.v2 as imageio # Use imageio-v2 for compatibility if needed (pip install imageio) 
import os

'''
Here I use freejoint with 6DOF (torse root w.r.t worldbody)
to simulate falling and attempt to write control functions
to prevent the robot from falling

Once I succeed in a basic control loop that the humanoid uses
to prevent itself from falling under gravity, I can check the 
robustness of the solution by putting in perturbations from 
test6.py (keyframes) and test7.py (actuator motion/poses)

'''

os.environ['MUJOCO_GL'] = 'egl' 

# Define the path to your MuJoCo XML model file
MODEL_PATH = './models/humanoid-push-freejoint.xml' 

# Simulation parameters
DURATION = 10          # seconds
FRAMERATE = 60        # frames per second
TIMESTEP = 0.002      # MuJoCo timestep
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

# set a target pose (target joint positions)
# this should be the standing or the half-sitting pose

target_qpos = np.array([0, 0, 1.2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# set initial position
data.qpos = target_qpos.copy()

# update physics with initial state
mujoco.mj_forward(model, data)

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

        '''
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
        '''

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
output_video_path = 'test8.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=FRAMERATE) 

print(f"Simulation finished. Video saved to {output_video_path}")

# read some data
print("positions: (len): ", len(data.qpos), "\n", data.qpos)
print("velocities:(len): ", len(data.qvel), "\n", data.qvel) 
print("actuators:(len): ", len(data.ctrl), "\n", data.ctrl) 

