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

# find id of the pelvis joint, this is what I will
# use to figure out the humanoid falling state
# TODO

def check_fall_condition(data):

    '''
    Checks if the humanoid has fallen over (if height is below a threshold)

    '''

    height = data.qpos[2]
    print("height: ", height)

    if (height < 1.0):
    #if (height < 0.6):
        return True

    return False

# PD control gains (proportional and derivative)
# These are crucial for stability and need tuning

Kp = 50.0 # Proportional gain
Kd = 5.0  # Derivative gain

def apply_balance_control(model, data, target_qpos, Kp, Kd):
    """
    Applies PD control to joints to maintain a target posture and prevent falling.
    """
    # Get current joint positions and velocities
    current_qpos = data.qpos[7:] # Exclude the first 7 DOFs (free joint: pos + quat)
    current_qvel = data.qvel[6:] # Exclude the first 6 DOFs (free joint: vel + ang_vel)

    # Calculate position error
    error = target_qpos[7:] - current_qpos

    # Calculate desired control torques
    # tau = Kp * error - Kd * current_qvel (simple PD controller)
    # The control input is applied to the actuators (data.ctrl)
    data.ctrl[:] = Kp * error - Kd * current_qvel

reset_count=0
while data.time < DURATION:

    # mj_step advances the physics simulation
    mujoco.mj_step(model, data) 
    reset_count += 1

    if check_fall_condition(data):
        print("Humanoid is falling !!")

        # 1. store current time
        current_time = data.time
        
        # 2. rudimentary: reset target pose
        #data.qpos = target_qpos.copy()
        apply_balance_control(model, data, target_qpos, Kp, Kd)

        # 3. restore current time
        data.time = current_time

        # 4. resync
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

