'''
Example: Simulate free-bodies with the self inverting tippe-top

Notables:

1. A 6-DoF free joint is added with the <freejoint/> clause.

2. We use the <option/> clause to set the integrator to the 4th order Runge Kutta. 
   Runge-Kutta has a higher rate of convergence than the default Euler integrator, 
   which in many cases increases the accuracy at a given timestep size.

3. We define the floor's grid material inside the <asset/> clause and reference it 
   in the "floor" geom. (Introduce <texture> and <material> under <asset>) 

4. We use an invisible and non-colliding box geom called ballast to move the top's 
   center-of-mass lower. Having a low center of mass is (counter-intuitively) required 
   for the flipping behavior to occur.

5. We save our initial spinning state as a keyframe. It has a high rotational velocity 
   around the Z-axis, but is not perfectly oriented with the world, which introduces 
   the symmetry-breaking required for the flipping.

6. We define a <camera> in our model, and then render from it using the camera argument 
   to update_scene().
'''

import mujoco
import time
import itertools
import numpy as np

# graphics and plotting
import mediapy as media
import matplotlib.pyplot as plt
import imageio.v2 as imageio


tippe_top = """
<mujoco model="my tippe top">
    <option integrator="RK4"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
         rgb2=".2 .3 .4" width="300" height="300"/> 
        <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
    </asset>

    <worldbody>
        <geom size=".2 .2 .01" type="plane" material="grid"/>
        <light pos="0 0 .6"/>
        <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>
        <body name="top" pos="0 0 .02">
          <freejoint/>
          <geom name="ball" type="sphere" size=".02" />
          <geom name="stem" type="cylinder" pos="0 0 .02" size="0.004 .008"/>
          <geom name="ballast" type="box" size=".023 .023 0.005"  pos="0 0 -.015"
           contype="0" conaffinity="0" group="3"/>
        </body>
    </worldbody>

  <keyframe>
    <key name="spinning" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 200" />
  </keyframe>
</mujoco>
"""
# render this:
from PIL import Image

model = mujoco.MjModel.from_xml_string(tippe_top)
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)
with mujoco.Renderer(model) as renderer:
    renderer.update_scene(data, camera="closeup")

    media.show_image(renderer.render())

    # dump pixels to image file
    pixels = renderer.render()
    image = Image.fromarray(np.flipud(pixels))

    output_path = "test4.png"
    image.save(output_path)

    print(f"Rendered image saved to {output_path}")

# let's simulate (create a video)

duration = 7    # (seconds)
framerate = 60  # (Hz)

# Simulate and display video.
frames = []
mujoco.mj_resetDataKeyframe(model, data, 0)  # Reset the state to keyframe 0
with mujoco.Renderer(model) as renderer:
  while data.time < duration:
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
      renderer.update_scene(data, "closeup")
      pixels = renderer.render()
      frames.append(pixels)

# Save frames to a file using imageio
output_video_path = 'test4_output.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=framerate) 

print(f"Simulation finished. Video saved to {output_video_path}")

# let's get some data..
print('positions: ', data.qpos)
print('velocities:', data.qvel)

'''
positions:  [ 0.02332768 -0.02439969  0.02792166  0.07778017  0.86488317 -0.4959039
 -0.00258259]
velocities: [ 6.83240423e-02 -8.69515134e-03 -2.56495395e-04  3.65873967e+00
  9.43958903e+00 -8.12162364e+01]

Post execution analysis
-----------------------

Notes: 

A free body is a body with a free joint having 6 DOFs: 3 translations and 3 rotations.
(<freejoint/>) 

The 6 values of velocity is easy to explain : it is one for each DOF.
There are 7 values for position: the first 3 are the .02. .02, .02 as defined for the body (assuming
it didn't move much. The last 4 are the unit quaternion for the 3D orientation. 3D orientations are
represented by 4 numbers while angular velocities are 3 numbers. 

See https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
(or refreshers/Quaternion*.pdf) 

(Compared to rotation matrices, quaternions are more compact, efficient, and numerically stable. 
Compared to Euler angles, they are simpler to compose. However, they are not as intuitive and easy 
to understand and, due to the periodic nature of sine and cosine, rotation angles differing precisely 
by the natural period will be encoded into identical quaternions and recovered angles in radians will 
be limited to [0, 2pi] 

Notes (of test3.py):

In the real world, all rigid objects have 6 degrees-of-freedom: 3 translations and 3 rotations. 
Real-world joints act as constraints, removing relative degrees-of-freedom from bodies connected 
by joints. Some physics simulation software use this representation which is known as the 
"Cartesian" or "subtractive" representation, but it is inefficient. 

MuJoCo uses a representation known as the "Lagrangian", "generalized" or "additive" representation, 
whereby objects have no degrees of freedom unless explicitly added using joints.

'''

'''
Now, lets measure values held in mjData:

The mjData struct contains the dynamic variables and intermediate results produced by the simulation
(such as the one above) which are _expected to change_ on each timestep. Below, we simulate for 2000
timesteps and plot the angular velocity of the top and height of the stem as a function of time.
'''

print("re-run simulation to plot saved intermediate data from mj_data..")

timevals = []
angular_velocity = []
stem_height = []

# simulate (again) and save data
mujoco.mj_resetDataKeyframe(model, data, 0)
while data.time < duration:

    mujoco.mj_step(model, data)

    timevals.append(data.time)
    angular_velocity.append(data.qvel[3:6].copy())
    stem_height.append(data.geom_xpos[2,2]);

dpi = 120
width = 600
height = 800
figsize = (width / dpi, height / dpi)
_, ax = plt.subplots(2, 1, figsize=figsize, dpi=dpi, sharex=True)

ax[0].plot(timevals, angular_velocity)
ax[0].set_title('angular velocity')
ax[0].set_ylabel('radians / second')

ax[1].plot(timevals, stem_height)
ax[1].set_xlabel('time (seconds)')
ax[1].set_ylabel('meters')
_ = ax[1].set_title('stem height')

plt.show()
