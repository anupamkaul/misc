'''
On simulation (video) of a worldbody model
'''

import mujoco
import time
import itertools
import numpy as np

# graphics and plotting
import mediapy as media
import matplotlib.pyplot as plt
import imageio.v2 as imageio  

'''
cause motion by adding DOFs. The things that move (and have intertia) are
called bodies. We add DOF by adding joints to bodies, specifying how they
can move with respect to their parents. Will make a new body that contains
our geoms, add a hinge joint and re-render, while visualizing the joint axis
using the visualization option object MjvOption

basically encapsulate the previous xml inside a body and add a hinge joint for
motion
'''

xml = """
<mujoco>
    <worldbody>
        <light name="top" pos="0 0 1"/>
        <body name="box_and_sphere" euler="0 0 -30">
            <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
            <geom name="red box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
            <geom name="green sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
        </body>
    </worldbody>
</mujoco>
"""

#make model and data
model = mujoco.MjModel.from_xml_string(xml)
data  = mujoco.MjData(model)

# simulate and display video
duration  = 3.8 #seconds
framerate = 60

frames = []
mujoco.mj_resetData(model, data) #reset state and time

with mujoco.Renderer(model) as renderer:
    while data.time < duration:
        mujoco.mj_step(model, data)

        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            pixels = renderer.render()
            frames.append(pixels)

    #media.show_video(frames, fps=framerate) # for ipython

# Save frames to a file using imageio
output_video_path = 'test3_output.mp4'
# Note: imageio might require ffmpeg as a backend (pip install imageio[ffmpeg]) 
imageio.mimsave(output_video_path, frames, fps=framerate) 

print(f"Simulation finished. Video saved to {output_video_path}")

