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

xml = """
<mujoco>
    <worldbody>
        <light name="top" pos="0 0 1"/>
        <geom name="red box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
        <geom name="green sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
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

