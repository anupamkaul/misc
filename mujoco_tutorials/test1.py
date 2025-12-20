# test that mujoco is installed and all is well

import mujoco
mujoco.MjModel.from_xml_string('<mujoco/>')

# other imports and helper functions
import time
import itertools
import numpy as np

# graphics and plotting
import mediapy as media
import matplotlib.pyplot as plt

# legible printing from numpy
np.set_printoptions(precision=3, suppress=True, linewidth=100)

# let's start by defining and loading a simple model

xml = """
<mujoco>
    <worldbody>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
print(model)

'''
Notes: 
The xml string is written in MuJoCo's MJCF, which is an XML-based modeling language.

The only required element is <mujoco>.
The smallest valid MJCF model is <mujoco/> which is a completely empty model.

All physical elements live inside the <worldbody> which is always the top-level body 
and constitutes the global origin in Cartesian coordinates.

We define two geoms in the world named red_box and green_sphere.

Question: The red_box has no position, the green_sphere has no type, why is that?

Answer: MJCF attributes have default values. The default position is 0 0 0, the 
default geom type is sphere. The MJCF language is described in the documentation's XML Reference chapter.

The from_xml_string() method invokes the model compiler, which creates a binary mjModel instance.
'''

print("number of geometries in the scene: ", model.ngeom) 

# this hack lets you inspect all the geometries if you didn't know them (via e)
try:
    model.geom()
except KeyError as e:
    print("Error: ", e)

