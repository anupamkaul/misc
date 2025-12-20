# test that mujoco is installed and all is well

import mujoco

# from_xml_string invokes the model compiler and creates 
# a binary model instance
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

# let's start by making (defining and loading) a simple model

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
@Notes 
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

# https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h (near end of file): 
print("number of geometries in the scene: ", model.ngeom) 

# this hack lets you inspect all the geometries if you didn't know them (via e)
try:
    model.geom()
except KeyError as e:
    print("Error: ", e)

'''
Output: 
<mujoco._structs.MjModel object at 0x73bc77edddf0>
number of geometries in the scene:  2
Error:  "Invalid name ''. Valid names: ['green_sphere', 'red_box']"
'''

'''
@Notes

## mjModel

MuJoCo's `mjModel`, contains the *model description*, i.e., all quantities which *do not change over time*. 
The complete description of `mjModel` can be found at the end of the header file 
[`mjmodel.h`](https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h). 
Note that the header files contain short, useful inline comments, describing each field.

Examples of quantities that can be found in `mjModel` are `ngeom`, the number of geoms in the scene and `geom_rgba`, 
their respective colors:
'''

print("geometry of green sphere: ", model.geom('green_sphere'))
print("geometry of red box: ", model.geom('red_box'))

'''
Output:
geometry of green sphere:  <_MjModelGeomViews
  bodyid: array([0], dtype=int32)
  conaffinity: array([1], dtype=int32)
  condim: array([3], dtype=int32)
  contype: array([1], dtype=int32)
  dataid: array([-1], dtype=int32)
  friction: array([1.   , 0.005, 0.   ])
  gap: array([0.])
  group: array([0], dtype=int32)
  id: 1
  margin: array([0.])
  matid: array([-1], dtype=int32)
  name: 'green_sphere'
  pos: array([0.2, 0.2, 0.2])
  priority: array([0], dtype=int32)
  quat: array([1., 0., 0., 0.])
  rbound: array([0.1])
  rgba: array([0., 1., 0., 1.], dtype=float32)
  sameframe: array([3], dtype=uint8)
  size: array([0.1, 0. , 0. ])
  solimp: array([0.9  , 0.95 , 0.001, 0.5  , 2.   ])
  solmix: array([1.])
  solref: array([0.02, 1.  ])
  type: array([2], dtype=int32)
  user: array([], dtype=float64)

'''

print("rgba of green_sphere: ", model.geom('green_sphere').rgba)

'''
Output:
rgba of green_sphere:  [0. 1. 0. 1.]
'''

# query some other params, as defined in https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h
print("Number of DOF: ", model.nv)
print("Number of Bodies: ", model.nbody)
print("Number of Geoms: ",  model.ngeom)
print("Number of Joints: ", model.njnt)

# Use the mujoco documentation effectively:
# API doc access info is here: https://mujoco.readthedocs.io/en/latest/APIreference/index.html

# e.g. https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html#mjtobj
# e.g. https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#mj-name2id

# some conveniences (mjtObt) to convert between object names and integer IDs
id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'green_sphere')
print(model.geom_rgba[id, :])

print('id of "green sphere": ', model.geom('green_sphere').id)
print('name of geom 0: ', model.geom(0).name)
print('name of geom 1: ', model.geom(1).name)
print('name of body 0: ', model.body(0).name)

# the next big struct is mjdata : this holds states, forces, motion, time etc (almost all of the dynamics)
# https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h





