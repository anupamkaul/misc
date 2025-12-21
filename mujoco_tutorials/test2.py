'''
On rendering of a worldbody model
'''

import mujoco
import time
import itertools
import numpy as np

# graphics and plotting
import mediapy as media
import matplotlib.pyplot as plt

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

from PIL import Image

#make renderer, render and show pixels
with mujoco.Renderer(model) as renderer:

    # need to propagate values in mjData (using mjForward)
    # in order to render something more than black pixels
    mujoco.mj_forward(model, data)
    renderer.update_scene(data)

    media.show_image(renderer.render()) # works with ipynb

    # dump pixels to image file
    pixels = renderer.render()
    image = Image.fromarray(np.flipud(pixels))

    output_path = "mujoco_render.png"
    image.save(output_path)

    print(f"Rendered image saved to {output_path}")
