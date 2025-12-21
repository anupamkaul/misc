import mujoco
import numpy as np
from PIL import Image
import os

# Set environment variable for offscreen rendering backend (EGL is recommended for GPU acceleration)
# If EGL is not available, use OSMesa (software rendering)
# You can set this in your terminal before running the script: export MESA_GL_VERSION_OVERRIDE=4.6
# Or uncomment the line below (setting env vars in script might not work in all setups)
# os.environ['MESA_GL_VERSION_OVERRIDE'] = '4.6'

# 1. Define the MJCF model XML as a string (or load from a file)
xml_string = """
<mujoco model="simple_capsule">
    <worldbody>
        <light pos="0 0 1"/>
        <geom type="capsule" size=".1" fromto="0 0 0 0 0 0.5" rgba="1 0 0 1"/>
        <geom type="plane" size="1 1 0.1" rgba="0 1 0 1"/>
    </worldbody>
</mujoco>
"""

# 2. Load the model and data
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# 3. Setup the renderer
# Use mujoco.GLContext for offscreen rendering
#width, height = 640, 480
width, height = 480, 480
renderer = mujoco.Renderer(model, width, height)

# 4. Simulate and render a frame
mujoco.mj_step(model, data)  # Step the simulation forward (optional, but shows an active state)

# Update the scene and render the image
renderer.update_scene(data)
# The render method returns a numpy array
pixels = renderer.render()

# 5. Save the image to a file
# The 'pixels' array is upside down (OpenGL convention), so flip it
image = Image.fromarray(np.flipud(pixels))
output_path = "sample_mujoco_render.png"
image.save(output_path)

print(f"Rendered image saved to {output_path}")

# Note: The Renderer class handles the GLContext creation and management internally
