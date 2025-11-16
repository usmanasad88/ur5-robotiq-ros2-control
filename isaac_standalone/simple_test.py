#!/usr/bin/env python3
"""
Simple test script to verify Isaac Sim installation and UR robot loading.
This is the most minimal example to get started.
"""

from isaacsim import SimulationApp

# Create app
app = SimulationApp({"headless": False})

# Import after app creation
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage

# Create world
world = World()
world.scene.add_default_ground_plane()

# Get Isaac Sim assets path
assets_path = get_assets_root_path()
print(f"Isaac Sim assets: {assets_path}")

# Load UR10
ur10_path = assets_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
print(f"Loading UR10 from: {ur10_path}")
add_reference_to_stage(usd_path=ur10_path, prim_path="/World/UR10")

# Add robot to scene
robot = world.scene.add(Robot(prim_path="/World/UR10", name="my_ur10"))

# Reset world
world.reset()

print("\nUR10 loaded successfully!")
print("Close the window to exit.\n")

# Simple loop
for i in range(500):
    world.step(render=True)

app.close()
