## Core API tutorial

### Create world

```bash
# Launch Isaac Sim before any other imports
from isaacsim import simulationApp
simulation_app = SimulationApp({"headless": False})  # or True for headless mode

# Import necessary modules
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid # DynamicCylinder, StaticCylinder, DynamicCuboid, DynamicSphere. dynamics means it will be affected by gravity, physics. static means it move only when collision with dynamics objects like colision with stone
from isaacsim.core.api import World

# Create two separate world instances
world_a = World()
world_b = World()

# Define a custom world/sample class
class MyWorld(BaseSample):  # inherit from BaseSample
    def __init__(self):
        super().__init__()
    
    def setup_scene(self):
        """
        Set up the scene with a default ground and a dynamic cuboid.
        """
        # Get the world this sample is attached to
        world = self.get_world()
        world.scene.add_default_ground_plane()
        
        # Add a dynamic cuboid to the scene
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/fancy_cube",  # example path
                name="fancy_cube",
                position=(0, 0, 1),
                scale=(0.2, 0.2, 0.2),
                color=(1, 0, 0),  # red cube
            )
        )

        # Reset the world before stepping the simulation
        world.reset()

        # Step the simulation and log cube properties
        for i in range(500):
            position, orientation = fancy_cube.get_world_pose()
            linear_velocity = fancy_cube.get_linear_velocity()
            print(f"Step {i}: Position={position}, Orientation={orientation}, Velocity={linear_velocity}")
            
            # Step the world and render
            world.step(render=True)

        return

    async def setup_post_load(self):
        """
        Post-load setup (asynchronous).

        Short explanation of async:
        - `async def` defines a coroutine that can pause at `await` points.
        - While waiting, other tasks in the simulation can continue running.
        - Useful for loading assets, waiting for events, or performing timed operations.

        Example usage:
        - Access world and objects after loading
        - Get object pose and velocity
        """
        self._world = self.get_world()  # get the world instance
        self._cube = self._world.scene.get_object("fancy_cube")
        
        # Get the cube's world pose and linear velocity
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        print(f"Async post-load: Position={position}, Orientation={orientation}, Velocity={linear_velocity}")
        return

```

### Hello robot

follow up the previous setup scene part, we can add a robot with the help of following module

```bash
from isaacsim.core.api import World
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# Create a World
world = World()
world.scene.add_default_ground_plane()

# Load Jetbot
assets_root = get_assets_root_path()
jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

jetbot = world.scene.add(
    WheeledRobot(
        prim_path="/World/Fancy_Robot",
        name="fancy_robot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_usd,
    )
)

world.reset()

# Step the world with random actions
for i in range(500):
    jetbot.apply_wheel_actions(
        ArticulationAction(joint_velocities=5 * np.random.rand(2,))
    )
    world.step(render=True)


```
