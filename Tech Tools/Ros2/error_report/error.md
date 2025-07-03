Issues discussed here are ROS 2 (Humble ) and Gazebo (Fortress ) based

## Colcon build error

## plugin issues
initialize the plugin in the world file
```bash
    # gps example
    <plugin
      filename="ignition-gazebo-navsat-system"
      name="ignition::gazebo::systems::NavSat">
    </plugin>
```
### How to find the `filename` and `name` for the plugin?

1. **Approach one**: Check the provided examples from the package sources.
2. **Approach two**: Locate `ignition-gazebo-target_sensor-system` in the Gazebo installation directory.
3. **Approach three**: Use `strings` to read symbols from compiled libraries:

    ```bash
    strings /usr/lib/aarch64-linux-gnu/libignition-sensors6-lidar.so | grep Lidar
    ```

    This can reveal names like `ignition::gazebo::systems::Lidar`.
