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
how to find the filename and name for the plugin ??? 