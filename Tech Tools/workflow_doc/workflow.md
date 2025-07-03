
## Plugins

- Update plugins in the world file  
- Update the sensor in the link  
- Update the launch file to bridge the connection
```bash
# Important reference sources:
# https://gazebosim.org/docs/fortress/sensors/#lidar-sensor
# http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_imu
```

## Concept
RViz2 is basically a ROS 2 node that acts as a visualization client. Once you launch it, it:

- Subscribes to any topics you tell it to (like `/lidar`, `/camera/image_raw`, `/tf`, etc.)
- Reads the incoming messages from those topics
- Displays the data graphically in real-time, like laser scans, point clouds, images, robot models, and more.
