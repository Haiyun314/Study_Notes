# ROS2 Concepts

###
<img src="./images/Screenshot%202025-05-07%20at%2017.39.37.png" alt="ROS2 Concepts" width="400"/>

## Nodes
A **node** is a fundamental unit in ROS 2 that performs computation and interacts with other nodes.

## Communication Mechanisms
**Topics**, **Services**, and **Actions** are communication mechanisms used by nodes to exchange data:

- **Topic**: A mechanism for unidirectional, streaming, pub-sub communication (e.g., sensor data).
- **Service**: A synchronous request/response communication (like a function call).
- **Action**: A communication mechanism for asynchronous, long-running tasks with feedback (e.g., robot navigation).
<img src='./images/Screenshot 2025-05-07 at 17.40.22.png' width='300'/>

# Command 
## Create a package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
```
## Build a package
```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/local_setup.bash
ros2 run my_package my_node
```
when there are multiple nodes, just update the entry_point in setup.py
```bash 
entry_points={
    'console_scripts': [
        'my_node = my_package.my_node:main',
        'another_node = my_package.another_node:main',
        'third_node = my_package.third_node:main',
    ],
},
```

