[Material source]()

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

## Publisher and Subscriber
<details>
<summary> Publisher </summary>

```bash
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
</details>

<details>
<summary> Subscriber </summary>

```bash

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

update entry_point from setup.py

```bash
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_pubsub
source install/setup.bash
```
run
```bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

## Service and Client
<details>
<summary> Service </summary>

```bash
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
</details>

<details>
<summary> Client </summary>

```bash
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
</details>

