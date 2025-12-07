---
sidebar_label: 'Week 3: ROS 2 Fundamentals'
sidebar_position: 1
estimated_time: '4-5 hours'
week: 3
module: 1
prerequisites:
  - Python intermediate level
  - Basic Linux command line
learning_objectives:
  - Understand ROS 2 architecture and design principles
  - Create and run ROS 2 nodes
  - Work with topics, services, and messages
---

# Week 3: ROS 2 Fundamentals

## Introduction to ROS 2

**ROS 2** (Robot Operating System 2) is a middleware framework for robot software development. Unlike ROS 1, ROS 2 is:

- **Real-time capable**: DDS middleware supports deterministic communication
- **Production-ready**: Used in commercial robots (Boston Dynamics, etc.)
- **Security-focused**: Built-in authentication and encryption
- **Multi-platform**: Linux, Windows, macOS, RTOS

## Core Concepts

### 1. Nodes

A **node** is a single-purpose executable that performs computation.

**Example**: A camera driver node publishes images

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 2. Topics

**Topics** enable publish-subscribe communication between nodes.

**Key properties:**
- **Asynchronous**: Publisher doesn't wait for subscribers
- **Many-to-many**: Multiple publishers and subscribers allowed
- **Typed**: Messages have defined schemas

```bash
# List all active topics
ros2 topic list

# Show message type
ros2 topic info /topic

# Echo messages
ros2 topic echo /topic
```

### 3. Services

**Services** provide synchronous request-response communication.

**Use cases:**
- Trigger actions (start/stop motors)
- Query state (get battery level)
- Configure parameters

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

## ROS 2 Workspace Setup

### Installation (Ubuntu 22.04)

```bash
# Add ROS 2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Source setup file
source /opt/ros/humble/setup.bash
```

### Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

## Hands-On Exercise: Build a Talker-Listener System

### Goal
Create two nodes: one publishes messages, another subscribes and prints them.

### Implementation

**Publisher (talker.py)**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        
    def publish_message(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main():
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
```

**Subscriber (listener.py)**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
```

## ROS 2 CLI Tools

| Command | Purpose |
|---------|---------|
| `ros2 node list` | List active nodes |
| `ros2 topic list` | List active topics |
| `ros2 service list` | List available services |
| `ros2 run <pkg> <node>` | Run a node |
| `ros2 pkg create` | Create new package |
| `ros2 bag record` | Record topic data |

## Key Takeaways

1. **Nodes are independent processes** that communicate via topics/services
2. **Topics** are for streaming data (sensor readings, commands)
3. **Services** are for request-response patterns (queries, triggers)
4. **ROS 2 is language-agnostic** - Python and C++ are most common

## Next Steps

Continue to [Week 4: Communication Patterns](./week-4-communication.md) to learn about publishers, subscribers, and actions in depth.
