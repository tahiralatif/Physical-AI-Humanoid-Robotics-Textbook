---
sidebar_label: 'Week 4: Communication Patterns'
sidebar_position: 2
estimated_time: '4-5 hours'
week: 4
module: 1
prerequisites:
  - Week 3 completed
  - ROS 2 workspace set up
learning_objectives:
  - Implement reliable publish-subscribe patterns
  - Create action servers for long-running tasks
  - Handle QoS (Quality of Service) settings
---

# Week 4: ROS 2 Communication Patterns

## Publisher-Subscriber Deep Dive

### QoS (Quality of Service)

ROS 2 uses **DDS (Data Distribution Service)** which provides QoS policies for reliable communication.

**Key QoS Settings:**

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable communication (for critical data)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best effort (for high-frequency sensor data)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

## Actions: For Long-Running Tasks

**Actions** extend services by providing:
- **Feedback**: Progress updates during execution
- **Cancellation**: Ability to abort ongoing tasks
- **Goal status**: Track completion state

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Generate Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)  # Simulate work
            
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Action Client Example

```python
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')
```

## Message Types

### Standard Messages

```python
from std_msgs.msg import String, Int32, Float64, Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image, LaserScan
```

### Custom Messages

Create `my_package/msg/Person.msg`:

```
string name
uint8 age
float32 height
```

Build and use:

```python
from my_package.msg import Person

msg = Person()
msg.name = "Alice"
msg.age = 30
msg.height = 1.75
```

## Practical Exercise: Robot Velocity Controller

### Goal
Create a node that subscribes to joystick input and publishes velocity commands.

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
    def joy_callback(self, msg):
        twist = Twist()
        # Map joystick axes to velocity
        twist.linear.x = msg.axes[1] * 0.5  # Forward/backward
        twist.angular.z = msg.axes[0] * 1.0  # Left/right
        self.cmd_pub.publish(twist)
```
 
## Debugging Tips

### Visualize Node Graph

```bash
# Install rqt_graph
sudo apt install ros-humble-rqt-graph

# Launch
rqt_graph
```

### Monitor Topic Frequency

```bash
ros2 topic hz /cmd_vel
```

### Echo Raw Messages

```bash
ros2 topic echo /joy
```

## Key Takeaways

1. **QoS settings** determine reliability vs performance tradeoffs
2. **Actions** are perfect for long-running tasks with feedback
3. **Custom messages** enable domain-specific communication
4. **Tools like rqt_graph** help visualize system architecture

## Next Steps

Continue to [Week 5: Advanced ROS 2](./week-5-advanced.md) for parameters, launch files, and lifecycle nodes.
