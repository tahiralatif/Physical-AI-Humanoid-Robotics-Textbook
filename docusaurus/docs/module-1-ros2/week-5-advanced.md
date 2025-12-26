---
sidebar_label: 'Week 5: Advanced ROS 2'
sidebar_position: 3
estimated_time: '5-6 hours'
week: 5
module: 1
prerequisites:
  - Weeks 3-4 completed
learning_objectives:
  - Configure nodes using ROS 2 parameters
  - Create launch files for multi-node systems
  - Implement lifecycle nodes for controlled startup/shutdown
---

# Week 5: Advanced ROS 2 Features

## Parameters: Runtime Configuration

**Parameters** allow users to configure node behavior without recompiling.

### Declaring and Using Parameters

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        
        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value
        
        self.get_logger().info(f'Max speed: {self.max_speed}')
        self.get_logger().info(f'Robot name: {self.robot_name}')
```

### Setting Parameters from Command Line

```bash
# Run node with custom parameters
ros2 run my_package parameter_node --ros-args \
  -p max_speed:=2.0 \
  -p robot_name:=robo1
```

### Parameter Callbacks

```python
def __init__(self):
    super().__init__('parameter_node')
    self.declare_parameter('max_speed', 1.0)
    
    # Add callback for parameter changes
    self.add_on_set_parameters_callback(self.parameters_callback)
    
def parameters_callback(self, params):
    for param in params:
        if param.name == 'max_speed':
            self.get_logger().info(f'Max speed changed to {param.value}')
    return SetParametersResult(successful=True)
```

## Launch Files: Orchestrating Systems

**Launch files** start multiple nodes with specific configurations.

### Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='talker_node',
            parameters=[{
                'topic_name': 'chatter',
                'publish_rate': 10.0
            }]
        ),
        Node(
            package='my_package',
            executable='listener',
            name='listener_node',
        ),
    ])
```

### Launch with Remapping

```python
Node(
    package='camera_driver',
    executable='camera_node',
    remappings=[
        ('image', 'camera/image_raw'),
        ('info', 'camera/camera_info')
    ]
)
```

### Include Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('package_name'),
        '/launch/file.launch.py'
    ])
)
```

## Lifecycle Nodes: Controlled State Management

**Lifecycle nodes** provide deterministic startup/shutdown sequences.

### States

1. **Unconfigured**: Initial state
2. **Inactive**: Configured but not running
3. **Active**: Fully operational
4. **Finalized**: Being torn down

### Lifecycle Node Example

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class CameraNode(LifecycleNode):
    def on_configure(self, state):
        self.get_logger().info('Configuring camera...')
        # Initialize hardware
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):
        self.get_logger().info('Activating camera...')
        # Start capturing
        self.timer = self.create_timer(0.1, self.capture_image)
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(self, state):
        self.get_logger().info('Deactivating camera...')
        # Stop capturing
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up camera...')
        # Release hardware
        return TransitionCallbackReturn.SUCCESS
```

### Managing Lifecycle

```bash
# Trigger lifecycle transitions
ros2 lifecycle set /camera_node configure
ros2 lifecycle set /camera_node activate
ros2 lifecycle set /camera_node deactivate
ros2 lifecycle set /camera_node cleanup
```

## Practical Project: Multi-Robot System

### Goal
Create a launch file that starts:
- 2 robot nodes with different namespaces
- A central coordinator node
- Parameter file for configuration

```python
def generate_launch_description():
    # Load parameters from YAML
    config = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        # Robot 1
        Node(
            package='my_package',
            namespace='robot1',
            executable='robot_node',
            name='robot',
            parameters=[config]
        ),
        
        # Robot 2
        Node(
            package='my_package',
            namespace='robot2',
            executable='robot_node',
            name='robot',
            parameters=[config]
        ),
        
        # Coordinator
        Node(
            package='my_package',
            executable='coordinator',
            name='coordinator',
            parameters=[config]
        ),
    ])
```

## Best Practices

1. **Use parameters** for all configurable values
2. **Create launch files** for complex systems
3. **Use lifecycle nodes** for hardware interfaces
4. **Namespace nodes** when running multiple instances
5. **Document parameters** in README files

## ROS 2 Package Structure

```
my_package/
├── launch/           # Launch files
│   └── system.launch.py
├── config/           # Parameter YAML files
│   └── params.yaml
├── src/              # Python source
│   └── my_node.py
├── package.xml       # Package metadata
└── setup.py          # Python package setup
```

## Key Takeaways

1. **Parameters** enable runtime configuration
2. **Launch files** orchestrate complex systems
3. **Lifecycle nodes** provide controlled state transitions
4. **Namespaces** enable multi-robot deployments

## Assessment: Build a ROS 2 Package

Complete the [ROS 2 Package Assessment](/docs/assessments/ros2-package) to demonstrate mastery of:
- Node creation
- Parameter configuration
- Launch file development
- Topic/service communication

## Next Module

Congratulations on completing Module 1! Continue to [Module 2: Digital Twin](/docs/module-2-digital-twin/week-6-gazebo) to learn simulation.
