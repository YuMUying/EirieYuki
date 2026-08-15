# WCR Robot - ROS2 Workspace

A comprehensive ROS2 workspace for a WCR 4WIS4WID with advanced control algorithms, simulation capabilities, and odometry estimation.

## 📋 Overview

This workspace contains a complete robot system implementation including URDF description, Gazebo simulation, multiple control strategies, odometry estimation, and launch configurations for WCR 4WIS4WID robot with steering capabilities.

## 🏗️ Package Structure

### `wcr_description`
Robot description package containing URDF/Xacro models and visualization configurations.

**Features:**
- Complete robot URDF model with configurable parameters
- Multiple ros2_control hardware interface configurations:
- Gazebo world files and simulation plugins
- RViz configuration files for visualization
- Robot meshes and visual assets

**Launch Files:**
- `display.launch.py` - Visualize robot in RViz
- `gazebo.launch.py` - Launch robot in Gazebo simulation

### `wcr_controllers`
[Custom controller implementations](https://github.com/BCaran/wcr_controllers) for trajectory tracking and motion control.

### `wcr_control`
ROS2 control configuration and controller management.

**Launch Files:**
- `controller.launch.py` - Start controller manager and load controllers

### `wcr_odometry`
[Odometry estimation node](https://github.com/BCaran/wcr_odometry) for robot state feedback.

### `wcr_launcher`
Unified launch package for starting the complete robot system.

**Features:**
- Centralized launch configuration
- YAML-based parameter management
- Conditional launching
- Namespace management
- Robot state publisher integration

**Configuration:**
- `config/robot_params.yaml` - Robot physical parameters:
  - Chassis dimensions (225mm x 225mm) and mass (3.0 kg)
  - Steering system parameters
  - Wheel specifications
  - Propeller configuration
  - EDF (Electric Ducted Fan) parameters
  - Inertia properties

**Launch Files:**
- `launcher.launch.py` - Main launch file for the complete system

## 🚀 Getting Started

### Prerequisites

- ROS2 (Humble/Iron/Rolling recommended)
- Gazebo (for simulation)
- ros2_control and ros2_controllers
- Python 3.8+
- CMake and build tools

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## 🎮 Usage

### Launch the Complete System

**Gazebo:**
```bash
ros2 launch wcr_launcher launcher.launch.py
```

## 🔧 Configuration

### Robot Parameters

Edit [config/robot_params.yaml](wcr_launcher/config/robot_params.yaml) to modify:
- Robot dimensions and mass
- Wheel and steering specifications
- Motor effort and velocity limits
- Inertia parameters
- Propeller and EDF configurations

Edit [config/launch_params.yaml](wcr_launcher/config/launch_params.yaml) to modify:
- World
- URDF files
- Control variants

### Controller Selection

Choose the appropriate ros2_control configuration in your URDF by including the desired xacro file:
- For testing/simulation: `mock_wcr_classic.ros2_control.xacro` (use [config/launch_params.yaml](wcr_launcher/config/launch_params.yaml): `variant: "mock"`)

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [gz_ros2_control Documentation](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [Gazebo Documentation](https://gazebosim.org/docs/fortress/getstarted/)

