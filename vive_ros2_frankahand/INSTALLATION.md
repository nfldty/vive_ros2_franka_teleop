# Installation Guide for Vive ROS2 with Franka Hand Teleoperation

This guide will help you set up the vive_ros2_frankahand project, which allows you to use the HTC Vive tracking system for teleoperation of a Franka robot hand.

## System Requirements

- **Server**: Windows computer with SteamVR (recommended for best HTC Vive driver support)
- **Client**: Linux machine with ROS2 (can be the same machine as the server if using Windows with WSL2)
- HTC Vive system with trackers and controllers
- Franka robot (optional, for teleoperation)

## Server Setup (vive_server)

### 1. Install Dependencies

The server requires Python with specific packages. We recommend using Conda:

```bash
# Create the conda environment from the provided configuration
conda env create -f env.yaml

# Activate the environment
conda activate pythonvr
```

### 2. Install SteamVR

1. Install Steam from [https://store.steampowered.com/](https://store.steampowered.com/)
2. Install SteamVR from the Steam store
3. Connect your HTC Vive hardware (headset, base stations, trackers, controllers)
4. Start SteamVR and ensure your devices are detected

### 3. Run the Server

Start the server by running `vive_tracker_server.py` in the `vive_server` folder. Make sure to have `SteamVR` installed and running with the headset plugged in.

After starting the server, note the IP address and port displayed in the console (e.g., `192.168.50.171:8000`). You'll need these for the client configuration.

## Client Setup (vive_ros2)

### 1. Install ROS2

Follow the official ROS2 installation instructions for your distribution: [ROS2 Installation Guide](https://docs.ros.org/en/galactic/Installation.html)

### 2. Install Additional Dependencies

```bash
# Install Python dependencies
pip install pydantic scipy numpy

# For Franka robot control (if using teleoperation)
pip install roboticstoolbox
```

### 3. Build the ROS2 Package

```bash
# Navigate to your ROS2 workspace (create one if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository (if not already done)
git clone <repository_url> vive_ros2_frankahand

# Install the vive_server package
cd vive_ros2_frankahand/vive_server
pip install -e .

# Build the ROS2 package
cd ~/ros2_ws
colcon build --packages-select vive_ros2
source install/setup.bash
```

## Running the Teleoperation Client

To use the Vive controller for teleoperating a Franka robot hand:

```bash
# Make sure the vive_server is running
# Navigate to the scripts directory
cd vive_ros2_frankahand/vive_ros2/scripts

# Run the teleoperation client
# Note: You may need to update the HOST and PORT in vive_teleopt.py to match your server
python vive_teleopt.py
```

## Verifying Installation

### Check ROS2 Topics

```bash
# List all ROS2 topics to verify tracker data is being published
ros2 topic list

# Echo the tracker data to see the values
ros2 topic echo /tracker/odom
```

### Controller Commands

When using the teleoperation client (vive_teleopt.py):

- **Trigger button**: Hold to move the robot arm
- **Menu button**: Recalibrate the orientation
- **Grip button**: Toggle the gripper open/closed

## Troubleshooting

- If the server cannot detect your Vive devices, ensure SteamVR is running and recognizing all hardware
- If the client cannot connect to the server, check firewall settings and ensure the IP address and port are correct
- For ROS2 issues, ensure your workspace is properly sourced with `source install/setup.bash` 