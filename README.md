# Modulr Unitree Go2

This ROS 2 package provides support for the Modulr Agent running on the Unitree Go2 robot. It includes nodes for motion control forwarding, camera streaming, and video transmission over ZMQ.

## Overview

The package provides three main executables:
- **fwd_motion**: Forwards motion commands to the Unitree Go2 robot
- **fwd_camera**: Forwards camera data from the robot as ROS 2 sensor messages
- **unitree_video_zmq**: Streams video from the Unitree SDK over ZMQ for efficient network transmission

## Requirements

- ROS 2 (Foxy is already installed on Go2)
- Unitree Go2 Edu with standard installation
- SSH access to Go2
- Wireless connection to the internet from Go2 (for remote operation)
- Dependencies:
  - `unitree_go` (Unitree ROS 2 messages, already installed on system)
  - `zmq` (ZeroMQ)

## System Setup (Optional, Recommended)

Before installing this package, you may configure the Go2's bash environment to automatically load ROS 2 Foxy and the Unitree messages.

### Modifying .bashrc

The default Unitree Go2 installation includes an interactive prompt in `~/.bashrc` that asks you to choose between ROS Foxy and ROS Noetic. This may be replaced with automatic loading.

1. SSH into your Go2 robot:
```bash
ssh unitree@<go2-ip-address>
```

2. Edit the `~/.bashrc` file:
```bash
nano ~/.bashrc
```

3. Find and replace the fishros initialization section. Replace this:
```bash
# >>> fishros initialize >>>
echo "ros:foxy(1) noetic(2) ?"
read choose
case $choose in
1) source  /opt/ros/foxy/setup.bash;
source ~/cyclonedds_ws/install/setup.bash;
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;;
2) source  /opt/ros/noetic/setup.bash;;
esac
# <<< fishros initialize <<<
```

With this:
```bash
# Choose foxy automatically
source ~/cyclonedds_ws/install/setup.bash;
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;

# Load extra messages from unitree_go
source /unitree/module/graph_pid_ws/install/setup.bash
```

4. Save and exit

5. Apply the changes:
```bash
source ~/.bashrc
```

This configuration ensures that ROS 2 Foxy and the Unitree Go2 messages are automatically loaded when you SSH into the robot.

## Installation

1. Clone the repository with submodules into a new ROS 2 workspace:
```bash
mkdir -p ~/local_ws/src
git clone --recursive https://github.com/your-org/modulr-unitree-go2.git ~/local_ws/src/modulr_unitree_go2
```

Or if you already cloned it, initialize the submodules:
```bash
git submodule update --init --recursive
```

2. Install dependencies:
```bash
sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-sensor-msgs libzmq3-dev
```

3. Build the package:
```bash
cd ~/local_ws
colcon build --packages-select modulr_unitree_go2
source install/setup.bash
```

## Autostart on Boot (Optional, Recommended)

To automatically launch the nodes when the Go2 boots up, you can install a systemd user service.

### Installing the Service

1. Navigate to the package directory:
```bash
cd ~/local_ws/src/modulr_unitree_go2/systemd
```

2. Run the installation script:
```bash
./install_service.sh
```

This will:
- Install the service file to `~/.config/systemd/user/`
- Enable the service to start automatically on boot
- Enable user lingering (allows the service to run without an active login session)

### Managing the Service

Start the service immediately:
```bash
systemctl --user start modulr-unitree-go2.service
```

Stop the service:
```bash
systemctl --user stop modulr-unitree-go2.service
```

Check service status:
```bash
systemctl --user status modulr-unitree-go2.service
```

View real-time logs:
```bash
journalctl --user -u modulr-unitree-go2.service -f
```

Disable autostart:
```bash
systemctl --user disable modulr-unitree-go2.service
```

Restart the service:
```bash
systemctl --user restart modulr-unitree-go2.service
```

### Service Configuration

The service file is located at [systemd/modulr-unitree-go2.service](systemd/modulr-unitree-go2.service) and includes:
- Automatic restart on failure (with 5-second delay)
- Proper environment variable setup for ROS 2 and CycloneDDS
- Logging to systemd journal

If you need to customize the service (e.g., change environment variables or workspace paths), edit the service file and run:
```bash
systemctl --user daemon-reload
systemctl --user restart modulr-unitree-go2.service
```

## Usage

### All Nodes (Recommended)

It is recommended to launch all nodes at the same time using the bringup launch file:

```bash
ros2 launch modulr_unitree_go2 bringup.launch.py
```

This will start all three nodes (fwd_motion, fwd_camera, and unitree_video_zmq) simultaneously for complete robot operation.

### Motion Forwarding Node

Forward motion commands to the robot:
```bash
ros2 run modulr_unitree_go2 fwd_motion
```

This node subscribes to motion command topics and forwards them to the Unitree Go2 robot using the Unitree SDK.

### Camera Forwarding Node

Forward camera data from the robot:
```bash
ros2 run modulr_unitree_go2 fwd_camera
```

This node receives camera data over ZMQ and publishes it as ROS 2 sensor_msgs/Image messages.

### Video ZMQ Node

Stream video from the Unitree SDK over ZMQ:
```bash
ros2 run modulr_unitree_go2 unitree_video_zmq
```

This executable uses the Unitree SDK2 to capture video and stream it over ZMQ for low-latency network transmission.

## Package Structure

```
modulr_unitree_go2/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS 2 package metadata
├── README.md               # This file
├── src/
│   ├── fwd_motion.cpp      # Motion forwarding node
│   ├── fwd_camera.cpp      # Camera forwarding node
│   └── unitree_video_zmq.cpp # Video streaming via ZMQ
├── launch/                 # ROS 2 launch files
│   └── bringup.launch.py   # Main launch file for all nodes
├── systemd/                # Systemd service files
│   ├── modulr-unitree-go2.service  # User service definition
│   └── install_service.sh  # Service installation script
└── third_party/
    └── unitree_sdk2/       # Unitree SDK2 submodule
```

## Third-Party Dependencies

This package includes the following third-party dependency as a git submodule:
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2.git): Official SDK for Unitree robots

## License

MIT-0

## Maintainer

Michael Hart (Modulr) - michael@modulr.cloud

## Contributing

For issues, questions, or contributions, please contact the maintainer.
