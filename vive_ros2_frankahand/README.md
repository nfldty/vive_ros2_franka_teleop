# vive_ros2_frankahand
Code for using the HTC vive tracking system with ROS2 for Franka hand teleoperation. This package supports the HTC Vive controller for teleoperating a Franka robot hand.

This package allows for maximum flexibility in terms of where the HTC drivers are run
since the package uses an independent server client architecture using python socket library.
For example, server can be run on a more powerful Windows machine for best driver support while
the client can be run on the robot or target device. The server and client can also
be run on the same device if desired.

## Getting Started
### Running the Python Server
The best way to get started is to install conda and run `conda env create -f env.yaml` which 
will create a conda environment with the correct python version and libraries. You can then
activate the environment with `conda activate pythonvr`. Start the server by running `vive_tracker_server.py` in 
the `vive_server` folder. Make sure to have `SteamVR` installed and running with the headset plugged in.

Once you start the server you should see something like the following printed to the screen indicating which VR devices 
are detected and what the ip address and port (IP:PORT) of the server is. This information will come in handy when 
setting up the client.

```
13:49:17|ViveTrackerServer|INFO|Starting server at 192.168.50.171:8000
13:49:17|ViveTrackerServer|INFO|Connected VR devices: 
###########
Found 4 Tracking References
  tracking_reference_1 (LHB-59F8D726, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_2 (LHB-AA3BEC72, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_3 (LHB-FFCE0AD4, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_4 (LHB-91047ECC, Mode Valve SR Imp, Valve SR Imp)
Found 1 HMD
  hmd_1 (LHR-8280F84D, VIVE_Pro MV)
Found 1 Controller
  controller_1 (LHR-4F3DC6EA, VIVE Controller Pro MV)
Found 1 Tracker
  tracker_1 (LHR-55804C5D, VIVE Tracker Pro MV)
###########
```

### Running the Franka Teleoperation Client
In order to run the teleoperation client, first install the required dependencies:

```bash
# Install Python dependencies
pip install pydantic scipy numpy roboticstoolbox
```

You'll also need to install the vive_server package:

```bash
# Navigate to the vive_server directory
cd vive_server
pip install -e .
```

You can then run the teleoperation client:

```bash
# Navigate to the scripts directory
cd vive_ros2/scripts

# Run the client
# Note: You may need to update the HOST and PORT in vive_teleopt.py to match your server
python vive_teleopt.py
```

### Controller Commands

When using the teleoperation client:

- **Trigger button**: Hold to move the robot arm
- **Menu button**: Recalibrate the orientation
- **Grip button**: Toggle the gripper open/closed

## Troubleshooting

- If the server cannot detect your Vive devices, ensure SteamVR is running and recognizing all hardware
- If the client cannot connect to the server, check firewall settings and ensure the IP address and port are correct
- If you see connection errors, verify that the HOST and PORT in vive_teleopt.py match your server's IP and port
