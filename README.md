# How to run the project:

To effectively run and execute the project follow theese steps. 
Firs of all, clone the repository in your Laptop into your workspace:
```Shell
git clone https://github.com/soyhorteconh/omni_ros2.git
```

## 1. Install dependencies 

*Dependencies for Raspberry Pi* (Theese dependencies are already installed on the board, in case of any error with one of those run: )

```Shell
pip install grpcio
pip install grpcio-tools
pip install pygame
pip install protobuf==3.20.*

# Build proto for Raspberry Pi:
cd ~/[your ws]/src/oav_utils/scripts
python3 -m grpc_tools.protoc -I ./interface --python_out=. --grpc_python_out=. ./interface/rpi-motor.proto
```

*Dependencies for Laptop* In order to run the project on your laptop, you need to install the following dependencies:

```Shell
sudo apt install ros-humble-joy
sudo apt install ros-humble-joy-linux
sudo apt install joystick jstest-gtk
```

*Turn on the hotstop* Make sure that the hotspot is turned on so the Raspberry Pi, Jetson and the your Laptop are in the same network.
The wifi name should something like "ecm2g-9d4759" and the password is "1243663215910"

## 2. Running the project 

To ensure a correct execution of the project, follow these steps:

### SSH Interface
#### Jetson
1. Connect to the Jetson board via SSH:
```Shell
ssh user@192.168.2.102
```

If have any troubles connecting, you may connect via HDMI/DisplayPort the Jetson board and verify if the network is properly configured.


2. Move into the workspace directory:
```Shell
cd ~/data/[your_ws]
```

3. Then, launch the ROS2 node:
```Shell
ros2 launch oav_utils jetson_launch.py
```

In the case that you get an error related with "no module named launch oav_utils," you need to rebuild the workspace:
```Shell
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

And then, try to launch the node again,you should not get the error anymore.

#### Raspberry

1. Connect to the Raspberry Pi via SSH:
```Shell
ssh pi@192.168.2.103
```

2. Move into the workspace ditectory
```Shell
cd ~/omni_ros2/src/oav_utils/scripts
```
3. Run the Raspberry Pi motor service:
```Shell
Shell
python3 RPIMotorService.py
```

In case you get an error related with the scripts you will need to rebuild the proto, follow the instructions in the "Install dependencies" section. And rebuild the workspace:
```Shell
cd ~/omni_ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### Laptop

1. Joystick Configuration:
```Shell
ls /dev/input/
jstest-gtk
```

2. Move into the workspace directory and build the workspace:
```Shell
cd ~/[your_ws]
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

3. Launch the ROS2 node:
```Shell
ros2 launch oav_utils gs_launch.py
```

