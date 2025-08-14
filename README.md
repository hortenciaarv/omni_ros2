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
If the IP address is not working, try subsecuent addresses like 192.168.2.104.... and so on. (This is a bug that must be fixed in the future, but it is not a priority right now).

If have any troubles connecting, you may connect via HDMI/DisplayPort the Jetson board and verify if the network is properly configured. Password is *Interfaces22*


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

### Running the docker container with the realsense camera 

To run the docker container with the realsense camera, follow these steps:
On the Jetson board, run the following commands: 
1. To see the available docker containers:
```Shell
sudo docker container ls -a 
```
2. Start the docker container of the project which  is:
```Shell
sudo docker start robotics-work-develop
```
3. Attach the docker container: 
```Shell
sudo docker attach robotics-work-develop
```
Once you are inside the docker container, just run:
```Shell
realsense-viewer
```
This will open the realsense viewer and you will be able to see the camera feed.

In case you get a log error "Authorization required, but no authorization protocol specified", just exit the docker container, and run the following command to fix it:
```Shell
xhost +local:root
```
Then, reattach the docker container and run the `realsense-viewer` command again.

If you get an error related with the camera not being found, you may need to turn off the jetson board, unplug the camera first and then turn it on again. You can also check if the camera is found by going to the `/dev' directory and check if video0 - videoN files are present:
```Shell
cd /dev
ls
```

If you want to just see if the camera is working, run the following command: 
```Shell
cd /home/robotics/data 
python rs2demo.py
```
This will run a simple script that will show the camera feed in a window. If you see the camera feed, then the camera is working properly.

### Running the docker container with the Lidar
In case you have a terminal busy with the realsense viewer, you can open a new terminal and run the following commands to run the docker container with the Lidar:
```Shell
sudo docker exec -it robotics-work-develop bash
```
This will open a new terminal inside the docker container.

To run the docker container with the Lidar, follow these steps: 
On the Jetson board, run the following commands:
1. To see the available docker containers:
```Shell
sudo docker container ls -a
```
2. Start the docker container of the project which is:
```Shell
sudo docker start robotics-work-develop
```
3. Attach the docker container:
```Shell
sudo docker attach robotics-work-develop
```
Once you are inside the docker container, just run:
```Shell
cd home/robotics/data/lidar_ws
colcon build
source ./install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
```
This will launch the RPLidar node and you will be able to see the Lidar data in RViz.

If you still need more help with the Lidar check the official documentation on the github repository: https://github.com/Slamtec/rplidar_ros/tree/ros2

#### Raspberry

1. Connect to the Raspberry Pi via SSH:
```Shell
ssh pi@192.168.2.103
```
Password is *1234*

2. Move into the workspace ditectory
```Shell
cd ~/omni_ros2/src/oav_utils/scripts
```
3. Run the Raspberry Pi motor service:
```Shell
python3 RPIMotorService4T.py
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
 
