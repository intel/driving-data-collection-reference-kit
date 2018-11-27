# Driving Data Collection Reference kit

Software tools that can be used for collecting automotive driving data.

To learn about design considerations, read <TBD> . Also a typical system implemented with these guidelines would look like this.

Tools are available to help in extrinsic calibration of cameras and LIDARs. Check out calibration/

## Requisites

#### Hardware

- Intel Core i5 7th Generation or above 
- 16 GB RAM
- Minimum 2 TB SSD with 500+ MB/s write
- Wifi
- Ethernet
- USB 3.0 
- USB 2.0
- Ethernet Hub (Incase of using more than one host)
- Powersupply

#### Software

- Ubuntu 16.04 
- ROS Kinetic
- Python 2

## Installation

Install a standard Ubuntu 16.04 on to the system. Incase you have plans to swap harddisks to copy data, we suggest you to use a USB thumb drive for OS installation.

### Compiling

1. Follow the instructions from  http://wiki.ros.org/kinetic/Installation/Ubuntu to install ROS Kinetic

2. Create a catkin workspace by following the steps below:
        - source /opt/ros/kinetic/setup.bash
        - Create a catkin directory: mkdir -p <CATKIN_DIR_NAME>/src
        - cd <CATKIN_DIR>
        - catkin_make
	- Update CMakeLists.txt: sudo vim <CATKIN_DIR_NAME>/src/CMakeLists.txt
	- Add : SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -D_FORTIFY_SOURCE=2 -fstack-protector-strong -fPIE -fPIC -Wformat -Wformat-security")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -z noexecstack -z relro -z now -pie")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now -pie")

3. Clone the source code for driving data collection ref kit.
catkin_make would have written some files to the folder. Hence instead of git clone, we do a git init + git pull 
- cd ~/catkin_ws/src
- git init . 
- git remote add origin repository_url
- git pull origin master

4. Some of the sensors are taken as submodules in our repository,
- Initate a 'git submodule update --init --recursive' command to checkout to the each submodule.

Cameras:
In case PtGrey cameras are used,
- Camera needs fly capture sdk to be installed, get the sdk from < @TODO: Update the url to download sdk > and install it.
- Install libpcap : sudo apt-get install libpcap0.8-dev

Calibration:
- swatbotics APRIL TAGS requires open cv to be installed seperately, Install latest version of opencv from : https://opencv.org/releases.html.
- Lidar camera calibration and zed_cpu_ros are fetched as a git submodule.

CAN:
If a Kvaser CAN transreciever is used,
- Download CAN library from : https://www.kvaser.com/linux-drivers-and-sdk/ and install CAN Drivers and sdk.

5. For Dashboard, please install the following:
- Install mosuitto MQTT broker on linux machine using apt-get install libmosquitto-dev mosquitto libmosquittopp-dev 
- Install libjson by executing the below command : 
	sudo apt-get install libjson0 libjson0-dev

6. Perform build: Give " catkin_make " command in <CATKIN_DIR> folder.

## Configuration
### Storage

Format SSD to use either XFS or BTrFS which gives a better write speed.

### WiFi Hotspot for Dashboard

Wifi needs to be configured for hotspot to host the dashboard.

### Distributed Data Collection

To setup multiple hosts for collecting data refer http://wiki.ros.org/ROS/NetworkSetup and  http://wiki.ros.org/ROS/Tutorials/MultipleMachines 

## Capturing Data

Launch all the ROS nodes in all the hosts.

rosrecord [list of topics]

## Post Processing

consolidate.py meta.bag

# Known Issues / Limitations
- Only USB cameras are supported as of now.
- Data from sensors are not tightly synchronised. This needs an external trigger (todo).
- Second stage processing takes longer time because of sequential writes for ROS bag. 

