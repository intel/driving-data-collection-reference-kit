DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
# Driving Data Collection Reference kit

Our motivation for a data collection framework is to enable a community based effort to collect driving data. Challenges in the ecosystem are high cost and steep learning curve of technical know-how. A low cost off-the-shelf solution used as-is falls short to meet reliability, quality, performance and realtime requirements. There are several systems for real time data collection systems are quite prohibitive for a developing economy. To address this challenge, we have created a recipe for a reference hardware and the associated software framework which is scalable in performance and minimizes initial capital investment. Also, the stack is designed to achieve maximum throughput possible in a commercial automotive grade system with real time constraints.

For technical details, please refer to:

[Johnny Jacob](http://orcid.org/0000-0002-4621-4237), [Pankaj Rabha](http://orcid.org/0000-0003-4477-0464), **"Driving data collection framework using low cost hardware"** , Proc. of the AutoNUE Workshop, European Conference on Computer Vision (ECCV) 2018, [http://cvit.iiit.ac.in/autonue2018/](http://cvit.iiit.ac.in/autonue2018/) 

## Requisites

### Hardware

- Intel Core i5 7th Generation or above 
- 16 GB RAM
- Minimum 2 TB SSD with 500+ MB/s write
- Wifi
- Ethernet
- USB 3.0 
- USB 2.0
- Ethernet Hub (Incase of using more than one host)
- Powersupply

#### Typical Setup

A typical system implemented with these guidelines and software would look like this :

![Typical Hardware Setup](https://github.com/intel/driving-data-collection-reference-kit/blob/master/docs/images/typical-hardware.png)

In our setup, we used off-the-self parts :
- Intel NUC - [SKU : BLKNUC7I7DNHE](https://www.mouser.in/ProductDetail/Intel/BLKNUC7i7DNHE)
- 8 GB DDR4 x 2
- 1 TB Samsung 860 EVO (MZ-76E1T0BW) SSD
- [DCDC-NUC, 6-48V automotiove power supply for NUC, 12V or 19V output](http://www.mini-box.com/DCDC-NUC)
- [ZED Stereo Camera](https://www.stereolabs.com/zed/)

![Hardware setup mounted on a electric car](https://github.com/intel/driving-data-collection-reference-kit/blob/master/docs/images/hardware-on-a-car.png)

### Software

- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic)
- Python 2

## Installation

Install a standard Ubuntu 16.04 on to the system. Incase you have plans to swap harddisks to copy data, we suggest you to use a USB thumb drive for OS installation.

### Compiling

1. Follow the instructions from  http://wiki.ros.org/kinetic/Installation/Ubuntu to install ROS Kinetic

2. Create a catkin workspace by following the steps below:
```bash
source /opt/ros/kinetic/setup.bash
mkdir -p <CATKIN_DIR_NAME>/src
cd <CATKIN_DIR>
catkin_make
```
- Update CMakeLists.txt
```bash
sudo vim <CATKIN_DIR_NAME>/src/CMakeLists.txt
```
- Add following the flags 
```make
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -D_FORTIFY_SOURCE=2 -fstack-protector-strong -fPIE -fPIC -Wformat -Wformat-security")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now") 
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -z noexecstack -z relro -z now -pie")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now -pie")
```
3. Clone the source code for driving data collection ref kit.
catkin_make would have written some files to the folder. Hence instead of git clone, we do a git init + git pull 
```bash
cd ~/catkin_ws/src
git init . 
git remote add origin repository_url
git pull origin master
```
4. Some of the sensors are taken as submodules in our repository,
- Download all the submodules using 
```bash
git submodule update --init --recursive'
```
Cameras:
In case PtGrey cameras are used,
- Camera needs [Fly Capture SDK](https://www.ptgrey.com/flycapture-sdk) to be installed, download the SDK and install it.
- Install libpcap
```bash
sudo apt-get install libpcap0.8-dev
```
Calibration:
- swatbotics APRIL TAGS requires open cv to be installed seperately, Install latest version of opencv from : https://opencv.org/releases.html.
- Lidar camera calibration and zed_cpu_ros are fetched as a git submodule.

CAN:
If a Kvaser CAN transreciever is used,
- Download CAN library from : https://www.kvaser.com/linux-drivers-and-sdk/ and install CAN Drivers and sdk.

5. For Dashboard, please install the following:
- Install mosuitto MQTT broker
```bash
apt-get install libmosquitto-dev mosquitto libmosquittopp-dev 
```
- Install libjson
```bash
sudo apt-get install libjson0 libjson0-dev
```
6. Start the built
```bash
catkin_make <CATKIN_DIR>
```

## Configuration
### Storage

Format SSD to use either XFS or BTrFS which gives a better write speed.

### WiFi Hotspot for Dashboard

Wifi needs to be configured for hotspot to host the dashboard.

### Distributed Data Collection

To setup multiple hosts for collecting data refer http://wiki.ros.org/ROS/NetworkSetup and  http://wiki.ros.org/ROS/Tutorials/MultipleMachines 

## Capturing Data

Launch all the ROS nodes in all the hosts.
```bash
rosrecord <list of topics>
```
## Post Processing

```bash
consolidate.py meta.bag
```

# Design Considerations

To learn about design considerations, read [docs/design.md](docs/design.md) . 




# Known Issues / Limitations
- Only USB cameras are supported as of now.
- Data from sensors are not tightly synchronised. This needs an external trigger.
- Second stage processing takes longer time because of sequential writes for ROS bag. 

# LICENSE

BSD-3-Clause

