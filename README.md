# Template: template-basic

This template provides a boilerplate repository for developing non-ROS software
in Duckietown.

**NOTE:** If you want to develop software that uses ROS, check out
[this template](https://github.com/duckietown/template-ros).


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your code in the directory `/packages/` of
your new repository.


### 5. Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.

## Getting the RPLiDAR to work on the Duckiebot

#### Connect to Duckiebot
`dts devel build -H sonic.local`

`dts devel run -H sonic.local -s -M --cmd bash -f -- --device=/dev/ttyUSB0`


```
cd packages/icp_ws
catkin_make
source devel/setup.bash
rosrun icp lidar_pub.py
```

When using RVIZ
In a seperate terminal run: 
rosrun tf static_transform_publisher 0 0 0 0 0 0.1 1 map sonic/lidar_frame 10

  File "/code/cuda_duckietown/packages/icp_ws/src/icp/src/icp_serial.py", line 81, in icp_svd
    norm_values.append(np.linalg.norm(p - q))
ValueError: operands could not be broadcast together with shapes (2,411) (2,386) 


# Todo: 
Find correct frame adjustment, need to add some rotation
Figure out how to pass that information as a default
Flip point cloud accordingly

If the lidar throws an exception saying 'rplidar.RPLidarException: Incorrect descriptor starting bytes', 
Try running again, then unplug and replug in the lidar and restart the node/ It's likely the lidar wasn't stopped properly. Also make sure your cable is good.

Add `--device=/dev/ttyACM0` to the end of the `dts devel run` command to pass the device information through to the docker container.
dts devel run -H sonic.local --cmd bash -f -- --device=/dev/ttyUSB0 

## Getting CUDA to work on Duckiebot

### 1. Change Dockerfile to: `ARG ARCH=arm32v7, ARG DISTRO=daffy`

### 2. Build Docker Container `dts devel build -f -H <duckiebot_name>.local`

### 3. Run Docker Container `dts devel run -H <duckeibot_name>.local -s -M --cmd bash`

### 4. Downgrade compiler in container down to 7

Add 
```
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100
RUN sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 100
```
to the dockerfile and `gcc-7 g++-7` to the dependencies-apt.txt

Due to the changes above, the steps below should now be obsolete. 

```
sudo apt remove gcc

apt update

apt install g++-7

sudo ln -s /usr/bin/gcc-7 /usr/bin/gcc

sudo ln -s /usr/bin/g++-7 /usr/bin/g++

sudo ln -s /usr/bin/gcc-7 /usr/bin/cc

sudo ln -s /usr/bin/g++-7 /usr/bin/c++
```
Run `gcc --version` to make sure it works
