# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites
- I have tested on below version.
  - Ubuntu 22.04
  - ROS2 Humble
  - OpenCV 4.5.4

- Build ORB_SLAM3
  - Follow the build instructions in the repo below . you may need to change the CMake version
  - After you build and follow the instruction go to `home/<username>/<orbslam_directory>/Thirdparty/Sophus/build` and do `sudo make install`
  - You don't necessarly have to download opencv from source as specified in the repo below
  - Go to this [repo](https://github.com/bharath5673/ORB-SLAM3/tree/main) and follow build instruction.
  

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
1. Clone repository to your ROS workspace
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/Melasmar-711/ORB_SLAM3_ROS2.git
$ git checkout RAMI_ORBSLAM3
```

2. Change this [line](https://github.com/Melasmar-711/ORB_SLAM3_ROS2/blob/d2f4a32196428e7e47d6e2ed496ef6395422bd85/CMakeLists.txt#L5C22-L5C66) to your own `python site-packages` path

3. Change this [line](https://github.com/Melasmar-711/ORB_SLAM3_ROS2/blob/d2f4a32196428e7e47d6e2ed496ef6395422bd85/CMakeModules/FindORB_SLAM3.cmake#L8C1-L8C52) to your own `ORB_SLAM3` path

Now, you are ready to build!
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```
2. Please compile with `OpenCV 4.5.4` version atleast.

## How to use
1. Source the workspace  
```
$ source ~/colcon_ws/install/local_setup.bash
```
2. before running the nodes  
```
tar -xvzf <Your_Package_Dir>/vocabulary/ORBvoc.txt.tar.gz
```
3. if you run the next command without specifying that you need the webcam the slam will run by subscribing to an image topic name `/sf/AUV/rgb_camera/image_color` which is the topic coming out of the simulation node.
```
ros2 run orbslam3 mono <Package_Dir>/vocabulary/ORBvoc.txt <Package_Dir>/config/monocular/sim_camera.yaml 
```
4. you can from the web cam using 
```
ros2 run orbslam3 mono <Package_Dir>/vocabulary/ORBvoc.txt <Package_Dir>/config/monocular/web_cam.yaml --webcam

```




## Acknowledgments
This repository is modified from [this](https://github.com/zang09/ORB_SLAM3_ROS2) repository.  
working with ubuntu 22.04 and adding an option to run from a usb or web camera
