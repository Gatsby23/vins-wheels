# VINS-Wheels
## VIO system incorporating wheel encoders

This work is based on VINS-Fusion

**Videos:**

<a href="https://youtu.be/lcgBF64WMOE" target="_blank"><img src="http://img.youtube.com/vi/lcgBF64WMOE/0.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. KAIST Example
Download [EuRoC MAV Dataset](https://sites.google.com/view/complex-urban-dataset/download-lidar-stereo) to YOUR_DATASET_FOLDER. 

### 3.1 Monocualr camera + IMU + wheels

```
    roslaunch vins vins_rviz.launch
    rosrun vins kaist_viwo  src/vins-wheels/coig/kaist/kaist_cam0_viwo_38-39.yaml /home/q_ftp/DataSet1/KAIST/kaist39/
    or
    rosrun vins kaist_viwo  src/vins-wheels/coig/kaist/kaist_cam0_viwo_38-39.yaml /home/q_ftp/DataSet1/KAIST/kaist39/ /output_folder
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
```
