# LOAM Mapper for Autoware Labs

In Autoware Labs, LOAM Based Localization will be used. For this purpose, we need to create a tool
for generating LOAM feature point cloud maps. This tool is created for that reason.

<p align='center'>
    <img src="images/loam_mapper_edge_and_surface_features.png" 
        alt="loam_mapper_edge_and_surface_features" width="80%"/>
</p>

Here are the videos shows how the tool works after running.
- Full process with the sample data. Shows Rviz point cloud visualization and saved point clouds.
    - https://youtu.be/jVBdZYEii20

- Closer look to the extracted features:
    - https://youtu.be/b4RjppSeNmw
    - https://youtu.be/b4RjppSeNmw
        - Red points shows the edge features.
        - Yellow points shows the surface features.

## Used data for development
We are using the below data for development 
- Ground truth pose data which contains the post-processed precise time, position and orientation data.
- PCAP file that contains LiDAR packets supported with PPS signal and GPRMC message.

You can find the data in the link below:
- https://drive.google.com/file/d/1ivVL4hYuqqzlTSMTbJV7gEvlbPvMJ7-M/view?usp=drive_link

## The Program
The program has 2 stages. Basic mapping part and LOAM mapping part. 

### Basic Mapping Part
In basic mapping part, points are  extracted from the PCAP files with precise time information. 
In that way, we can match all the LiDAR, point data with the corresponding ground truth
position via time. 

After matching, all the LiDAR points are transformed into the corresponding position with LiDAR-IMU
calibrated matrix.

Corresponding Issue in Autoware: https://github.com/autowarefoundation/autoware.universe/issues/6836

> You should split the PCAP file given in the data folder in order to get rid of errors caused by RAM overfilling.
> ```commandline
> editcap -c 100000 ytu_map_2_08_04_23.pcap pcaps/ytu_campus.pcap
> ```

### LOAM Mapping Part
This will be filled after LOAM mapping part is done.

Corresponding issues in Autoware:
- https://github.com/autowarefoundation/autoware.universe/issues/6837
- https://github.com/autowarefoundation/autoware.universe/issues/6838

## Dependencies
The program is using below libraries:
- [rclcpp](https://docs.ros.org/en/humble/Installation.html) (for parameter setting and debugging)
- [PcapPlusPlus](https://pcapplusplus.github.io/docs/install) (please install from the source. `cmake/FindPcapPlusPlus.cmake` will help to find it)
- [PCL] (https://pointclouds.org/) 

## Usage
### Setting the Environment
This is a ROS2 package. So, the installation instructions are the same with all packages. We are 
using the below commands for using.
```commandline
mkdir -p loam_mapping_ws/src
cd loam_mapping_ws/src
git clone https://github.com/ataparlar/loam-mapper
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Running the LOAM Mapper
 ```commandline
 source loam_mapping_ws/install/setup.bash
 ros2 launch loam_mapper loam_mapper.launch.py
 ```

## Parameters

| Param                | Description                                                                           |
|----------------------|---------------------------------------------------------------------------------------|
| pcap_dir_path        | The folder path contains the PCAPs.                                                   |
| pose_txt_path        | Path of the ground truth poses.                                                       |
| pcd_export_directory | Export directory for PCD files.                                                       |
| map_origin_x         | X-coordinate of the origin of the map.                                                |
| map_origin_y         | Y-coordinate of the origin of the map.                                                |
| map_origin_z         | Z-coordinate of the origin of the map.                                                |
| imu2lidar_roll       | LiDAR-IMU calibration **X** angle.                                                    |
| imu2lidar_pitch      | LiDAR-IMU calibration **Y** angle.                                                    |
| imu2lidar_yaw        | LiDAR-IMU calibration **Z** angle.                                                    |
| enable_ned2enu       | Decider parameter for enabling NED to ENU transform for LiDAR-IMU calibration values. |
| voxel_resolution     | Voxel resolution param for downsampling. (lower means denser point cloud)             |
| save_pcd             | Decider parameter for saving point cloud as `pcd`.                                    |



## Paper

Part of the code is adapted from [LIO-SAM (IROS-2020)](./config/doc/paper.pdf).
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

Part of the code is adapted from [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).
```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

## Acknowledgement

- loam_mapper is based on the feature extraction part of LIO-SAM (Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping).
