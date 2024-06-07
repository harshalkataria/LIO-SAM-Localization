# LIO-SAM-Localization
Estimate the odometry of a robot in pre-built map using LIO-SAM

## Menu
  - [**CHANGELOG**](#changelog)

  - [**Package dependency**](#dependency)

  - [**Package install**](#install)

  - [**Sample datasets, maps and video**](#sample-datasets-maps-and-video-setup)

  - [**Run the package**](#run-the-package)

  - [**Issues**](#issues)

  - [**Paper**](#paper)

  - [**Acknowledgement**](#acknowledgement)

## CHANGELOG
The following changes have been made to LIO-SAM in order to be able to run localization on pre-built map:
* **+mapOptimizationLocalization**
  - **+loadmap()**: This function loads the different .pcd files saved and stores the point cloud data into respective pointers, also creates vectors for map_key_frames_.
  - **extractNearby()**: In this function instead of taking the surroundingKeyPoses by comparing currentKeyPose with previousKeyPose we are now extracting nearby KeyPoses to cloudKeyPoses3D from map.cloudPoses3D.
  - **extractCloud()**: Similarly in this function instead of previousKeyFrames we are taking data from mapKeyFrames associated to map.cloudKeyPoses3D.
* **mapOptimization**
  - **saveKeyFramesAndFactor()**: In this function intensity for the points in cloudKeyFrames is assigned so that we can access accordingly while trying to localize later.

## Dependency

This is the original ROS1 implementation of LIO-SAM.

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Noetic)
  ```
  sudo apt-get install -y ros-noetic-navigation
  sudo apt-get install -y ros-noetic-robot-localization
  sudo apt-get install -y ros-noetic-robot-state-publisher
  ```
- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```

## Install

Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/harshalkataria/LIO-SAM-Localization.git
cd ..
catkin_make -j2
```

## Using Docker
Build image (based on ROS1 Noetic):

```bash
docker build -t gem-lio-noetic-image .
```

Once you have the image, start a container as follows:

```bash
docker run -dt --name gem-lio-noetic-1 \
  --env="DISPLAY" --net host \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/datasets-folder/testVolume-1:/home/ubuntu/testVolume-1 \
  gem-lio-noetic-image /bin/bash
```
## Sample datasets, maps and video Setup

  * Download some sample datasets and maps to test the functionality of the package:
    - **highway dataset:** [[Google Drive](https://drive.google.com/file/d/1cZjXGGRp57_Kic1d46NL9fA3iOlQMDyf/view?usp=sharing)]
    - **map with non-zero intensity:** [[Google Drive](https://drive.google.com/file/d/1hB_dSDCnRh3XYZAdwvpYNu1jc0viFDQ8/view?usp=sharing)]
    - **map with zero intensity:** [[Google Drive](https://drive.google.com/file/d/1H58qwIM2rJ6tREVQbDYd1oZtVaPaNSzY/view?usp=sharing)]
    - **sample mapping video:** [[Google Drive](https://drive.google.com/file/d/1UjE75DK_xf0YKJPGWW3-y9KRIot-oCfA/view?usp=sharing)]
  
  * Setup the sample dataset and map in testVolume-1
    1. Download and extract the [highway-dataset-bagFile](https://drive.google.com/file/d/1cZjXGGRp57_Kic1d46NL9fA3iOlQMDyf/view?usp=sharing)
    2. Copy to the **testVolume-1** folder
    3. Download and extract [[map-with-non-zero-intensity](https://drive.google.com/file/d/1hB_dSDCnRh3XYZAdwvpYNu1jc0viFDQ8/view?usp=sharing)]
    4. Copy the map folder into **testVolume-1** folder

## Run the packages inside the docker container with bag files

* ### Setup terminal

1. On a new terminal give display access to docker:
```
xhost +local:docker
```

2. Start the container if it is stopped:
```
docker start gem-lio-noetic-1
```

3. Get interactive shell access to run commands inside docker container:
```
docker exec -it gem-lio-noetic-1 bash
```

Repeat these steps for each new terminal you open for the commands below.

* ### To create a map using bag file

1. Source the required setup.bash
```
source /home/ubuntu/lio_sam_localization_ws/devel/setup.bash
```

2. Run the mapping launch file:
```
roslaunch lio_sam_localization run.launch config_file:="/home/ubuntu/lio_sam_localization_ws/src/LIO-SAM-Localization/config/params_gem.yaml"
```

3. To visualize against odom data, we need to publish transformation between world and map frame to get accurate comparison between ground truth odom data and estimated odom data: **OPTIONAL**
```
rosrun tf2_ros static_transform_publisher 22.748378703042732 -1.1095682571420336 -0.10003287520306003 3.526007880438855e-07 1.0449289132344871e-05 -0.006435430963485527 0.9999792923450977 world map
```

4. Play existing bag files:
```
rosbag play /home/ubuntu/testVolume-1/highbay_track-5-minutes-highres_2024-05-20-13-53-11.bag --start 115
```

5. Once the bag file ends, save the map:
```
rosservice call /lio_sam/save_map 0.2 "/home/ubuntu/testVolume-1/<sample-map-dir-name>/"
```

* ### To run localization on pre-built map using bag file

1. Source the required setup.bash
```
source /home/ubuntu/lio_sam_localization_ws/devel/setup.bash
```

2. Run the mapping launch file after editing *loadMapFileDir:="/home/ubuntu/testVolume-1/sample-map-dir-name/"* in params file:
```
roslaunch lio_sam_localization run_loc.launch config_file:="/home/ubuntu/lio_sam_localization_ws/src/LIO-SAM-Localization/config/params_gem.yaml" 
```
3. To visualize against odom data, we need to publish transformation between world and map frame to get accurate comparison between ground truth odom data and estimated odom data: **OPTIONAL**
```
rosrun tf2_ros static_transform_publisher 22.748378703042732 -1.1095682571420336 -0.10003287520306003 3.526007880438855e-07 1.0449289132344871e-05 -0.006435430963485527 0.9999792923450977 world map
```

4. Play existing bag files:
```
rosbag play /home/ubuntu/testVolume-1/highbay_track-5-minutes-highres_2024-05-20-13-53-11.bag --start 115
```

### Voila! the robot is localizing itself in known environment

Please note that the transformation between world and map frame needs to be initialized by user for now. (This will be improved in future)

## Run the Gazebo simulation of gem robot
1. In a new terminal source the required setup.bash
```
source /home/ubuntu/gem_ws/devel/setup.bash
```

2. You can edit the gem.gazebo file to change the **update_rate** of the **imu** or **params** for **velodyne** in **gem.urdf.xacro** and see impact on performance. **OPTIONAL**

3. Run the gem_gazebo_rviz launch file:
```
roslaunch gem_gazebo gem_gazebo_rviz.launch world_name:=./worlds/highbay_track.world x:=0 y:=0 velodyne_points:="true"
```

4. Transformation to be published between velodyne and gem/velodyne frame to complete the tf_tree so that we can visualize on rviz
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 /velodyne gem/velodyne
```

Please note that while running with gem_gazebo, we need to take care not to be stationary for too long after starting mapping, or else we will see drifts.
## Service
  - /lio_sam/save_map
    - save map as a PCD file.
      ``` bash
        rosservice call [service] [resolution] [destination]
      ```
      - Example:
      ``` bash
        $ rosservice call /lio_sam/save_map 0.2 "/Downloads/LOAM/"
      ```

## Issues

  - **Zigzag or jerking behavior in Localization**: If initial position while running localization is not near origin, then relocalization does not work well , resulting in zigzag jumps and Large Velocity error.

## Paper

Most of the code is adapted from [LIO-SAM (IROS-2020)](./config/doc/paper.pdf) [Github Repo](https://github.com/TixiaoShan/LIO-SAM).
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

## Acknowledgement

  - LIO-SAM-Localization is based on LIO-SAM (Tixiao Shan, Brendan Englot, Drew Meyers, Wei Wang, Carlo Ratti, and Daniela Rus. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping).
