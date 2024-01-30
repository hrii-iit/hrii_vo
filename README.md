# hrii_vo
Visual odometry repository.

### Installation
```bash
git_import_repos vo_repos.yaml
cdcb


rtabmap_ros installation is required. Follow https://github.com/introlab/rtabmap_ros


for zed2i running, cuda is required. Follow the guide on, https://github.com/stereolabs/zed-ros-wrapper and add zed-ros-wrapper to your catkin workspace.


```

### Usage
Launch the experiment by running:
```bash
roslaunch hrii_vo vo_comparison_experiment.launch   #franka_motion

roslaunch hrii_vo All_exp_VO.launch    ##used to run rgbd and zed2i camera for odometry test
roslaunch hrii_vo zed_test.launch    ##Odom test using rtabmap for zed2i

roslaunch hrii_vo realsense_test.launch   ##Odom test using rtabmap for realsense d435i


roslaunch hrii_vo visual_connector.launch  ##publishes topic realted to logging the data


In case the system could not handle All_exp_VO launch,
you should launch them one by one:
1. realsense_test
2. realsense_test
3. visual_connector
```
