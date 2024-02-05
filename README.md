# hrii_vo
Visual odometry repository.

### Installation
```bash
git_import_repos vo_repos.yaml
cdcb


rtabmap_ros installation is required. Follow https://github.com/introlab/rtabmap_ros


for zed2i running, cuda is required. Follow the guide on, https://github.com/stereolabs/zed-ros-wrapper and add zed-ros-wrapper to your catkin workspace.


```

### Usage:
```bash
''' Examine the contents of the "teleop_interface_all.launch" launch file and make necessary modifications based on your specific requirements and the designated topics for manipulation and locomotion commands. For instances where a Raspberry Pi 4 is employed, execute two scripts located in the "rpi_files" folder on the RPI, operating within the Raspbian OS.

It is essential to note that UDP is the chosen protocol for image transfer, and you must configure your communication settings, taking into account the relevant IP addresses.

Other necessary launch files are discussed below for other cameras.

'''
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
