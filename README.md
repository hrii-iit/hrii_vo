
# HRII Visual Odometry (HRII-VO)

This repository provides tools for visual odometry experiments and teleoperation of mobile collaborative robots (MCR). The system supports multiple cameras and teleoperation interfaces, including setups involving Raspberry Pi 4 for wireless communication. 

## Overview

The HRII-VO framework is designed for conducting experiments with RGB-D and stereo cameras, focusing on visual odometry and remote teleoperation. It integrates **rtabmap_ros** for SLAM and visual-inertial odometry (VIO) algorithms, with additional configurations for ZED2i and RealSense D435i cameras.

A GIF demo (`0122_VD.gif`) illustrating the system's operation is included in the main repository.

## Key Features

- UDP-based image transfer protocol for efficient communication.
- Support for wired and wireless camera configurations.
- Integration of VIO with **rtabmap_ros** for teleoperation.
- Compatibility with both stereo and RGB-D cameras.
- Experiments for odometry accuracy, comparison, and data logging.

## Installation

1. Import repositories:
   ```bash
   git_import_repos vo_repos.yaml
   cdcb
   ```

2. Install **rtabmap_ros**:
   Follow the installation guide at [rtabmap_ros GitHub](https://github.com/introlab/rtabmap_ros).

3. For ZED2i camera support:
   - Install CUDA.
   - Follow the setup instructions in [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper).
   - Add `zed-ros-wrapper` to your Catkin workspace.

## Usage

Launch different experiments and setups using the provided launch files:

1. **Franka Motion:**
   ```bash
   roslaunch hrii_vo vo_comparison_experiment.launch
   ```

2. **Odometry Test (RGB-D and ZED2i Cameras):**
   ```bash
   roslaunch hrii_vo All_exp_VO.launch
   ```

3. **ZED2i Odometry Test with RTAB-Map:**
   ```bash
   roslaunch hrii_vo zed_test.launch
   ```

4. **RealSense D435i Odometry Test with RTAB-Map:**
   ```bash
   roslaunch hrii_vo realsense_test.launch
   ```

5. **Data Logging:**
   ```bash
   roslaunch hrii_vo visual_connector.launch
   ```

### Note:
If the system cannot handle the `All_exp_VO` launch file, you can launch the components individually:
```bash
roslaunch hrii_vo realsense_test.launch
roslaunch hrii_vo zed_test.launch
roslaunch hrii_vo visual_connector.launch
```

## Teleoperation with Raspberry Pi 4

For Raspberry Pi 4 setups (running Raspbian OS), execute the scripts located in the `rpi_files` folder. Ensure that IP addresses and communication settings are correctly configured for UDP-based image transfer.

## Experimental Scenarios

- **Odometry Comparison:** Run tests with both stereo and RGB-D cameras to compare odometry accuracy.
- **Teleoperation Setup:** Evaluate VIO performance using ZED2i and RealSense cameras in wired and wireless configurations.
- **Home-Care Application:** Utilize the VIO-based teleoperation interface in real-world scenarios.

## References

The repository includes source code for the experiments, accessible at [GitHub HRII-VO](https://github.com/hrii-iit/hrii_vo).

## Publications

This work is based on the research presented in *A Multipurpose Interface for Close- and Far-Proximity Control of Mobile Collaborative Robots* ([PDF](A_Multipurpose_Interface_for_Close-_and_Far-Proximity_Control_of_Mobile_Collaborative_Robots.pdf)).

---

Contributions and feedback are welcome to improve this repository.
