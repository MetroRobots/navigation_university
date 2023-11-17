![Navigation University Logo](https://navigationuniversity.com/Logo.png)

See [navigationuniversity.com](https://navigationuniversity.com) for more details.

Code to support the Navigation University workshop.

# Software Setup Instructions
## Setup Up Your Folder Structure / Clone Repos
 * You can use your own `colcon` workspace, but you may want to create a clean one for this workshop.

 * If you are creating one, run `mkdir ~/ros2_ws/src -p`
 * `cd ~/ros2_ws`
 * `wget https://raw.githubusercontent.com/MetroRobots/navigation_university/main/repos.yaml`
 * `vcs import --recursive < repos.yaml`
 * You may also want to create a new empty git repo for the new robot you'll create.
  * `mkdir ~/ros2_ws/src/my_new_robot`
  * `cd ~/ros2_ws/src/my_new_robot`
  * `git init`

## Install More Dependencies
 * `cd ~/ros2_ws`,
 * `rosdep install -r --from-paths src --ignore-src --rosdistro iron -y`

## Build the ROS Workspace
 * `source /opt/ros/iron/setup.bash`
 * `colcon build --symlink-install`

# Curriculum

Presentations for the Navigation University workshop.

 1. [00 - Intro](https://docs.google.com/presentation/d/1Ix0YbG5T3BNs6E0aIlGHWx-LOWcSm6gAooYLkKlC7aA/edit?usp=sharing)
 2. [01 - Teleoperation](https://docs.google.com/presentation/d/1n13MWDcnPihA-IBY9qn0Eyqg2voVTSz69Dk_iipQsgU/edit?usp=sharing)
 3. [02 - Mapping](https://docs.google.com/presentation/d/1ZiZhw7uswBVzEkDrTCOjHh_HMbA6Duw5_YbPt8leqtY/edit?usp=sharing)
 4. [03 - Global Planning](https://docs.google.com/presentation/d/1P86WW4Zh_Xr57MBmwCfGA0vgjo_maeoSe70MJrYjXWM/edit?usp=sharing)
 5. [04 - Localization](https://docs.google.com/presentation/d/18TA7NGiPHXQrzqErOisxQS8GZxMOid9798djV9Hf7fk/edit?usp=sharing)
 6. [05 - Costmaps](https://docs.google.com/presentation/d/1sxIqtTtSlSyvCpn6x0fwloD2D-W_K8swfpHEGsYEBLk/edit?usp=sharing)
 7. [06 - Local Planning](https://docs.google.com/presentation/d/1kXSle8oyF-ooqYNL298knn994adwgV2_cCK4lpZnGho/edit?usp=sharing)
 8. [07 - High Level Coordination](https://docs.google.com/presentation/d/1txxOIOTduF_LYAMh2NWUG2syBrtXM6LIyLhOILcFwIs/edit?usp=sharing)
