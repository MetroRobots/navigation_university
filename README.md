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
