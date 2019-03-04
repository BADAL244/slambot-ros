# 2D Autonomous Mapping

## Development setup

0. Install ROS and choose a folder for your catkin workspace.
    - This folder will have several folders in it, including this repo as its
      `src` directory.
1. Clone this repository **with submodules** into the `<workspace>/src`
   directory:
    - `cd <workspace>/`
    - `git clone --recursive <url> src`
2. Initialize a Catkin workspace in the `src` directory:
    - `cd src/`
    - `catkin_init_workspace`
3. Build with catkin:
    - `cd ..` (back into the workspace top-level)
    - `catkin_make`
4. Source the development environment setup script:
    - `source devel/setup.sh` (or equivalent for your shell)
    - You may want to add this to your shell login scripts, so that the
      environment is set up automatically when you open your shell.
5. You can now try the demo using:
    - `roslaunch rplidar_ros view_rplidar.launch`
    - cf. https://blog.zhaw.ch/icclab/rplidar/

