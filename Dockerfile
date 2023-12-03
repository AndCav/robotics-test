# The solution works either way, but with this version, the system is unable to find the 'chomp-motion-planner' libraries.
# Although this does not necessarily impact the test solution, I opted for a clean installation."


# FROM ghcr.io/remyrobotics/robotics-test:latest

# RUN apt-get update && apt-get install -y  --fix-missing \
#     python3-catkin-tools \
#     ros-noetic-chomp-motion-planner \
#     ros-noetic-moveit

# RUN rosdep update

# COPY catkin_ws/src/solution_robotics_test /home/catkin_ws/src/solution_robotics_test

# RUN . /opt/ros/noetic/setup.sh \
#     && apt-get update \
#     && cd /home/catkin_ws \
#     && rosdep install --from-path . -y --ignore-src \
#     && catkin build
# -------------------------------------------------------------------------------------
 
FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    gazebo11 \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-moveit

RUN rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

COPY catkin_ws/src /home/catkin_ws/src
COPY catkin_ws/src/simple_scene/worlds /usr/share/gazebo-11/worlds
COPY catkin_ws/src/simple_scene/models /root/.gazebo/models

RUN . /opt/ros/noetic/setup.sh \
    && apt-get update \
    && cd /home/catkin_ws \
    && rosdep install --rosdistro=noetic --from-path . -y --ignore-src \
    && catkin build

