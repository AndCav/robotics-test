# Robotics Software Engineer Task

You are asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Goal

The scope of this assignment is limited to the robot control part of a continuous pick-and-place cycle. There are two tables; one with 5 blocks and one empty. The goal is to move all of the blocks onto the other table.

No vision/perception is needed. Poses of the blocks are accessible through the ros_service:

`/gazebo/get_model_state`

The gripper can be controlled through a topic:

`/gripper_joint_position/command`

## Build && Run

### Using docker image

```bash
$ docker pull ghcr.io/andcav/robotics-test:latest
$ xhost local:root
$ docker-compose up
```

Alternatively, you can build manually with the given Dockerfile.

### Building from Source

```bash
$ ./build.sh
$ cd catkin_ws
$ catkin build
$ source devel/setup.bash
$ roslaunch test_pick_and_place ur5_gr2_moveit_stack.launch
```

## Evaluation

You can use any 3rd party package. We would like to see how you design the system and the planning & control pipeline.

## Submission

A docker image and the source code are required. Please explain your design choices. If it is not possible due to circumstances, please send us source codes with clear instructions and explanations.

## Additional Questions

- How would you improve the efficiency and reliability?
- What would you do if we needed multiple robots in a system?
- How would you deploy the application to multiple physical locations? What is needed to scale it?
