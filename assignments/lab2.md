# Lab 2: Lerobot Ex ROS

Lerobot is an incredible system for quickly training VLA models.  
However, it directly occupies the camera and motor ports, which disables rapid switching to other control systems.  
If you want to use lerobot with "classical" ros-based controls, you first have to disable lerobot.  
This causes the motors to lose power and collapse.  
In this lab, we will use the [rosetta](https://github.com/iblnkn/rosetta) group of ros packages to run lerobot policies through ros2_control to control the arm(s).  
This will allow us to switch between multiple policies and classical control systems seamlessly.


## Learning Objectives

- Inspect the output of lerobot policies to understand how they control the robot
- Convert the output of lerobot policies into ros2_control signals
- Run lerobot policies on the physical robot using ros2_control


## TODO

1. Complete the rosetta contract for the SO101 robot arm in `soa_ros2/soa_rosetta/contracts`

2. Run your lerobot policy from lab 1 on a physical arm using ROS and the rosetta package.
    In one terminal, bring-up your robot.  
    In another terminal, start your action_node_relay.  
    In a third terminal, start the rosetta policy server.  
    Read through the [deploying policies section](/home/ubuntu/soa_dev/soa_ws/src/soa_ros2/soa_moveit_config) of the rosetta documentation to understand how to use the policy server.  
    In a fourth terminal, send a goal to the policy server to complete the action you trained on.


## Deliverables

1. Copy your rosetta contract into your lab report.

2. Link a video of your lerobot policy running through ros2.  
    Show your ros2 terminal and the robot running autonomously.


## FAQ


## Resources

[Lerobot async inference server documentation](https://huggingface.co/docs/lerobot/async)  
Rosetta runs its policy server using lerobot's built-in server system.  
This could theoretically allow you run the policy on a separate powerful computer if you wanted to deploy the arms (or another robot) as a mobile system with less computing power.  

[Structured VLA reading list by MilkClouds on Github](https://github.com/MilkClouds/awesome-vla-study)  
Compilation of resources for learning all about different Vision Language Action models.