hide-n-seek-ros-pkg
===================

A ROS package that demonstrates something resembling a POMDP.

## Summary 
* Author: Josh Villbrandt <josh@javconcepts.com>, Divya Sharma <divyasha@usc.edu>, Prithvi Balaram <balaram@usc.edu>
* Date: Spring 2011
* Class: USC CS545 (http://robotics.usc.edu/~aatrash/cs545/)

## Description
This package enables a robot to play a classic game of Hide and Seek. During the seeking portion of the game, the robot uses the POMDP implemented in this package to guide itself through a space in search of a person.

## Implementation Details
See final report PDF.

## Installation / Usage Preparation
    # install, setup ROS
    sudo apt-get install ros-electric-pr2-desktop
    # add the following lines to your ~/.bashrc
    source /opt/ros/electric/setup.bash
    export ROS_PACKAGE_PATH=~/ros:/opt/ros/electric/stacks:$ROS_PACKAGE_PATH
    export ROBOT=sim
    # checkout, make code
    cd ~/ros
    svn co http://svn.javconcepts.com/cs545/trunk/hide_n_seek
    rosmake hide_n_seek
    # to edit code
    roscd hide_n_seek
    make eclipse-project
    # for manual PR2 control
    roslaunch pr2_teleop teleop_keyboard.launch

## Usage [sim]
    # terminal 1
    roscore
    # terminal 2
    roslaunch hide_n_seek custom_pr2_sim.launch
    # terminal 3
    rosrun hide_n_seek fake_people_finder
    # terminal 4
    rosrun hide_n_seek planner

## Revision History
4/28/2012 - turned in for final project
