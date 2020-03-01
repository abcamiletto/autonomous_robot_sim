#!/bin/sh
roslaunch avatar_moveit avatar_gazebo.launch &
sleep 8 &
rosservice call gazebo/unpause_physics &
roslaunch avatar_moveiavatar_moveit.launch &
wait

