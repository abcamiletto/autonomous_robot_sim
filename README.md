# Robot Mechanics Project
This is a Gazebo simulation of a mobile autonomous robot

## Install
You need to install the libfranka library. You can find the instruction here: https://frankaemika.github.io/docs/installation_linux.html
It may be better to build it from source, i got problem with the single command line you can find in the tutorial.

After that, just clone this repository in a src folder and make sure the `franka_ros` submodule is correctly downloaded, and in the right franka_ros folder. If there is any error, you can download it from here: https://github.com/frankaemika/franka_ros

## Task 1: Pick and Place
The first thing to do is to make sure we are in the right folder and we sourced the workspace so:
`
cd your_path/mecrob_ws
source devel/setup.sh
`

Launch Gazebo

`
roslaunch avatar_moveit avatar_gazebo_empty.launch 
`

Then press on start in the Gazebo GUI.
Now launch RViz in a new terminal

`
roslaunch avatar_moveit avatar_moveit.launch 
`

So now you should have: 
![Screenshot1](https://github.com/abcamiletto/mecrob_project/blob/master/images/Screenshot%20from%202020-03-01%2018-45-05.png?raw=true)

To start the task you have to open a new terminal and type
`
rosrun avatar_moveit pick_n_place.py X Y Z
`
Where X,Y,Z are the coordinates of the point in the space you want to reach.
You should get something like:

![Gif1](https://github.com/abcamiletto/mecrob_project/blob/master/images/ezgif.com-video-to-gif.gif?raw=true)

## Task 2: Paint a cylinder
Be sure you are in the right folder as before and then:

Launch Gazebo and then RViz

`
roslaunch avatar_moveit avatar_gazebo.launch 
roslaunch avatar_moveit avatar_moveit.launch 
`

Now just launch the script 2
`
rosrun avatar_moveit cylinder_v1.py
`
And you should get:
![Gif2](https://github.com/abcamiletto/mecrob_project/blob/master/images/ezgif.com-video-to-gif%20(1).gif?raw=true)
