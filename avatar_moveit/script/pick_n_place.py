#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import math
import moveit_msgs.msg
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

global X, Y, Z, roll, pitch, yaw, z_add, x, y, z

#ARM PARAMETERS
X = 0
Y = 0.45
Z = float(sys.argv[3])
roll = 0
pitch = pi/2
yaw = pi/2

z_add = 0.535
#GLOBAL PARAMETERS
x = float(sys.argv[1])
y = float(sys.argv[2])
z=Z
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    global pitch
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()

    if abs(diff_direction) < 10 :
        pose_goal.position.y = -Y
        pitch =  3*pi/2
    elif abs(diff_direction) > 170 and abs(diff_direction) < 190 :
        pose_goal.position.y = Y
    else :
        rospy.loginfo("TARGET TOO CLOSE")
        pose_goal.position.y = - Y
        pitch =  3*pi/2

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = X

    pose_goal.position.z = Z

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    i = 0
    while i < 5 :
        wpose.position.z +=  scale* z_add
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * z_add  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))
        i += 1


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


def reach(x, y, z, pace):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    #GETTING PLATFORM POSITION
    rospy.wait_for_service ('/gazebo/get_link_state')
    get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    platform = GetLinkStateRequest()
    platform.link_name='platform'

    vel = Twist ()
    vel.linear.z = 0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0

    platform_position = get_link_srv(platform)
    plat_x = platform_position.link_state.pose.position.x
    plat_y = platform_position.link_state.pose.position.y
    quaternion = (
        platform_position.link_state.pose.orientation.x,
        platform_position.link_state.pose.orientation.y,
        platform_position.link_state.pose.orientation.z,
        platform_position.link_state.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    plat_theta = euler[2]
    dist_vector = [ x - plat_x , y - plat_y ]
    dist_vec_module = math.sqrt(dist_vector[0] ** 2 + dist_vector[1] ** 2  )
    dist_vector_theta = math.atan2(dist_vector[1],dist_vector[0])
    theta_to_rotate = -plat_theta + dist_vector_theta -pi/2
    print "-----------------------------------"
    print "   theta to rotate = " + str(round(theta_to_rotate*180/pi,4))
    print "-----------------------------------"
    if theta_to_rotate > pi :
        theta_to_rotate = theta_to_rotate - pi
    elif theta_to_rotate < -pi :
        theta_to_rotate = theta_to_rotate + pi
    if theta_to_rotate > pi/2 and theta_to_rotate < pi:
        theta_to_rotate = pi - theta_to_rotate
    elif theta_to_rotate < -pi/2 and theta_to_rotate > -pi:
        theta_to_rotate = pi + theta_to_rotate


    vel.angular.z = theta_to_rotate * pace / (dist_vec_module - 0.45)
    counter = 0
    am_i_close = False
    r = rospy.Rate(100)
    while not am_i_close and not rospy.is_shutdown() :
        platform_position = get_link_srv(platform)
        plat_x = platform_position.link_state.pose.position.x
        plat_y = platform_position.link_state.pose.position.y
        quaternion = (
            platform_position.link_state.pose.orientation.x,
            platform_position.link_state.pose.orientation.y,
            platform_position.link_state.pose.orientation.z,
            platform_position.link_state.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        plat_theta = euler[2]
        if counter % 10 == 0 :
            print "-----------------------------------"
            print "   x = " + str(round(plat_x,2))
            print "   y = " + str(round(plat_y,2))
            print "   th = " + str(round(plat_theta,2))

        dist_vector = [ x - plat_x , y - plat_y ]
        dist_vec_module = math.sqrt(dist_vector[0] ** 2 + dist_vector[1] ** 2  )
        dist_vector[0] = dist_vector[0]/dist_vec_module
        dist_vector[1] = dist_vector[1]/dist_vec_module
        if dist_vec_module > 0.45 :
            vel.linear.x = pace*(dist_vector[0]*math.cos(plat_theta) + dist_vector[1]*math.sin(plat_theta))
            vel.linear.y = pace*(-dist_vector[0]*math.sin(plat_theta) + dist_vector[1]*math.cos(plat_theta))
            pub.publish(vel)
        else :
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            pub.publish(vel)
            am_i_close = True
            break
        counter += 1
        r.sleep()
def reach_v2(x, y, z, pace, rot_pace):
    global diff_direction
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    #GETTING PLATFORM POSITION
    rospy.wait_for_service ('/gazebo/get_link_state')
    get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    platform = GetLinkStateRequest()
    platform.link_name='platform'

    vel = Twist ()
    vel.linear.z = 0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0

    platform_position = get_link_srv(platform)
    plat_x = platform_position.link_state.pose.position.x
    plat_y = platform_position.link_state.pose.position.y
    quaternion = (
        platform_position.link_state.pose.orientation.x,
        platform_position.link_state.pose.orientation.y,
        platform_position.link_state.pose.orientation.z,
        platform_position.link_state.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    plat_theta = euler[2]
    dist_vector = [ x - plat_x , y - plat_y ]
    dist_vec_module = math.sqrt(dist_vector[0] ** 2 + dist_vector[1] ** 2  )
    dist_vector_theta = math.atan2(dist_vector[1],dist_vector[0])

    #RUOTO SUL POSTO
    if dist_vec_module > 0.5 :
        too_close = False
        theta_to_rotate = -plat_theta + dist_vector_theta -pi/2
        print "-----------------------------------"
        print "    theta to rotate = " + str(round(theta_to_rotate*180/pi,1))
        print "-----------------------------------"
        if theta_to_rotate > pi :
            theta_to_rotate = theta_to_rotate - pi
        elif theta_to_rotate < -pi :
            theta_to_rotate = theta_to_rotate + pi
        if theta_to_rotate > pi/2 and theta_to_rotate < pi:
            theta_to_rotate = pi - theta_to_rotate
        elif theta_to_rotate < -pi/2 and theta_to_rotate > -pi:
            theta_to_rotate = pi + theta_to_rotate

        if theta_to_rotate < 0 :
            theta_to_rotate = pi/2 + theta_to_rotate
            vel.angular.z = rot_pace
        else :
            theta_to_rotate = -pi/2 + theta_to_rotate
            vel.angular.z = -rot_pace

        vel.linear.x = 0
        vel.linear.y = 0

        t = 0
        tic = rospy.Time.now()
        toc = rospy.Time.now() - tic
        r = rospy.Rate(100)
        while t < abs((theta_to_rotate)/rot_pace):
            toc = rospy.Time.now() - tic
            t = (toc.secs * (10 ** 9) + toc.nsecs) / (10 ** 9 * 1.0000)
            pub.publish(vel)
            r.sleep()

        vel.angular.z = 0
        pub.publish(vel)
    else :
        too_close = True

    # MI AVVICINO AL TARGET
    rospy.sleep(0.2)
    counter = 0
    am_i_close = False
    r = rospy.Rate(100)
    while not am_i_close and not rospy.is_shutdown() :
        platform_position = get_link_srv(platform)
        plat_x = platform_position.link_state.pose.position.x
        plat_y = platform_position.link_state.pose.position.y
        quaternion = (
            platform_position.link_state.pose.orientation.x,
            platform_position.link_state.pose.orientation.y,
            platform_position.link_state.pose.orientation.z,
            platform_position.link_state.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        plat_theta = euler[2]
        if counter % 50 == 0 :
            print "-----------------------------------"
            print "   x = " + str(round(plat_x,2))
            print "   y = " + str(round(plat_y,2))
            print "   th = " + str(round(plat_theta,2))

        dist_vector = [ x - plat_x , y - plat_y ]
        dist_vec_module = math.sqrt(dist_vector[0] ** 2 + dist_vector[1] ** 2  )
        dist_vector[0] = dist_vector[0]/dist_vec_module
        dist_vector[1] = dist_vector[1]/dist_vec_module
        if dist_vec_module > 0.5 :
            vel.linear.x = pace*(dist_vector[0]*math.cos(plat_theta) + dist_vector[1]*math.sin(plat_theta))
            vel.linear.y = pace*(-dist_vector[0]*math.sin(plat_theta) + dist_vector[1]*math.cos(plat_theta))
            pub.publish(vel)
        else :
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            pub.publish(vel)
            am_i_close = True
            break
        counter += 1
        r.sleep()

    diff_direction = (plat_theta - dist_vector_theta)*180/pi
    print "   diff =" + str(diff_direction)
    #RUOTO NUOVAMENTE
    if not too_close :
        t = 0
        vel.angular.z = rot_pace
        tic = rospy.Time.now()
        toc = rospy.Time.now() - tic
        r = rospy.Rate(100)

        while t < (pi/(2*rot_pace)):
            toc = rospy.Time.now() - tic
            t = (toc.secs * (10 ** 9) + toc.nsecs) / (10 ** 9 * 1.0000)
            pub.publish(vel)
            r.sleep()

        vel.angular.z = 0
        pub.publish(vel)

        platform_position = get_link_srv(platform)
        quaternion = (
            platform_position.link_state.pose.orientation.x,
            platform_position.link_state.pose.orientation.y,
            platform_position.link_state.pose.orientation.z,
            platform_position.link_state.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        plat_theta = euler[2]


def main():
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
  try:
    print ""
    print "----------------------------------------"
    print "               Welcome"
    print "----------------------------------------"

    move_ARM = MoveGroupPythonIntefaceTutorial()

    pace = 0.3
    rot_pace = 0.55
    reach_v2(x, y, z, pace, rot_pace)
    move_ARM.go_to_pose_goal()
    #
    #
    # # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # # raw_input()
    # # tutorial.display_trajectory(cartesian_plan)
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()
    #
    # tutorial.execute_plan(cartesian_plan)
    #
    #
    #
    # print "============ FINISHED!"
    #
    # vel = Twist ()
    # vel.linear.x = 0.0
    # vel.linear.y = 0.0
    # vel.linear.z = 0.0
    # vel.angular.x = 0.0
    # vel.angular.y = 0.0
    # vel.angular.z = 0.0
    # pub.publish(vel)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
