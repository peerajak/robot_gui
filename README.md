# Robot GUI
There are two ROS nodes  

## distance Tracker Service node

This node subscribes to a /odom topic, calculates the distance traveled by the robot using the odometry data, 
and provides a service server of type std_srvs/Trigger to send a message with the distance traveled on request. 
The service name is /get_distance

## ros_gui_node

This is GUI node for ROS

## Important topic
1. /cmd_vel. ros_gui node publish to this topic to control the robot with GUI teleop buttons.
2. /odom     distance Tracker Service node use this odometry data to calculate its distance traveled
3. /robot_info  ros_gui node need robot_info topic provided by robot_info depository to show up on GUI.

## Important service
1. /get_distance distance Tracker Service node provides this service for ros_gui node to get robot traveled distance.

To use this GUI:
- Terminal 1
Launch Gazebo with robot
'''roslaunch mir_gazebo mir_maze_world.launch'''
and wait for gazebo to launch 

- Terminal 2
To play the robot in gazebo
'''rosservice call /gazebo/unpause_physics'''


- Terminal 3
To provide /robot_info topic
'''rosrun robot_info agv_robot_info_node'''

- Terminal 4
To provide distance tracker service
'''rosrun robot_gui robot_gui_distance_travelled_service_node'''

- Terminal 5
To provide GUI
'''rosrun robot_gui robot_gui_node'''