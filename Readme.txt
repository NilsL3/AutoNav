This is a ROS package for the autonomous flight of the Parrot AR Drone 2.0 using  ORB-SLAM (Raúl Mur-Artal, 2015) for the position and yaw estimation of the quadrocopter. This controller regulates, the x position, y position and yaw.

1. Related paper:
J. Martinez-Carranza, Nils Lowen, F. Marquez, E.O. García Rodriguez, W. Mayol-Cuevas. Towards Autonomous Flight of Micro Aerial Vehicles using ORB-SLAM. IEEE 3rd Workshop on Research, Education and Development of Unmanned Aerial Systems. RED-UAS 2015. Cancún, México. November, 2015.

Video:
https://www.youtube.com/watch?v=Gl0BN86T7Tw

2. Installation

Tested under Ubuntu 14.04, using ROS Indigo. 

Required packages:
Ardrone Autonomy (developed by Willow Garage), the ROS API for controlling the AR Drone 2.0. and receiving sensor data and video
stream.
Ardrone_tutorials by Mike Hamer. This package includes code for flying the AR Drone 2.0. with the keyboard.
ORB-SLAM (Raúl Mur-Artal, 2015). The specific branch I used was bmagyar/ORB_SLAM.

 ArdroneFrontSettings.yaml in the "Settings" folder should be pasted in ORB_SLAM/orb_slam/Data and the ORB_SLAM/orb_slam/launch/orb_slam.launch should be replaced with the orb_slam.launch found in the Settings folder. These changes shall remap the 
front camera topic of the AR Drone 2.0 to the input of ORB-SLAM. 
The controller package and the tf_pose package should be left in catkin_ws/src.  

3. Usage

Start by plugging in the AR Drone's battery. Once the network appears on your computer, connect to it.

3.1. Find out the position of your desired waypoint according to the ORB-SLAM map.

Run roscore.
Roslaunch keyboard_controller.launch (located in the ardrone_tutorials package). A window containing the real time videostream from the front camera of the AR Drone should appear. When selecting this window the AR Drone can be controlled from the keyboard.
Roslaunch orb_slam.launch. This will open up the tracking window and Rviz from ORB_SLAM for displaying the sparse pointcloud and current pose of the camera.
Takeoff the AR Drone with the 'Y' key or apply a translational movement in order to initialize ORB-SLAM.
Next, rosrun tf_pose tf_pose. This node publishes the /pose topic, which contains the x,y and z position of the 
quadcopter and also its roll, pitch and yaw. The x axis is the front/back axis of the quadcopter, the y axis is the left/right axis of the quadcopter and the z axis is the up/down axis.
Then listen to this topic by typing: rostopic echo /pose, and you will see the current position of the quadcopter displayed. 
Move the quadrotor manually or with the keyboard towards a waypoint position at which you would like it to land autonomously.
Record the x position of the /pose topic once the quadcopter is located at the desired waypoint position.Then return the
quadcopter to the place at which you initialized ORB-SLAM, which would be the origin.

3.2. Edit the waypointPlanner.yaml found inside the controller package.

Pick two intermediate x positions (waypoints) between the origin and the waypoint.  Note that the origin is the position and orientation at which ORB-SLAM was initialized. Edit the pit_ref array. THe array should always start with 0.0 and then the following x positions (waypoints). Example: 
pit_ref: [0.0, 0.200, 0.400, 0.600]
At each of the waypoints the quadcopter hovers for the amount of cycles set in the resetStep parameter. 
The roll_ref is the Y position setpoint. If set to 0.0 the quadcopter shall fly straight.
The yaw_ref is the yaw setpoint. It is set to 0.0 in order to maintain the quadcopter's heading towards the waypoint
straight ahead. 
The controllers managing the x and y position are both PD controllers, the gains of the x position controller are
pit_Kp and conPit_Kd and the gains of the y position controller are roll_Kp and conRoll_Kd. The yaw controller is
a proportional controller and the gain is set with the parameter yaw_Kp. THe controllers need to be tuned experimentally 
at each new ORB-SLAM initialization because the map scale depends on the size of the environment ORB-SLAM is initialized in.

3.3. Run the autonomous flight. 

Once the waypointPlanner.yaml file has been configured, use the 'Y' key to takeoff the AR Drone. Then type the instruction
roslaunch navcontrol.launch. The AR Drone shall fly through the waypoints listed in the waypointPlanner. Note that the waypoint shall be reached once the x position of the quadcopter is within a certain threshold of the waypoint. The threshold is set 
in the waypointPlanner.yaml file. Once the AR Drone reaches the last waypoint, the quadcopter lands autonomously. 





