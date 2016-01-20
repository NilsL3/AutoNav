#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  double roll, pitch, yaw;

  ros::Publisher drone_pose = node.advertise<geometry_msgs::Twist>("/pose", 1000);

  tf::TransformListener listener;

  ros::Rate rate(30.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      //ros::Time now = ros::Time::now();
      ros::Time now = ros::Time(0);
      listener.waitForTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera", now, ros::Duration(3.0));
      listener.lookupTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera", now, transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("failure %s\n",ex.what());
      //ros::Duration(1.0).sleep();
      continue;
    }

    tf::Matrix3x3(transform.getRotation()).getRPY(pitch, yaw, roll);
    
    geometry_msgs::Twist pose_msg;
    pose_msg.linear.x = transform.getOrigin().z();
    pose_msg.linear.y = transform.getOrigin().x()*(-1);
    pose_msg.linear.z = transform.getOrigin().y()*(-1);
	pose_msg.angular.x = roll;	
	pose_msg.angular.y = pitch;	
	pose_msg.angular.z = yaw;
	           
    drone_pose.publish(pose_msg);
	
    rate.sleep();
  }
  return 0;
};

