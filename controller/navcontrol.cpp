#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
using namespace std;

class pid_control
{

public:	
	pid_control(){
		
		wpx1=0.0;
		wpx2=0.0;
		wpx3=0.0;
		
		wpy1=0.0;
		wpy2=0.0;
		wpy3=0.0;
		
		i = 1;
		
		resetStep = 0;	
		deltaT = 1.0/30.0;	
		
		
		pit_lastErr = 0; 
		roll_lastErr = 0; 
		
		
        pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	    publand_ = n_.advertise<std_msgs::Empty>("ardrone/land", 1);
	    sub_ = n_.subscribe("/pose", 1, &pid_control::callback, this); 
	    
	    
	    n_.getParam("pit_ref", pit_ref);
	    n_.getParam("roll_ref", roll_ref);
  	    n_.getParam("pit_Kp", pit_Kp);
  	    //n_.getParam("conPit_Ki", conPit_Ki);
		n_.getParam("conPit_Kd", conPit_Kd);
	    n_.getParam("roll_Kp", roll_Kp);
	    n_.getParam("conRoll_Kd", conRoll_Kd);
	    n_.getParam("roll_ref", roll_ref);
	    n_.getParam("threshold", threshold);
	    n_.getParam("yaw_Kp", yaw_Kp);
	    n_.getParam("yaw_ref", yaw_ref);
	    n_.getParam("resetStep",resetStep);
	    
	    
	    rStep = resetStep;
	    
	    geometry_msgs::Twist msgOut; 
	  
	    msgOut.linear.x = 0.0;
		msgOut.linear.y =0.0;
		msgOut.linear.z =0.0;
		msgOut.angular.z =0.0;		
		
	    pub_.publish(msgOut);
	    
	    
	    pit_Kd = conPit_Kd/deltaT;	
	    //pit_Ki = conPit_Ki*deltaT;
		roll_Kd = conRoll_Kd/deltaT;
		
	}
	

	void callback(const geometry_msgs::Twist &msg); 
        
	std::vector<float> pit_ref;
	//std::vector<float> roll_ref;
	float wpx1, wpx2, wpx3;
	float wpy1, wpy2, wpy3;
	int i;
	
	//int waypoint_reached = 0;
	double resetStep;
	double rStep;
	float pit_dist, roll_dist;
	float deltaT;	
	double pit_Kp;
	double yaw_ref;
	
	//double conPit_Ki;
	
	double conPit_Kd;
	double conRoll_Kd;
	
	double roll_ref;
	
	double yaw_Kp;
	double roll_Kp;
	
	//double pit_Ki;
	
	double pit_Kd;	
	double roll_Kd;
	
	float pit_error_dot;	
	float roll_error_dot;	
	
	double threshold;
	//float pit_Ki = 0.1*deltaT;
	//float pit_Kd = 0.1/deltaT;	

	float pit_error;
	float pit_lastErr; 
	//float pit_addErr;
	
	float yaw_error;
	
	float roll_error;
	float roll_lastErr; 	
	
	ofstream myfile;
	
	private : 
	ros::NodeHandle n_; 
	ros::Publisher pub_; 
	ros::Subscriber sub_;
	ros::Publisher publand_;

};


void pid_control::callback(const geometry_msgs::Twist &msg){
	
	geometry_msgs::Twist msgOut; 
	std_msgs::Empty msgLand;
	
	double yawMax = M_PI*45.0/180.0;
	
	if(rStep > 0)
	{
		
				
		ROS_INFO_STREAM("output_pit x="<<msgOut.linear.x); 
		ROS_INFO_STREAM("output_pit y="<<msgOut.linear.y); 
		ROS_INFO_STREAM("output_pit z="<<msgOut.linear.z); 
		ROS_INFO_STREAM("output_yaw ="<<msgOut.angular.z); 	
		ROS_INFO_STREAM("=============== reset step ="<<rStep); 	
		
		
		rStep--;
		
		
		msgOut.linear.x = 0.0;
		msgOut.linear.y =0.0;
		msgOut.linear.z =0.0;
		msgOut.angular.z =0.0;		
				
		myfile.open ("data.txt", fstream::app|fstream::out);
	    myfile << i << '\t' << 0 << '\t' << 0;
	    myfile <<'\t' << 0 << '\t' << 0;
	    myfile <<'\t' << 0 << '\t' << 0;
	    myfile <<'\t' << 0 << '\n';
	    myfile.close();	
	    return;
	}
	
	pit_dist = fabs(pit_ref[i] - pit_ref[i-1]);
	
	pit_error = pit_ref[i] - msg.linear.x;
	pit_error_dot = pit_error - pit_lastErr;
		
	yaw_error = yaw_ref - msg.angular.z;
	
	roll_error= roll_ref - msg.linear.y;
	roll_error_dot = roll_error - roll_lastErr;
	
	
	msgOut.linear.x = pit_Kp * (pit_error / pit_dist) + pit_Kd * pit_error_dot;
	
	pit_lastErr = pit_error;
	
	msgOut.linear.y = roll_Kp * roll_error/ (3*pit_dist) + roll_Kd * roll_error_dot;
	if(msgOut.linear.y < -0.5)
		msgOut.linear.y = -0.5;
	if(msgOut.linear.y > 0.5)
		msgOut.linear.y = 0.5;
	
	roll_lastErr = roll_error;
	
	
	msgOut.linear.z = 0.0;
	msgOut.angular.x =1.0;
	msgOut.angular.y =1.0;
	//msgOut.angular.z =0.0;
	msgOut.angular.z = (-1.0) * yaw_Kp * (yaw_error / yawMax);
	
	if(msg.linear.x > (pit_ref[i]-threshold) && msg.linear.x < (pit_ref[i]+threshold)){
		rStep = resetStep;
		if(i==1){
		wpx1=msg.linear.x;
		wpy1=msg.linear.y;
		}

		if(i==2){
		wpx2=msg.linear.x;
		wpy2=msg.linear.y;
		}

		if(i==3){
		wpx3=msg.linear.x;
		wpy3=msg.linear.y;
		}
/*
		if(i==4){
		wpx4=msg.linear.x;
		wpy4=msg.linear.y;
		}
		if(i==5){
		wpx5=msg.linear.x;
		wpy5=msg.linear.y;
		}

		if(i==6){
		wpx6=msg.linear.x;
		wpy6=msg.linear.y;
		}
		if(i==7){
		wpx7=msg.linear.x;
		wpy7=msg.linear.y;
		}
*/
				
		i++;
		if(i == pit_ref.size()) publand_.publish(msgLand);
		
		}
	
	ROS_INFO_STREAM("pit_reference = "<<pit_ref[i]); 
	ROS_INFO_STREAM("pit ERROR = "<<pit_error);
	ROS_INFO_STREAM("OUT_pit = "<<msgOut.linear.x); 
	ROS_INFO_STREAM("--------------------------------"); 	
	ROS_INFO_STREAM("roll_ERROR = "<<roll_error);
	ROS_INFO_STREAM("OUT_roll ="<<msgOut.linear.y); 
	ROS_INFO_STREAM("--------------------------------"); 	
	ROS_INFO_STREAM("yaw_ERROR = "<<yaw_error);
	ROS_INFO_STREAM("OUT_yaw ="<<msgOut.angular.z); 	
	ROS_INFO_STREAM("--------------------------------"); 	
	
	ROS_INFO_STREAM("wpx1 ="<<wpx1);
	ROS_INFO_STREAM("wpy1 ="<<wpy1); 
	ROS_INFO_STREAM("wpx2 ="<<wpx2); 
	ROS_INFO_STREAM("wpy2 ="<<wpy2); 
	ROS_INFO_STREAM("wpx3 ="<<wpx3); 
	ROS_INFO_STREAM("wpy3 ="<<wpy3); 
	//ROS_INFO_STREAM("wpx4 ="<<wpx4); 
	//ROS_INFO_STREAM("wpy4 ="<<wpy4); 
	//ROS_INFO_STREAM("wpx5 ="<<wpx5); 
	//ROS_INFO_STREAM("wpy5 ="<<wpy5);
	ROS_INFO_STREAM("i ="<<i);  
	
	myfile.open ("data.txt", fstream::app|fstream::out);
    myfile << i << '\t' << msg.linear.x << '\t' << msg.linear.y << '\t' <<msg.linear.z <<'\t' << msg.angular.z;
    myfile << '\t' << msgOut.linear.x << '\t' << msgOut.linear.y << '\t' << msgOut.angular.z <<'\n';
    myfile.close();	
	
	//publish
	pub_.publish(msgOut);
}
	
int main(int argc, char **argv){
	      	  
	ros::init(argc, argv, "pid_control");
	pid_control control;
	ros::spin();
}
