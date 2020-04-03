/*
  IMU Dead Reckoning
  Update Position of a Robot given IMU measurements
  IMU Navigation Paper: https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf-6.1, 6.2

  Inputs:
  	- angular velocity
  	- linear acceleration
  Outputs:
  	- orientation
  	- position
*/

#include<cmath>
#include<iostream>

#include<Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
// #include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

class IMU_Navigation
{
private:
	//ROS
	ros::NodeHandle node_handle;
	ros::Subscriber sub_imu;
	ros::Publisher pub_marker;
    uint16_t marker_id;
	// visualization_msgs::Marker marker;
	//Inertial Navigation
	ros::Time latest_time; //latest timestamp
	Eigen::Matrix3d C;
	Eigen::Vector3d gravity;
	Eigen::Vector3d vel_global, pos_global;

	void msg_callback(const sensor_msgs::Imu::ConstPtr& msg){
		ros::Time current_time = ros::Time::now();
        visualization_msgs::Marker marker;

		// Initial msg
		if (latest_time == ros::Time(0)){
			this -> C = Eigen::Matrix3d::Identity(); // Initial orientation
			this -> vel_global = Eigen::Vector3d::Zero(); // Initial velocity
			this -> pos_global = Eigen::Vector3d::Zero(); // Initial position
			this -> gravity = Eigen::Vector3d(msg -> linear_acceleration.x, msg -> linear_acceleration.y, msg -> linear_acceleration.z);
			this -> latest_time = current_time;
			return;

		}
		else{
			// Next msg
		    ros::Duration dt = current_time - this -> latest_time;
			this -> get_orientation(msg -> angular_velocity, dt);
			this -> get_position(msg -> linear_acceleration, dt);
			this -> get_marker(this -> pos_global, marker, current_time);
	  		return;
  		}
	}

	void get_orientation(const geometry_msgs::Vector3& ang_vel,
						 const ros::Duration dt){
		Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
		Eigen::Vector3d ang_vel_dt = Eigen::Vector3d(ang_vel.x, ang_vel.y, ang_vel.z) * dt.toNSec()/1e9;
		double sigma = ang_vel_dt.norm();
		Eigen::Matrix3d B = Eigen::Matrix3d::Zero(); // B: a 3x3 rotation matrix
		B(0, 1) = - ang_vel_dt[2];
		B(0, 2) = ang_vel_dt[1];
		B(1, 0) = ang_vel_dt[2];
		B(1, 2) = - ang_vel_dt[0];
		B(2, 0) = - ang_vel_dt[1];
		B(2, 1) = ang_vel_dt[0];
		// std::cout << "B = " << B << std::endl;

		// Update Orientation C (C is 3x3 rotation matrix)
		C = C * (I + B * (sin(sigma) / sigma) + B * B * (((double)1 - cos(sigma) / sigma * sigma)));
		// std::cout << "C = " << C << std::endl;
	}

	void get_position(const geometry_msgs::Vector3& lin_acce, const ros::Duration dt){

		Eigen::Vector3d acce_global = C * Eigen::Vector3d(lin_acce.x, lin_acce.y, lin_acce.z);
		this -> vel_global += (acce_global - gravity) * (double)dt.toNSec() / 1e9;
		this -> pos_global += this -> vel_global * (double)dt.toNSec() / 1e9;
        // std::cout << "pos: \n" << acce_global.transpose() << std::endl;


	}

	void get_marker(Eigen::Vector3d pos_global, visualization_msgs::Marker marker, ros::Time current_time){
		// visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.type = visualization_msgs::Marker::CUBE; 
		marker.header.stamp = ros::Time::now();
		marker.id = marker_id++;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1;
		marker.color.a = 1;
		// geometry_msgs::Point p;
		// p.x = x, p.y = y, p.z = z;
		marker.pose.position.x = pos_global(0);
		marker.pose.position.y = pos_global(1);
		marker.pose.position.z = pos_global(2);
		// std::cout << "marker.pose.position: " << marker.pose.position << std::endl;
		this -> pub_marker.publish(marker);
		// pub_marker.publish(marker)
		this -> latest_time = current_time;
		//return;
	}
public:
	IMU_Navigation();
	IMU_Navigation(ros::NodeHandle nh){
		this -> node_handle = nh;
		ROS_INFO("Initializing node...");
		this -> latest_time = ros::Time(0);
		this -> sub_imu = node_handle.subscribe("/imu/data", 1, &IMU_Navigation::msg_callback, this);
		this -> pub_marker = node_handle.advertise<visualization_msgs::Marker>("/position", 1);
		marker_id = 0;
		ROS_INFO("Ready to update position!");
	}
	// ~IMU_Navigation();
	
};

int main(int argc, char** argv){
	ros::init(argc, argv, "imu_navigation_dead_reckoning");
	ros::NodeHandle n;
	IMU_Navigation node(n);
	while (ros::ok())
		ros::spin();
	return 0;
}