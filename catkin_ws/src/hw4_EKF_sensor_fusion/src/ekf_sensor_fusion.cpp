/*
  Extended Kalman Filter for Sensor Fusion 
	-sensors:
		- /imu/data (IMU)
		  	- angular velocity
  			- linear acceleration
		- /zed/odom (zed stereo camera odometry)

  Outputs after using EKF for sensor fusion:
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
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include<Eigen/Geometry>



class Ekf_Fusion
{
private:
	//ROS
	ros::NodeHandle node_handle;
	ros::Subscriber sub_imu; // to subscibe /imu/data
	ros::Subscriber sub_odom_fused; // to subscribe sensor fusion results from /robot_pose_ekf/odom_combined
	ros::Subscriber sub_zed_odom; // to subscribe /zed/odom

	ros::Publisher pub_imu_refined; // to publish imu transformed from IMU'frame to Zed's frame
	ros::Publisher pub_marker; // to publish position (x, y, z)
    uint16_t marker_id;
	// visualization_msgs::Marker marker;
	//Inertial Navigation
	ros::Time latest_time; //latest timestamp
	Eigen::Matrix3d C;
	Eigen::Vector3d gravity;
	Eigen::Vector3d vel_global, pos_global;

	visualization_msgs::Marker imu_marker, fusion_marker, zed_marker;
	// Eigen::Matrix3d Imu_to_Zed;
    Eigen::Matrix3d Imu_to_Cam, Cam_to_Zed, Imu_to_Zed; // 2 rotatin matrix
    
    // Imu_to_Cam <<   0.0225226, 0.999745, 0.0017194,
				// 	0.0648765, -0.00317777, 0.997888,
				// 	0.997639, -0.0223635, -0.0649315;
    // Cam_to_Zed <<  0,  0, 1,
				// 	-1, 0, 0,
				// 	0, -1, 0;

    // Imu_to_Zed = Cam_to_Zed * Imu_to_Cam;

    // std::cout << "Imu_to_Cam\n" << Imu_to_Cam << std::endl;
    // std::cout << "Cam_to_Zed\n" << Cam_to_Zed << std::endl;

    void set_calibration_param(){
	    // Eigen::Matrix3d Imu_to_Cam, Cam_to_Zed, Imu_to_Zed; // 2 rotatin matrix
	    Imu_to_Cam <<   0.0225226, 0.999745, 0.0017194,
						0.0648765, -0.00317777, 0.997888,
						0.997639, -0.0223635, -0.0649315;
	    Cam_to_Zed <<  0,  0, 1,
						-1, 0, 0,
						0, -1, 0;

	    Imu_to_Zed = Cam_to_Zed * Imu_to_Cam;
    }

	void get_imu_transformed(const sensor_msgs::Imu::ConstPtr& msg){
		// to transform IMU's frame to Zed's frame & publish it for robot_pose_ekf

		sensor_msgs::Imu imu_transformed_msg;

		// get angular velocity from msg
		Eigen::Vector3d ang_vel = Eigen::Vector3d(msg -> angular_velocity.x,
												msg -> angular_velocity.y,
												msg -> angular_velocity.z);
		Eigen::Vector3d ang_vel_transformed = this -> Imu_to_Zed * ang_vel;// transform angular velocity
		
		// get linear velocity from msg
		Eigen::Vector3d lin_acc = Eigen::Vector3d(msg -> linear_acceleration.x,
												msg -> linear_acceleration.y,
												msg -> linear_acceleration.z);
		Eigen::Vector3d lin_acc_tranformed = this -> Imu_to_Zed * lin_acc; // transform linear velocity
		
		// get orientation from msg
		Eigen::Matrix3d orientation(Eigen::Quaterniond(msg -> orientation.w,
													msg -> orientation.x,
													msg -> orientation.y,
													msg -> orientation.z)); // // BUG solved (xyzw -> wxyz)!!

		Eigen::Quaterniond orientation_tranformed(this -> Imu_to_Zed * orientation); // transform orientation

		// imu transformed orientation
		imu_transformed_msg.orientation.x = orientation_tranformed.x(); 
		imu_transformed_msg.orientation.y = orientation_tranformed.y();
		imu_transformed_msg.orientation.z = orientation_tranformed.z();
		imu_transformed_msg.orientation.w = orientation_tranformed.w();

		// imu transformed angular velocity
		imu_transformed_msg.angular_velocity.x = ang_vel_transformed(0);
		imu_transformed_msg.angular_velocity.y = ang_vel_transformed(1);
		imu_transformed_msg.angular_velocity.z = ang_vel_transformed(2);


		// imu transformed linear acceleration
		imu_transformed_msg.linear_acceleration.x = lin_acc_tranformed(0);
		imu_transformed_msg.linear_acceleration.y = lin_acc_tranformed(1);
		imu_transformed_msg.linear_acceleration.z = lin_acc_tranformed(2);

		// header & frame id
		imu_transformed_msg.header = msg -> header;
		imu_transformed_msg.header.frame_id = "map";

        // std::cout << "imu tf: \n" << imu_transformed_msg << std::endl;

		this -> pub_imu_refined.publish(imu_transformed_msg);
	}

	void get_imu_orientation(const geometry_msgs::Vector3& ang_vel,
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

	void get_imu_position(const geometry_msgs::Vector3& lin_acce, const ros::Duration dt){

		Eigen::Vector3d acce_global = C * Eigen::Vector3d(lin_acce.x, lin_acce.y, lin_acce.z);
		this -> vel_global += (acce_global - gravity) * (double)dt.toNSec() / 1e9;
		this -> pos_global += this -> vel_global * (double)dt.toNSec() / 1e9; // Update Position
        // std::cout << "pos: \n" << acce_global.transpose() << std::endl;


	}

	void set_imu_marker(Eigen::Vector3d pos_global_tf){

        //draw imu marker
        geometry_msgs::Point point;
        point.x = pos_global_tf(0); 
        point.y = pos_global_tf(1); 
        point.z = pos_global_tf(2);
        imu_marker.id++;
        imu_marker.action = visualization_msgs::Marker::ADD;
        imu_marker.points.push_back(point);
		
		this -> pub_marker.publish(imu_marker);
	
	}

	void set_fusion_marker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
		// fusion_marker.header.frame_id = "map";

		fusion_marker.id++;
		fusion_marker.action = visualization_msgs::Marker::ADD;

	}

	void set_zedcam_marker(const nav_msgs::Odometry::ConstPtr& msg){

		zed_marker.id++;
		zed_marker.action = visualization_msgs::Marker::ADD;
		// zed_marker.points.push_back(zed_pos);
		zed_marker.header.stamp = msg -> header.stamp;

	}

	void init_imu_marker(){
	    imu_marker.type = visualization_msgs::Marker::LINE_STRIP;
	    imu_marker.id = 0;
	    imu_marker.header.frame_id = "map";
	    imu_marker.ns = "imu_xyz";
	    imu_marker.scale.x = 0.2;
	    imu_marker.color.a = 2.0;
	    imu_marker.color.r = 0.0;
	    imu_marker.color.g = 0.0;
	    imu_marker.color.b = 1.0;		
	}

	void init_fusion_marker(){
		fusion_marker.type = visualization_msgs::Marker::LINE_STRIP;
        fusion_marker.id = 0;
        fusion_marker.header.frame_id = "map";
        fusion_marker.ns = "fusion_xyz";
        fusion_marker.scale.x = 0.2;
        fusion_marker.color.a = 2.0;
        fusion_marker.color.r = 0.0;
        fusion_marker.color.g = 1.0;
        fusion_marker.color.b = 0.0;
	}

	void init_zed_marker(){
        zed_marker.type = visualization_msgs::Marker::LINE_STRIP;
        zed_marker.id = 0;
        zed_marker.header.frame_id = "map";
        zed_marker.ns = "zed_xyz";
        zed_marker.scale.x = 0.2;
        zed_marker.color.a = 2.0;
        zed_marker.color.r = 1.0;
        zed_marker.color.g = 0.0;
        zed_marker.color.b = 0.0;
	}

	void imu_msg_callback(const sensor_msgs::Imu::ConstPtr& msg){
		// transform imu & publish new imu (/imu_data)
		this -> get_imu_transformed(msg);

		// ros::Time current_time = ros::Time::now();
		ros::Time current_time = msg -> header.stamp;
        // visualization_msgs::Marker marker;
        // this -> get_imu_transformed(msg);
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
			this -> get_imu_orientation(msg -> angular_velocity, dt);
			this -> get_imu_position(msg -> linear_acceleration, dt);
			// std::cout << "Imu_to_Zed:\n" << Imu_to_Zed << std::endl;
			// std::cout << "pos_global:\n" << pos_global << std::endl;

			// transform position from IMU's frame to Zed camera's frame
			Eigen::Vector3d pos_global_tf = Imu_to_Zed * pos_global;

			// set imu marker
			this -> set_imu_marker(pos_global_tf);
	        imu_marker.header.stamp = current_time;

	        // publish imu marker
	        this->pub_marker.publish(imu_marker);

	        // update time stamp
			this -> latest_time = current_time;
	  		return;
  		}
	}

	void odom_fused_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
		// get EKF for sensor fusion result from robot_pose_ekf
		geometry_msgs::Point odom_pos;
		odom_pos.x = msg -> pose.pose.position.x;
		odom_pos.y = msg -> pose.pose.position.y;
		odom_pos.z = msg -> pose.pose.position.z;

		fusion_marker.points.push_back(odom_pos);
		fusion_marker.header.stamp = msg -> header.stamp;
		this -> set_fusion_marker(msg);
		this -> pub_marker.publish(this -> fusion_marker);
		return;
		
	}

	void zed_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
		// get msg from Zed sterio camera
		geometry_msgs::Point zed_pos;
		zed_pos.x = msg -> pose.pose.position.x;
		zed_pos.y = msg -> pose.pose.position.y;
		zed_pos.z = msg -> pose.pose.position.z;

		zed_marker.points.push_back(zed_pos);
		// zed_marker.header.stamp = msg -> header.stamp;
		this -> set_zedcam_marker(msg);
		this -> pub_marker.publish(this -> zed_marker);
		return;

	}

public:
	Ekf_Fusion();
	Ekf_Fusion(ros::NodeHandle nh){
		this -> node_handle = nh;
		ROS_INFO("Initializing node...");
		this -> latest_time = ros::Time(0);

		this -> sub_imu = node_handle.subscribe("/imu/data", 1, &Ekf_Fusion::imu_msg_callback, this);
		this -> sub_zed_odom = node_handle.subscribe("/zed/odom", 1, &Ekf_Fusion::zed_odom_callback, this);
		this -> sub_odom_fused = node_handle.subscribe("/robot_pose_ekf/odom_combined", 1, &Ekf_Fusion::odom_fused_callback, this);

		this -> pub_marker = node_handle.advertise<visualization_msgs::Marker>("/position", 1);
		this -> pub_imu_refined = node_handle.advertise<sensor_msgs::Imu>("/imu_data", 1);

		this -> init_imu_marker();
		this -> init_fusion_marker();
		this -> init_zed_marker();
		this -> set_calibration_param();

		ROS_INFO("Ready for sensor fusion!");
	}
	// ~Ekf_Fusion();
	
};

int main(int argc, char** argv){
	ros::init(argc, argv, "ekf_sensor_fusion");
	ros::NodeHandle n;
	Ekf_Fusion node(n);
	while (ros::ok())
		ros::spin();
	return 0;
}
