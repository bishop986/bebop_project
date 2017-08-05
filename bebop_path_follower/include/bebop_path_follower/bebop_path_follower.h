#ifndef BEBOP_PATH_FOLLOWER_H_
#define BEBOP_PATH_FOLLOWER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <cmath>

class BebopPathFollower {
	public:
		BebopPathFollower( ::tf::TransformListener* tf);
		bool computeVelocityCommand( ::geometry_msgs::Twist& res);
	private:
		::geometry_msgs::Twist diff2D(const ::tf::Pose& drone_pose, const ::tf::Pose& target_pose);
		void initialize( ::tf::TransformListener* tf);
		double headingDiff( const double x, const double y, const double pt_x, const double pt_y, const double heading);
		void limitTwist( ::geometry_msgs::Twist& twist_vel);
		void getDronePose( const ::nav_msgs::OdometryConstPtr& msgs);

		::tf::TransformListener *tf_;
		::ros::Subscriber odom_sub_;
		::ros::ServiceClient plan_reciever_;
		::tf::Stamped< ::tf::Pose> current_pose_;
		::ros::Time point_reach_time;

		double p_max_linear_vel;
		double p_min_linear_vel;
		double p_max_rotation_vel;
		double p_min_rotation_vel;
		double p_torlerance_rot_;
		double p_torlerance_trans_;
		double p_torlerance_timeout_;
		double p_scale_factor_;
		double p_in_place_trans;
		double p_in_place_rotation;

		::std::string p_odom_frame;
		::std::string p_odom_topic;
		::std::string p_drone_frame;

		bool initialized_;
};
#endif
		
