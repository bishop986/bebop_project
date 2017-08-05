#include "../include/bebop_path_follower/bebop_path_follower.h"
#include <math.h>
#include "bebop_msgs_nwpu/nextPoint.h"
#include <unistd.h>

BebopPathFollower::BebopPathFollower( ::tf::TransformListener* tf) :
	initialized_(false)
{
	this->initialize(tf);
}

void BebopPathFollower::initialize( ::tf::TransformListener* tf)
{
	this->tf_ = tf;
	
	::ros::NodeHandle node_private("/bebop_controller/");
	::ros::NodeHandle node_public;

	ROS_WARN("Setting initialization begins!");

	node_private.param( "max_linear_vel", this->p_max_linear_vel, 0.5);
	node_private.param( "min_linear_vel", this->p_min_linear_vel, 0.1);

	ROS_INFO_STREAM("linear_vel: max:[" << this->p_max_linear_vel << "] min:[" << this->p_min_linear_vel << "]");

	node_private.param( "max_rotation_vel", this->p_max_rotation_vel, 0.8);
	node_private.param( "min_rotation_vel", this->p_min_rotation_vel, 0.5);

	ROS_INFO_STREAM("rotation_vel: max:[" << this->p_max_rotation_vel << "] min:[" << this->p_min_rotation_vel << "]");
	
	node_private.param( "torlerance_trans", this->p_torlerance_trans_, 0.2);
	node_private.param( "torlerance_rot", this->p_torlerance_rot_, 0.2);
	node_private.param( "torlerance_timeout", this->p_torlerance_timeout_, 0.5);
	node_private.param( "in_place_trans", this->p_in_place_trans, 0.06);
	node_private.param( "in_place_rotation", this->p_in_place_rotation, 0.1);

	ROS_INFO_STREAM("torlerance: trans:[" << this->p_torlerance_trans_ << "] rot:[" << this->p_torlerance_rot_ << "]");
	ROS_INFO_STREAM("torlerance: timeout:[" << this->p_torlerance_timeout_ << "]");
	ROS_INFO_STREAM("in_place: trans:[" << this->p_in_place_trans << "] rot:[" << this->p_in_place_rotation << "]");
	
	node_private.param( "odom_topic", this->p_odom_topic, ::std::string("/bebop/odom_fix"));
	node_private.param( "odom_frame", this->p_odom_frame, ::std::string("odom"));

	ROS_INFO_STREAM("Setting odom_topic:[" << this->p_odom_topic << "] odom_frame:[" << this->p_odom_frame << "]");
	
	node_private.param( "drone_frame", this->p_drone_frame, ::std::string("base_link"));

	ROS_INFO_STREAM("drone_frame: trans:[" << this->p_drone_frame << "]");
	
	// this factor is to configure the cmd_vel
	node_private.param( "scale_factor", this->p_scale_factor_, 0.1);

	ROS_INFO_STREAM("Speed Scale: factor:[" << this->p_scale_factor_ << "]");

	::tf::Stamped< ::tf::Pose> drone_pose;
	drone_pose.setIdentity();
	drone_pose.stamp_ = ros::Time(0);
	drone_pose.frame_id_ = this->p_drone_frame;

	bool frame_exsist = false;
	while( !frame_exsist) {
		try {
			this->tf_->transformPose( this->p_odom_frame, drone_pose, this->current_pose_);
			frame_exsist =  true;
		} catch( ::tf::TransformException &e) {
			ROS_WARN("waiting for tf from %s to %s", this->p_odom_frame.c_str(), this->p_drone_frame.c_str());
			frame_exsist = false;
		}
		sleep(10);
	}

	this->odom_sub_ = 
		node_public.subscribe(this->p_odom_topic, 10, &BebopPathFollower::getDronePose, this);

	this->point_reach_time = ::ros::Time::now();

	this->plan_reciever_ = node_public.serviceClient< ::bebop_msgs_nwpu::nextPoint>("/next_point_shower");

	ROS_WARN("Initialization completed!");

	this->initialized_ = true;
}

void BebopPathFollower::getDronePose( const ::nav_msgs::OdometryConstPtr& msgs)
{

	if ( !this->initialized_)
	  return;

	::geometry_msgs::PoseStamped tmp;
	tmp.header = msgs->header;
	tmp.pose.orientation = msgs->pose.pose.orientation;
	tmp.pose.position = msgs->pose.pose.position;

	::tf::poseStampedMsgToTF( tmp, this->current_pose_);
}

// calculate the angle diff
double BebopPathFollower::headingDiff( const double x, const double y, 
			                           const double pt_x, const double pt_y, 
									   const double heading)
{
	double v1_x = x - pt_x;
	double v1_y = y - pt_y;
	double v2_x = ::std::cos(heading);
	double v2_y = ::std::sin(heading);
	
	double perp_dot = v1_x * v2_y - v1_y * v2_x;
	double dot = v1_x * v2_x + v1_y * v2_y;

	double vector_angle = ::std::atan2( perp_dot, dot);

	return -1.0 * vector_angle;
}

::geometry_msgs::Twist BebopPathFollower::diff2D( const ::tf::Pose& drone_pose, 
												  const ::tf::Pose& target_pose)
{
	::geometry_msgs::Twist res;
	::tf::Pose diff = target_pose.inverse() * drone_pose;
	res.linear.x = diff.getOrigin().x();
	res.linear.y = diff.getOrigin().y();
	res.angular.z = ::tf::getYaw(diff.getRotation());

	if ( ::std::fabs( res.linear.x) <= this->p_torlerance_trans_
		&& ::std::fabs( res.linear.y) <= this->p_torlerance_trans_) {

		return res;
	}

	// In case we can't reach the goal
	
	double yaw_diff = this->headingDiff(drone_pose.getOrigin().x(),
				drone_pose.getOrigin().y(), target_pose.getOrigin().x(),
				target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));
				
	double neg_yaw_diff = this->headingDiff(drone_pose.getOrigin().x(),
				drone_pose.getOrigin().y(), target_pose.getOrigin().x(),
				target_pose.getOrigin().y(), M_PI + tf::getYaw(target_pose.getRotation()));

	if( ::std::fabs( neg_yaw_diff) < ::std::fabs( yaw_diff)){
		yaw_diff = neg_yaw_diff;
	}

	::tf::Quaternion rot_diff = ::tf::createQuaternionFromYaw(yaw_diff);
	::tf::Pose new_pose = drone_pose;
	new_pose.setRotation(rot_diff);

	diff = target_pose.inverse() * new_pose;
	res.linear.x = diff.getOrigin().x();
	res.linear.y = diff.getOrigin().y();
	res.angular.z = ::tf::getYaw( diff.getRotation());
	return res;
}

bool BebopPathFollower::computeVelocityCommand( ::geometry_msgs::Twist& res)
{

	if ( !this->initialized_) {
		::geometry_msgs::Twist empty_vel;
		res = empty_vel;
	    return false;
	}

	::bebop_msgs_nwpu::nextPoint point_srv;
	::geometry_msgs::PoseStamped target_pose;
	::tf::Stamped< ::tf::Pose> target_pose_tf;
	::geometry_msgs::PoseStamped current_point;
	::geometry_msgs::Twist empty_vel;

	if ( !this->plan_reciever_.call(point_srv))
	{
		ROS_ERROR(" Call strategy service failed");
		res = empty_vel;
		return false;
	}

	::tf::poseStampedTFToMsg(this->current_pose_, current_point);

	target_pose = point_srv.response.point;

	if ( target_pose.pose.orientation.x <= 1e-6 && target_pose.pose.orientation.y <= 1e-6 &&
				target_pose.pose.orientation.z <= 1e-6 && target_pose.pose.orientation.w <= 1e-6) {
		
		double dx = target_pose.pose.position.x - current_point.pose.position.x;
		double dy = target_pose.pose.position.y - current_point.pose.position.y;
		double yaw_path = ::std::atan2(dy, dx);
		
		target_pose.pose.orientation.x = 0.0;
		target_pose.pose.orientation.y = 0.0;
		target_pose.pose.orientation.y = ::std::sin(yaw_path*0.5f);
		target_pose.pose.orientation.w = ::std::cos(yaw_path*0.5f);
	}

	tf::poseStampedMsgToTF( target_pose, target_pose_tf);

	res = this->diff2D( this->current_pose_, target_pose_tf);

	bool isReached = false;

	if ( ::std::fabs( res.linear.x) <= this->p_torlerance_trans_ &&
		 ::std::fabs( res.linear.y) <= this->p_torlerance_trans_ &&
		 ::std::fabs( res.angular.z) <= this->p_torlerance_rot_) {
		isReached = true;
	}

	if ( !isReached) {
		this->point_reach_time = ::ros::Time::now();
	}

	if ( this->point_reach_time + 
				::ros::Duration(this->p_torlerance_timeout_) < ::ros::Time::now()) {
		res = empty_vel;
	}

	this->limitTwist( res);
	return true;
}

void BebopPathFollower::limitTwist( ::geometry_msgs::Twist& twist_vel)
{
	double line_overlimit = ::std::sqrt( twist_vel.linear.x * twist_vel.linear.x +
									    twist_vel.linear.y * twist_vel.linear.y) / 
									    this->p_max_linear_vel;

	double line_underlimit = this->p_min_linear_vel / 
							 ::std::sqrt( twist_vel.linear.x * twist_vel.linear.x + 
										  twist_vel.linear.y * twist_vel.linear.y);

	if ( line_overlimit > 1.0) {
		twist_vel.linear.x /= line_overlimit;
		twist_vel.linear.y /= line_overlimit;
	}

	if ( line_underlimit > 1.0) {
		twist_vel.linear.x /= line_underlimit;
		twist_vel.linear.y /= line_underlimit;
	}

	if ( ::std::fabs( twist_vel.angular.z) > this->p_max_rotation_vel) {
		twist_vel.angular.z = this->p_max_rotation_vel;
	} else if ( ::std::fabs( twist_vel.angular.z) < this->p_min_rotation_vel) {
		twist_vel.angular.z = this->p_min_rotation_vel;
	}

	if ( ::std::sqrt(twist_vel.linear.x * twist_vel.linear.x + twist_vel.linear.y * twist_vel.linear.y) 
				< this->p_in_place_trans){
		if ( ::std::fabs(twist_vel.angular.z) < this->p_in_place_rotation) {
			twist_vel.angular.z = this->p_in_place_rotation * ( twist_vel.angular.z < 0? -1:1);
		}
		twist_vel.linear.x = 0;
		twist_vel.linear.y = 0;
	}

}
	  





	






















	
