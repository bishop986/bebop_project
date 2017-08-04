#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

class odom_fix {
	public:

		odom_fix(::tf::TransformListener* tf):
			initialized_(false), qr_counter(0)
	    {
			this->initialize( tf);
		}
		void update_odom(const ::nav_msgs::OdometryConstPtr &msg);
		void update_error(const ::geometry_msgs::PoseStampedConstPtr &msg);
		~odom_fix();
	private:
		
		void initialize(::tf::TransformListener* tf);
		bool initialized_;
		int qr_counter;

		::tf::TransformListener* tf_;
		::std::map< int, ::geometry_msgs::Vector3> qr_realpos;
		::std::string p_picture_topic;
		::std::string p_picture_frame;
		::std::string p_odom_frame;
		::std::string p_odom_topic;
		::std::string p_odom_fix_topic;
		::std::vector<int> p_qrcode_seq;
	    int p_qrcode_count;
		::ros::Publisher odom_publisher;
		::ros::Subscriber odom_subscriber;
		::ros::Subscriber picture_subscriber;
		::geometry_msgs::PoseStamped* error_pose;
};
