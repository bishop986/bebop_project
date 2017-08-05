#include "bebop_path_follower/bebop_path_follower.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


class SimpleBebopController{
	public:
		SimpleBebopController() {
			::ros::NodeHandle nh;
			cmd_pub = nh.advertise< ::geometry_msgs::Twist>("/cmd_vel", 1000);
			this->path_follwer_ = new BebopPathFollower(&this->tf_); 
			this->cmd_vel_generator_timer_ = nh.createTimer( ::ros::Duration(0.1), &SimpleBebopController::timerCmdlVelGeneration, this, false);
		}

		void timerCmdlVelGeneration(const ros::TimerEvent& e) {
			::geometry_msgs::Twist twist;
			this->path_follwer_->computeVelocityCommand(twist);
			this->cmd_pub.publish(twist);
		}

		~SimpleBebopController()
		{
			if ( path_follwer_ != nullptr)
			  delete path_follwer_;
		}

	private:
		::BebopPathFollower *path_follwer_;
		::ros::Timer cmd_vel_generator_timer_;
		::tf::TransformListener tf_;
		::ros::Publisher cmd_pub;
};

int main(int argc, char** argv)
{
	::ros::init( argc, argv, "bebop_controller_node");

	SimpleBebopController rc;

	::ros::spin();

	return 0;
}


