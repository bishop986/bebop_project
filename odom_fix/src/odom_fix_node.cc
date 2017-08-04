#include "../include/odom_fix/odom_fix.h"

int main(int argc, char** argv)
{
	::ros::init(argc, argv, "odom_fix_node");
	::tf::TransformListener tf;
	odom_fix StartOdom(&tf);

	::ros::spin();
}

