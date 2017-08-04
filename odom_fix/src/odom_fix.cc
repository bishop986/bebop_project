#include "../include/odom_fix/odom_fix.h"

void odom_fix::initialize(::tf::TransformListener *tf)
{
	if (this->initialized_) {
		return;
	}

	this->tf_ = tf;

	::ros::NodeHandle node_public;
	::ros::NodeHandle node_private("~/");
	
	node_private.param( "picture_topic", this->p_picture_topic, ::std::string("qr_position"));
	node_private.param( "odom_topic",    this->p_odom_topic, ::std::string("/bebop/odom"));
	node_private.param( "odom_frame",    this->p_odom_frame,    ::std::string("odom"));
	node_private.param( "odom_fix_topic", this->p_odom_fix_topic, ::std::string("/bebop/odom_fix"));
	node_private.getParam( "qrcode_seq", this->p_qrcode_seq);

	// read qr_code position from paramserver which're written in config file
	for(auto counter = this->p_qrcode_seq.begin(); counter != this->p_qrcode_seq.end() ; ++counter)
	{
		::geometry_msgs::Vector3 tmp_pos;
		::std::map< ::std::string,double> tmp;
		::std::stringstream ss;
		::std::string str;

		ss << "qr_code_" << *counter;
		str = ss.str();
		node_private.getParam( str, tmp);
		tmp_pos.x = tmp["x"];
		tmp_pos.y = tmp["y"];
		tmp_pos.z = tmp["z"];

		ROS_INFO("Get qrcode position: X:[%f] Y:[%f] Z:[%f]", tmp["x"], tmp["y"], tmp["z"]);

		qr_realpos[*counter] = tmp_pos;
		
	}


	this->odom_publisher = 
		node_public.advertise< ::nav_msgs::Odometry>(this->p_odom_fix_topic, 10);
	this->odom_subscriber = 
		node_public.subscribe("/bebop/odom", 10, &odom_fix::update_odom, this);
	this->picture_subscriber = 
		node_public.subscribe( this->p_picture_topic, 10, &odom_fix::update_error, this);

	// set initial offset as zero
	this->error_pose = new ::geometry_msgs::PoseStamped();

	this->error_pose->pose.position.x = 0;
	this->error_pose->pose.position.y = 0;
	this->error_pose->pose.position.z = 0;

	// turn starting swtich as true
	this->initialized_ = true;
	ROS_WARN("Initialization Completed");

}

// publish final position
void odom_fix::update_odom( const ::nav_msgs::OdometryConstPtr &msg)
{
	if(!this->initialized_)
	{
		return;
	}

	::nav_msgs::Odometry pub_msg;
	pub_msg.twist = msg->twist;
	pub_msg.header = msg->header;
	pub_msg.child_frame_id = msg->child_frame_id;
	pub_msg.pose.covariance = msg->pose.covariance;
	pub_msg.pose.pose.orientation = msg->pose.pose.orientation;

	// add error to reading data
	pub_msg.pose.pose.position.x = 
		msg->pose.pose.position.x + this->error_pose->pose.position.x;
	pub_msg.pose.pose.position.y =
		msg->pose.pose.position.y + this->error_pose->pose.position.y;
	pub_msg.pose.pose.position.z =
		msg->pose.pose.position.z + this->error_pose->pose.position.z;

	ROS_INFO("Pub pos: X:[%f], Y:[%f], Z:[%f]",
				pub_msg.pose.pose.position.x,
				pub_msg.pose.pose.position.y,
				pub_msg.pose.pose.position.z); 

	this->odom_publisher.publish( pub_msg);
}

// here to update error information from qr_code position
void odom_fix::update_error( const ::geometry_msgs::PoseStampedConstPtr &msg)
{
	if (! this->initialized_) {
		return;
	}

	if(msg->header.frame_id == "none")
	{
		ROS_WARN("No qr_code recieved");
		return;
	}

	::geometry_msgs::PoseStamped odom_qr_pos;

	try {
		this->tf_->transformPose(this->p_odom_frame, *msg, odom_qr_pos);
	} catch (tf::TransformException &e) {
		ROS_ERROR("Failed to compute odom pose, skipping [%s]", e.what());
		return;
	}

	// calculating error
	this->error_pose->pose.position.x = 
		qr_realpos[this->qr_counter].x - odom_qr_pos.pose.position.x;
	this->error_pose->pose.position.y = 
		qr_realpos[this->qr_counter].y - odom_qr_pos.pose.position.y;
	this->error_pose->pose.position.z = 
		qr_realpos[this->qr_counter].z - odom_qr_pos.pose.position.z;
	
	++qr_counter;
}

odom_fix::~odom_fix()
{
	if ( this->error_pose != nullptr) {
		delete this->error_pose;
	}
}
