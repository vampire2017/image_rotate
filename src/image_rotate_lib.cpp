//
// Created by bpeer on 17-10-25.
//
#include "image_rotate/image_rotate.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

void mapCallback( const nav_msgs::OccupancyGrid& map )
{
	ImageRotate imageRotate;
	double x = 0, y = 2.0, th = 0;  // x y th
	geometry_msgs::PoseStamped src_point;
	imageRotate.point_pub_src = imageRotate.n.advertise<geometry_msgs::PoseStamped>("point_src", 1, true);
	src_point.header.frame_id = "map";
	src_point.header.stamp = ros::Time::now();
	src_point.pose.position.x = x;
	src_point.pose.position.y = y;
	src_point.pose.orientation = tf::createQuaternionMsgFromYaw( th );
	imageRotate.point_pub_src.publish( src_point );


	nav_msgs::OccupancyGrid src_map, dist_map;
	src_map = map;
	src_map = imageRotate.receiveMapDataCallback( src_map, x, y, th );
	geometry_msgs::PoseStamped dist_point;
	imageRotate.point_pub_dist = imageRotate.n.advertise<geometry_msgs::PoseStamped>("point_dst", 1, true);
	dist_point.header.frame_id = "map";
	dist_point.header.stamp = ros::Time::now();
	dist_point.pose.position.x = x;
	dist_point.pose.position.y = y;
	dist_point.pose.orientation = tf::createQuaternionMsgFromYaw( th );
	imageRotate.point_pub_dist.publish( dist_point );


	geometry_msgs::PoseStamped axis_point;
	imageRotate.point_axis = imageRotate.n.advertise<geometry_msgs::PoseStamped>("point_axis", 1, true);
	axis_point.header.frame_id = "map";
	axis_point.header.stamp = ros::Time::now();
	axis_point.pose.position.x = 0;
	axis_point.pose.position.y = 0;
	axis_point.pose.orientation.w = 1;
	imageRotate.point_axis.publish( axis_point );

	imageRotate.map_pub = imageRotate.n.advertise<nav_msgs::OccupancyGrid>("map_resize", 1, true);
	imageRotate.map_pub.publish( src_map );

	sleep(3);
	CutMap cutMap_;
	cutMap_( src_map, dist_map );
	imageRotate.MapSave( dist_map );
	imageRotate.YamlSave( dist_map );

	//test ---// Latched publisher for data
	imageRotate.map_pub_cut = imageRotate.n.advertise<nav_msgs::OccupancyGrid>("map_cut_resize", 1, true);
	imageRotate.map_pub_cut.publish( dist_map );
	std::cout << "test end ... " << std::endl;
	while( ros::ok() )
	{
		sleep(1);
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "image_rotate_node");
	ros::NodeHandle nh_;

	ros::Subscriber receive_map_ = nh_.subscribe("/map", 1, mapCallback);

	ros::spin();
	return 0;
}

