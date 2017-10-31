//
// Created by bpeer on 17-10-25.
//
#include "image_rotate/image_rotate.h"

void mapCallback( const nav_msgs::OccupancyGridConstPtr& map )
{
	ImageRotate imageRotate;
	double x = 0.0, y = 0.0, th = 0.0;  // x y th
	imageRotate.receiveMapDataCallback( map, x, y, th );

//	//test ---// Latched publisher for data
//	imageRotate.map_pub = imageRotate.n.advertise<nav_msgs::OccupancyGrid>("map_resize", 1, true);
//	while(1)
//	{
//		imageRotate.map_pub.publish( imageRotate.map_resp_.map );
//		sleep(1);
//	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "image_rotate_node");
	ros::NodeHandle nh_;

	ros::Subscriber receive_map_ = nh_.subscribe("/map", 1, mapCallback);

	ros::spin();
	return 0;
}

