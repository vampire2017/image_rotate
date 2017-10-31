//
// Created by bpeer on 17-10-25.
//

#ifndef IMAGE_ROTATE_IMAGE_ROTATE_H
#define IMAGE_ROTATE_IMAGE_ROTATE_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"

class ImageRotate
{
public:
	ImageRotate();

	void rotateImage( cv::Mat img, double degree );

	double detectDegree( cv::Mat img );

	nav_msgs::OccupancyGrid receiveMapDataCallback( const nav_msgs::OccupancyGridConstPtr& map, double &x_, double &y_, double &th_ );

//  // for test
//	nav_msgs::GetMap::Response map_resp_;  //返回修改后的map:.map
//	ros::NodeHandle n;
//	ros::Publisher map_pub;
private:
	cv::Point2f center;
	double scale;
	bool saved_map_;
//	std::string mapname_;  //保存图片的位置 test
	std::string mapdatafile;  //保存图片的位置pgm
	std::string mapmetadatafile;  //保存图片配置文件的位置yaml
	nav_msgs::GetMap::Response map_resp_;  //返回修改后的map:.map
};


#endif //IMAGE_ROTATE_IMAGE_ROTATE_H
