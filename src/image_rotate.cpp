//
// Created by bpeer on 17-10-25.
//

#include "image_rotate/image_rotate.h"

ImageRotate::ImageRotate()
{
	mapdatafile = "/home/bpeer/Desktop/test1.pgm";
	mapmetadatafile = "/home/bpeer/Desktop/test1.yaml";
}

double ImageRotate::detectDegree(cv::Mat img)
{
	if ( img.empty() )
		return 1000.0;

	cv::Mat output_gray, output_img;

	//【1】将原图像转换为灰度图像
	cv::cvtColor( img, output_gray, CV_BGR2GRAY );

	//【2】先用使用 3x3内核来降噪
	cv::blur( output_gray, output_img, cv::Size(3,3) );

	//【3】执行Canny边缘检测
	cv::Canny( output_img, output_img, 50, 150, 3 );

	// 对边缘图像提取轮廓信息
	std::vector<std::vector<cv::Point> >contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(output_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::cout << "2..." << std::endl;

	cv::Mat contours_image = cv::Mat::zeros(img.size(),CV_8UC1); ;
	//画出轮廓
	cv::drawContours(contours_image, contours, -1, cv::Scalar(255, 0, 255));
	cv::imshow("contours", contours_image);
	std::cout << "3..." << std::endl;
	cv::waitKey(0);

	//【4】执行概率霍夫线变换，检测直线
	std::vector< cv::Vec4i > lines;  //定义一个矢量结构lines用于存放得到的线段矢量集合
	double minLineLength = std::min(output_img.cols, output_img.rows) * 0.02; //最短线长度
	double maxLineGap = std::min(output_img.cols, output_img.rows) * 0.02 ; //最小线间距

	int threshold = 100;
	// test
	std::cout << "minLineLength:= " << minLineLength << std::endl;
	std::cout << "maxLineGap:= " << maxLineGap << std::endl;

	cv::HoughLinesP( output_img, lines, 1, M_PI/180, threshold, minLineLength, maxLineGap );

	std::cout << "num lines:= " << lines.size() << std::endl;

	int tmp_iter = 0;
	while ( lines.empty() )
	{
		threshold = threshold - 2;
		cv::HoughLinesP( output_img, lines, 1, M_PI/180, threshold, minLineLength, maxLineGap );
		tmp_iter++;
		std::cout << "threshold: " << threshold << std::endl;
		if ( 15 == tmp_iter)
			return 1000.0;
	}

	//【6】筛选直线
	int x_s, y_s, x_e, y_e;
	double angle;   //直线的角度
	double best_angle = 0.0; //最优角度
	double line_length, max_length = 0.0; //直线长度
	cv::Vec4i best_line_test;  //for test--show

	for( size_t i = 0; i < lines.size(); i++ )
	{
		std::cout << "lines:= " << lines[i] << std::endl;

		x_s = lines[i][0];
		y_s = lines[i][1];
		x_e = lines[i][2];
		y_e = lines[i][3];

		//计算线的长度
		line_length = sqrt( (x_s-x_e) * (x_s-x_e) + (y_s-y_e) * (y_s-y_e) );
		std::cout << "line_length:= " << line_length << std::endl;
		if( line_length > max_length )
		{
			//计算直线角度(-90 -- 90)
			angle = (x_s == x_e) ? 90 : ( atan( (y_e-y_s)*1.0/(x_e-x_s) ) / M_PI * 180 );
			std::cout << "< angle:= " << angle << " >" << std::endl;

			max_length = line_length;
			best_angle = angle;

			best_line_test = lines[i];
		}
	}

	cv::line( output_img, cv::Point(best_line_test[0], best_line_test[1]),
	          cv::Point(best_line_test[2],best_line_test[3]),
	          cv::Scalar(186,88,255), 4, CV_AA);
	cv::imshow("HoughLinesP..",output_img);
	cv::waitKey(0);  //@todo

	//【7】计算旋转角度  °C
	double rotating_angle;
	if ( 90 == std::fabs(best_angle) || std::fabs(best_angle) < 1e-5 )
		rotating_angle = 0;
	else if ( std::fabs(best_angle) < 45 )
		rotating_angle = (-best_angle);
	else
		rotating_angle = ( 90 - std::fabs(best_angle) ) * best_angle / std::fabs(best_angle);

	return rotating_angle;
}

void ImageRotate::rotateImage(cv::Mat img, double degree)
{
	std::cout << "rotate angle:= " <<  degree << std::endl;
	cv::Mat img_rotate;
	cv::Mat img_rotate_dst = img.clone();
	cv::Mat rot_mat( 2, 3, CV_32FC1 );

	//得到图像大小
	int width = img.cols;
	int height = img.rows;

	//【1】计算图像中心点
	center.x = width / 2.0;
	center.y = height / 2.0;

	//【2】获得旋转变换矩阵
	scale = 1.0;
	cv::Mat trans_mat = getRotationMatrix2D( center, -degree, scale );

	//【3】仿射变换
	cv::warpAffine( img, img_rotate_dst, trans_mat, cv::Size(width, height), 1, 1 );

	//【4】保存图片
	cv::imwrite( mapdatafile, img_rotate_dst );

	cv::imshow( "src", img );

	cv::imshow( "dist", img_rotate_dst );

//	// for test
//	cv::circle( img_rotate_dst, cv::Point(img.cols/2, img.rows/2), 5, cv::Scalar(1,0,255));
//	cv::imshow( "show here", img_rotate_dst );
	cvWaitKey(0);  //@todo
}

nav_msgs::OccupancyGrid ImageRotate::receiveMapDataCallback( const nav_msgs::OccupancyGridConstPtr& map, double &x_, double &y_, double &th_ )
{
	saved_map_ = false;

	ROS_INFO("Received a %d X %d map @ %.3f m/pix",
	         map->info.width,
	         map->info.height,
	         map->info.resolution);

	//@todo mapname_  保存图片的位置
//	std::string mapdatafile = mapname_ + ".pgm";
	ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
	FILE* out = fopen(mapdatafile.c_str(), "w");
	if (!out)
	{
		ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
		exit( -1 );
	}

	fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
	        map->info.resolution, map->info.width, map->info.height);

	for(unsigned int y = 0; y < map->info.height; y++) {
		for(unsigned int x = 0; x < map->info.width; x++) {
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			if (map->data[i] == 0) { //occ [0,0.1)
				fputc(254, out);
			} else if (map->data[i] == +100) { //occ (0.65,1]
				fputc(000, out);
			} else { //occ [0.1,0.65]
				fputc(205, out);
			}
		}
	}

	fclose(out);

	//test
//	std::cout << map->info.width;
//	map->info.height;
//
	/********************修改图片**************************/
	cv::Mat img_rotate = cv::imread( mapdatafile );

	double degree = detectDegree( img_rotate.clone() );
	std::cout << "degree: " << degree << std::endl;
	if ( degree != 1000.0 )
		rotateImage( img_rotate,  degree );
	else
		ROS_WARN( "sth wrong while rotating img..." );
	/********************修改图片**************************/

//	std::string mapmetadatafile = mapname_ + ".yaml";
	ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
	FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

	/*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

 */
	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y,
	                                 orientation.z, orientation.w));
	double yaw, pitch, roll;
	mat.getEulerYPR(yaw, pitch, roll);

	fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
	        mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

	fclose(yaml);
	ROS_INFO("Done\n");
	saved_map_ = true;

	/********************加载图片**************************/
	double last_origin[3];

	last_origin[0] = map->info.origin.position.x;
	last_origin[1] = map->info.origin.position.y;
	last_origin[2] = yaw;

	map_server::loadMapFromFile(&map_resp_, mapdatafile.c_str(), map->info.resolution,
	                            0, 0.65, 0.196, last_origin );

	map_resp_.map.info.map_load_time = ros::Time::now();
	map_resp_.map.header.frame_id = map->header.frame_id;
	map_resp_.map.header.stamp = ros::Time::now();
	ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
	         map_resp_.map.info.width,
	         map_resp_.map.info.height,
	         map_resp_.map.info.resolution);
	/********************返回map格式**************************/

	//建图完成时位置的旋转点
	double th = - degree / 180 * M_PI;  //°C 2 rad
	double rot_x = cos(th) * x_ - sin(th) * y_;
	double rot_y = sin(th) * x_ + cos(th) * y_;
	x_ = rot_x;
	y_ = rot_y;
	th_ += th;
	std::cout << "resize map ok.. " << std::endl;
	return 	map_resp_.map;
}



