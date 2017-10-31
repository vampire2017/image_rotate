//
// Created by bpeer on 17-10-23.
//

//double detectDegree( cv::Mat img )
//{
//	double max_angle = 90;
//	if ( img.empty() )
//		return NAN;
//
//	cv::Mat output_gray, output_img;
//
//	//【1】将原图像转换为灰度图像
//	cv::cvtColor( img, output_gray, CV_BGR2GRAY );
////	cv::imshow("gray", input_gray);
////	cv::waitKey(1000);
//	std::cout << "test..." << img.type() << std::endl;
//
//	//【2】先用使用 3x3内核来降噪
//	cv::blur( output_gray, output_img, cv::Size(3,3) );
//
//	//【3】执行Canny边缘检测
//	cv::Canny( output_img, output_img, 50, 150, 3 );
////	cv::imshow("dst",output_img);
////	cv::waitKey(1000);
//
//	//【4】执行概率霍夫线变换，检测直线
//	std::vector< cv::Vec4i > lines;  //定义一个矢量结构lines用于存放得到的线段矢量集合
//	double minLineLength = std::min(output_img.cols, output_img.rows) * 0.02; //最短线长度
//	double maxLineGap = std::min(output_img.cols, output_img.rows) * 0.03 ; //最小线间距
//	int threshold = 100;
//	cv::HoughLinesP( output_img, lines, 1, M_PI/180, threshold, minLineLength, maxLineGap );
//
//	if ( lines.empty() )
//		return NAN;
//	std::cout << "num:= " << lines.size() << std::endl;
//
////	//【5】依次在图中绘制出每条线段
////	for( size_t i = 0; i < lines.size(); i++ )
////	{
////		cv::Vec4i l = lines[i];
////		cv::line( output_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,255), 2, CV_AA);
////	}
////	cv::imshow("HoughLinesP..",output_img);
//
//	//【6】筛选直线
//	int x_s, y_s, x_e, y_e;
//	double angle;   //直线的角度
//	double best_angle = 0.0; //最优角度
//	double line_length, max_length = 0.0; //直线长度
//	cv::Vec4i best_line_test;  //for test--show
//
//	for( size_t i = 0; i < lines.size(); i++ )
//	{
//		std::cout << "lines:= " << lines[i] << std::endl;
//
//		x_s = lines[i][0];
//		y_s = lines[i][1];
//		x_e = lines[i][2];
//		y_e = lines[i][3];
//
////		//计算线的长度  计算第一第二长度
////		line_length = sqrt( (x_s-x_e) * (x_s-x_e) + (y_s-y_e) * (y_s-y_e) );
////		std::cout << "line_length:= " << line_length << std::endl;
////		if( line_length > max_length2nd )
////			if ( line_length > max_length1nd )
////			{
////				//计算直线角度(-90 -- 90)
////				angle = (x_s == x_e) ? 90 : ( atan( (y_e-y_s)*1.0/(x_e-x_s) ) / M_PI * 180 );
////				std::cout << "< angle:= " << angle << " >" << std::endl;
////
////				max_length2nd = max_length1nd;
////				max_length1nd = line_length;
////
////				best_angle2nd = best_angle1nd;
////				best_angle1nd = angle;
////
////				best_line_test1nd = lines[i];
////			}
////			else
////			{
////				//计算直线角度(-90 -- 90)
////				angle = (x_s == x_e) ? 90 : ( atan( (y_e-y_s)*1.0/(x_e-x_s) ) / M_PI * 180 );
////				std::cout << "< angle:= " << angle << " >" << std::endl;
////
////				max_length2nd = line_length;
////				best_angle2nd = angle;
////				best_line_test2nd = lines[i];
////			}
//
//		//计算线的长度
//		line_length = sqrt( (x_s-x_e) * (x_s-x_e) + (y_s-y_e) * (y_s-y_e) );
//		std::cout << "line_length:= " << line_length << std::endl;
//		if( line_length > max_length )
//		{
//			//计算直线角度(-90 -- 90)
//			angle = (x_s == x_e) ? 90 : ( atan( (y_e-y_s)*1.0/(x_e-x_s) ) / M_PI * 180 );
//			std::cout << "< angle:= " << angle << " >" << std::endl;
//
//			max_length = line_length;
//			best_angle = angle;
//
//			best_line_test = lines[i];
//		}
//	}
//
//	cv::line( output_img, cv::Point(best_line_test[0], best_line_test[1]),
//	          cv::Point(best_line_test[2],best_line_test[3]),
//	          cv::Scalar(186,88,255), 4, CV_AA);
//	cv::imshow("HoughLinesP..",output_img);
//	cvWaitKey(10000);
//
//	//【7】计算旋转角度
//	double rotating_angle;
//	if ( 90 == std::fabs(best_angle) || std::fabs(best_angle) < 1e-5 )
//		rotating_angle = 0;
//	else if ( std::fabs(best_angle) < 45 )
//		rotating_angle = (-best_angle);
//	else
//		rotating_angle = ( 90 - std::fabs(best_angle) ) * best_angle / std::fabs(best_angle);
//
//	return rotating_angle;
//
//}
//
//void rotateImage( cv::Mat img, double degree )
//{
//	std::cout << "rotate angle:= " <<  degree << std::endl;
//	cv::Mat img_rotate;
//	cv::Mat img_rotate_dst = img.clone();
//	cv::Mat rot_mat( 2, 3, CV_32FC1 );
//
//	//得到图像大小
//	int width = img.cols;
//	int height = img.rows;
//
////	cv::Mat dst = img.clone();  test1
//
//	//【1】计算图像中心点
//	cv::Point2f center;
//	center.x = width / 2.0;
//	center.y = height / 2.0;
//
//	//【2】获得旋转变换矩阵
//	double scale = 1.0;
//	cv::Mat trans_mat = getRotationMatrix2D( center, -degree, scale );
//
//	//【3】仿射变换
//	cv::warpAffine( img, img_rotate_dst, trans_mat, cv::Size(width, height), 1, 1 );
//
//
////	img_rotate_dst.copyTo( dst, img_rotate_dst );  test2
//
//	cv::imshow( "src", img );
//
//	cv::imshow( "dist", img_rotate_dst );
//
////	cv::imshow( "catjian", dst );  test3
//	cvWaitKey(0);
//}

#include "image_rotate/image_rotate.h"

int main( int argc, char** argv )
{
	if( argc !=2 )
	{
		std::cout << "need map***********" << std::endl;
		return 1;
	}
	std::string input_img = argv[1];
	ImageRotate imageRotate;
//	ros::init( argc, argv, "image_rotate_node" );
//	ros::NodeHandle nh_;

	cv::Mat img_rotate = cv::imread( input_img );

	double degree = imageRotate.detectDegree( img_rotate );
	//@todo here 需要对degree进行判断
	imageRotate.rotateImage( img_rotate,  degree );

//	ros::spin();
	return 0;
}