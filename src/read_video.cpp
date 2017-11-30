/*
 * brief: come on
 * author: Created by bpeer on 17-11-2.
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <boost/timer.hpp>

///home/bpeer/catkin_ws/src/bpeer_sj/database/img

int main( int argc, char** argv )
{
	std::string video_path = "/home/bpeer/catkin_ws/src/bpeer_sj/database/img/s_666_4_1506504125_60.ts";
	cv::VideoCapture capture( video_path );
//	if ( !capture.isOpened() )
//	{
//		std::cout<<"video not open."<<std::endl;
//		return 0;
//	}

	char c;
	int video_detect_name = 0;
	std::string video_begin_stamp;
	std::string video_sum_stamp;
	for( std::string::iterator s_iter = video_path.begin(); s_iter != video_path.end();++s_iter)
	{
		c = *s_iter;
		if( c == '_' || c == '.' )
		{
			video_detect_name++;
//			std::cout << "find _ " << std::endl;
		}
		if ( 5 == video_detect_name && c != '_' )
		{
			video_begin_stamp += c;
		}
		if ( 6 == video_detect_name && c != '_' && c != '.' )
		{
			video_sum_stamp += c;
		}
	}
	std::cout << "video_stamp=>" << video_begin_stamp << std::endl;
	std::cout << "video_sum_stamp=>" << video_sum_stamp << std::endl;

	cv::Mat frame;
	/*
	double t = (double)cv::getTickCount();
	if ( capture.isOpened() )
	{
		boost::timer ti;

		capture >> frame;
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		double fps = 1.0 / t;
		std::cout << " fps : " << fps << std::endl;
		std::cout << " fps2 : " << 1.0 / ti.elapsed() << std::endl;

	}
	 */

	int num_frame = 0;
	std::vector<cv::Mat> video_img;
	std::map<double, cv::Mat> video_data;
	while ( capture.read( frame ) )
	{
		video_img.push_back(frame.clone());
		num_frame++;
	}

	boost::timer tst_1;
	double tmp_time = atoll( video_begin_stamp.c_str() );  // begin time
	double time_interval = num_frame * 1.0 / atoi( video_sum_stamp.c_str() ); // interval: 1/30 => 1/(video_sum_stamp/num_frame)
	for (int j = 0; j < num_frame; ++j) {
		tmp_time += j * time_interval;
		video_data.insert( std::make_pair(tmp_time, video_img[j]) );
	}
	std::cout << "use tst_1: " << tst_1.elapsed() << std::endl;

	std::cout << "size: " << video_data.size() << std::endl;

	//test
	for ( auto tst_img:video_data)
	{
//		std::cout << "doing test .. " << std::endl;
		cv::imshow("test img", tst_img.second );
		cv::waitKey(1);
	}

	std::cout << "num_frame: " << num_frame << std::endl;
	capture.release();
//	//获取当前视频帧率
//	double rate = capture.get(CV_CAP_PROP_FPS);
//	std::cout << "当前视频帧率: " << rate << std::endl;
/*
	long nFrame = capture.get(CV_CAP_PROP_FRAME_COUNT); // 获取总帧数
	std::cout << "视频总帧数: " << nFrame << std::endl;

//	long position = rate * 10 + rate / 30 * 5;
//	capture.set(CV_CAP_PROP_POS_FRAMES, position);
	int be_frame_num = capture.get(CV_CAP_PROP_POS_FRAMES);
	std::cout<<"Frame Num : "<<be_frame_num<<std::endl;

	// hz * s
//	double position=27.687566278 * (3+1.0/27.687566278*1);  // 30*60
//	capture.set(CV_CAP_PROP_POS_FRAMES, position);

//	capture.read( frame );
	boost::timer timer;
	int i = 0;
	while ( capture.read( frame ) )
	{
		i++;
	}
	std::cout << "use time: " << timer.elapsed() << std::endl;

	std::cout << "i: " << i << std::endl;
	double hz =  i * 1.0 / nFrame * rate;
	std::cout << "hz: " << hz << std::endl;

	if ( frame.empty() )
	{
		std::cout << "read null" << std::endl;
		return 0;
	}
//	capture.release();
//
//	cv::imshow( "dist", frame );
//	cv::waitKey(0);

*/

	return 0;
}
