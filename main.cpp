//  局部图像特征提取与匹配  
//  Author:  www.icvpr.com  
//  Blog  ： http://blog.csdn.net/icvpr    
  
//#include   
//#include   
#include <opencv2/opencv.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <iostream>
  
using namespace cv;  
using namespace std;  
  
#include <vector>
#include "LocalFeature.h"  
  
int main(int argc, char** argv)  
{  
/*
	if (argc != 6)  
	{  
		cout << "wrong usage!" << endl;  
		cout << "usage: .exe FAST SIFT BruteForce leftImage rightImage" << endl;  
		return -1;  
	}  
	string detectorType = argv[1];  
	string extractorType = argv[2];  
	string matchType = argv[3];  
	string leftImagePath = argv[4];  
	string rightImagePath = argv[5];  

*/
//打开视频文件：其实就是建立一个VideoCapture结构
//        rm<<0.01857,   0.00525,  -0.03124;
        cv::Mat_<double> rm(3,1);
        rm<<0.04034,   0.00395,  -0.00853;
        cv::Mat_<double> Rm(3,3);
        cv::Rodrigues(rm, Rm);
 FileStorage fs("exter.xml", FileStorage::WRITE);
        if(!fs.isOpened())
        {
            printf("Failed to open file ");
            return -1;
        }
fs<<"R"<<Rm;
fs.release();


	VideoCapture capture(argv[1]);
	//检测是否正常打开:成功打开时，isOpened返回ture
	if(!capture.isOpened())
		cout<<"fail to open!"<<endl;
	//获取整个帧数
	long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout<<"整个视频共"<<totalFrameNumber<<"帧"<<endl;

	//设置开始帧()
	long frameToStart = 0;
	capture.set( CV_CAP_PROP_POS_FRAMES,frameToStart);
	cout<<"从第"<<frameToStart<<"帧开始读"<<endl;

	//设置结束帧
	//int frameToStop = 100;
	int frameToStop = totalFrameNumber;

	if(frameToStop < frameToStart)
	{
		cout<<"结束帧小于开始帧，程序错误，即将退出！"<<endl;
		return -1;
	}
	else
	{
		cout<<"结束帧为：第"<<frameToStop<<"帧"<<endl;
	}

	//获取帧率
	double rate = capture.get(CV_CAP_PROP_FPS);
	cout<<"帧率为:"<<rate<<endl;

	//定义一个用来控制读取视频循环结束的变量
	bool stop = false;
	//承载每一帧的图像
	Mat frame;
	//显示每一帧的窗口
	namedWindow("Extracted frame");
	//两帧间的间隔时间:
	int delay = 1000/rate;

	//利用while循环读取帧
	//currentFrame是在循环体中控制读取到指定的帧后循环结束的变量
	long currentFrame = frameToStart;

	//滤波器的核
	int kernel_size = 3;
	Mat kernel = Mat::ones(kernel_size,kernel_size,CV_32F)/(float)(kernel_size*kernel_size);


	long Swidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	long Sheight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

			cout<<"swidth" << Swidth << "sheight=" << Sheight<<endl;

	while(!stop)
	{
		//读取下一帧
		if(!capture.read(frame))
		{
			cout<<"读取视频失败"<<endl;
			return -1;	
		}
		
	//Mat leftImage = imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);  
        
	Mat leftImage = frame(cv::Rect(0,Sheight/4,Swidth/2,Sheight/2) );  
	if (leftImage.empty())  
	{  
		cout<<"read failed"<< endl;  
		return -1;  
	}  
	  
	//Mat rightImage = imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);  
	Mat rightImage = frame(cv::Rect(Swidth/2,Sheight/4,Swidth/2,Sheight/2));  
	if (rightImage.empty())  
	{  
		cout<<"read failed"<< endl;  
		return -1;  
	}  


		//这里加滤波程序
                
		imshow("Extracted frame",frame);
                cvMoveWindow("Extracted frame", 100,100);
		imshow("left frame",leftImage);
                cvMoveWindow("left frame", 100,100);
		imshow("right frame",rightImage);
                cvMoveWindow("right frame", 1100,100);
//		filter2D(frame,frame,-1,kernel);

//		imshow("after filter",frame);

		cout<<"正在读取第"<<currentFrame<<"帧"<<endl;
                if ( currentFrame %5 == 0) {
		int c = waitKey();//当delay ≤ 0时会永远等待；当delay>0时会等待delay毫秒
	//	当时间结束前没有按键按下时，返回值为-1；否则返回按键

	//	int c = waitKey(delay);
		//按下ESC或者到达指定的结束帧后退出读取视频
		if((char) c == 27 || currentFrame > frameToStop)
		{
			stop = true;
		}
		//按下按键后会停留在当前帧，等待下一次按键
		if( c >= 0)
		{
			waitKey(0);
		}
                }
		currentFrame++;

/*
	//Feature feature(detectorType, extractorType, matchType);  
	Feature feature("FAST", "FAST", "BruteForce");  
	  
	std::vector<KeyPoint> queryKeypoints, trainKeypoints;  
	feature.detectKeypoints(leftImage, queryKeypoints);  
	feature.detectKeypoints(rightImage, trainKeypoints);  
	  
	Mat queryDescriptor, trainDescriptor;  
	  
	feature.extractDescriptors(leftImage, queryKeypoints, queryDescriptor);  
	feature.extractDescriptors(rightImage, trainKeypoints, trainDescriptor);  
	  
//	std::vector<cv::DMatch> matches;  
//	feature.bestMatch(queryDescriptor, trainDescriptor, matches);  
	  
//	vector<vector<DMatch> > knnmatches;  
//	feature.knnMatch(queryDescriptor, trainDescriptor, knnmatches, 2);  
	  
//	Mat outImage;  
//	feature.saveMatches(leftImage, queryKeypoints, rightImage, trainKeypoints, matches, "../");  
*/ 
/*
{
  Mat img_1, img_2;
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  //Ptr<SURF> detector = SURF::create( minHessian );
  //Ptr<BRISK> detector = BRISK::create( minHessian );
 cvtColor( leftImage, img_1, COLOR_BGR2GRAY  );
  cvtColor( rightImage, img_2, COLOR_BGR2GRAY );

  Ptr<BRISK> detector = BRISK::create();
 std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
  detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );
  //-- Step 2: Matching descriptor vectors with a brute force matcher
  BFMatcher matcher(NORM_L2);
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );
 //-- Step 2: Matching descriptor vectors using FLANN matcher
 // FlannBasedMatcher matcher;
 // std::vector< DMatch > matches;
 // matcher.match( descriptors_1, descriptors_2, matches );
  double max_dist = 0; double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(3*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }
  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  //-- Show detected matches
  imshow( "Good Matches", img_matches );
  for( int i = 0; i < (int)good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
  //-- Draw matches
  //Mat img_matches;
  //drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_matches );
  //drawMatches( leftImage, keypoints_1, rightImage, keypoints_2, matches, img_matches );
  //-- Show detected matches
  imshow("Matches", img_matches );
/* 

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  cvtColor( leftImage, img_1, COLOR_BGR2GRAY );
  cvtColor( rightImage, img_2, COLOR_BGR2GRAY );


  detector->detect( img_1, keypoints_1 );
  detector->detect( img_2, keypoints_2 );
  //-- Draw keypoints
  Mat img_keypoints_1; Mat img_keypoints_2;
  drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  //-- Show detected (drawn) keypoints
  imshow("Keypoints 1", img_keypoints_1 );
  imshow("Keypoints 2", img_keypoints_2 );
}
*/
	
	}
	//关闭视频文件
	capture.release();
	waitKey(0);
        return 0;
	  
  
	return 0;  
}  

