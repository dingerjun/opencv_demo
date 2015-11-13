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
#include "StereoVision.h"
  
int main(int argc, char** argv)  
{  

//打开视频文件：其实就是建立一个VideoCapture结构
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



//dingej1 init ob
        OclBM oclbm(Swidth/2, Sheight/2);
        // set stereo method
        oclbm.setStereoMethod(STEREO_BM_OCL);
        oclbm.setShowMethod(SHOW_DISP);
        oclbm.setEdgeMethod(NO_EDGE);
        oclbm.setMaxZ(3.0);


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

  		Mat img_1, img_2;
 		cvtColor( leftImage, img_1, COLOR_BGR2GRAY );
		cvtColor( rightImage, img_2, COLOR_BGR2GRAY );
                oclbm.process(img_1,img_2);



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

	}
	//关闭视频文件
	capture.release();
	waitKey(0);
        return 0;
  
}  

