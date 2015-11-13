/*
 * StereoVision.h
 *
 *  Created on: 2015-8-27
 *      Author: fonerzhang
 */

#ifndef STEREOVISION_H_
#define STEREOVISION_H_

//#include <android/log.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d.hpp>
#define  RENDER_DEBUG  1
#define  DUMP_DEPTH 1

#define  LOG_TAG    "StereoVision"
//#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
//#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  printf(__VA_ARGS__)
#define  LOGE(...)  printf(__VA_ARGS__)



enum { STEREO_BM=0, STEREO_BP_OCL=1, STEREO_BM_OCL=2 };
enum { SHOW_DEPTH=0, SHOW_DISP=1 };
enum { HAS_EDGE=0, NO_EDGE=1 };
class OclBM{

public:
	OclBM(int width,int height,int numofdisps=128,int win=15);

	void process(unsigned char* imgBuf);
        void process(cv::Mat Limg, cv::Mat Rimg);
	void getResult(unsigned char* dispImageBuf,float *depth,int &dataSize);

	void getDepth(const cv::Mat& depthMat,float*depth,int &dataSize);
	void getDispImage(unsigned char* dispImage);

	void setStereoMethod(int method) { stereo_method = method; }
	void setShowMethod(int method) { show_method = method; }
	void setEdgeMethod(int method) { edge_method = method; }
	void setMaxZ(double maxValue){maxZ = maxValue;}
private:
	// stereo system intrinsic params
	cv::Mat mapx1,mapy1;
	cv::Mat mapx2,mapy2;
//	cv::Mat mapx,mapy;
	cv::Mat Qw;

	int imgWidth,imgHeight;
	int ndisp,winSize;
	cv::Mat dispImg;
	cv::Mat depthMatrix;

	cv::Mat dispMat;
	cv::Mat dispGrayMat;
	cv::Mat CannyMat;

	double maxZ;
	int stereo_method;
	int show_method;
	int edge_method;

#ifdef DUMP_DEPTH
	int dumpCount;
#endif
};


#endif /* STEREOVISION_H_ */
