/*
 * StereoVision.cpp
 *
 *  Created on: 2015-8-27
 *      Author: fonerzhang
 */

#include "StereoVision.h"
//#include <CL/cl.h>

OclBM::OclBM(int width,int height,int numofdisps,int win){
	edge_method = HAS_EDGE;
	maxZ = 3.0;

	cv::Mat_<double> cameraMatrix1(3,3);
	cameraMatrix1<< 583.41225,  0.       ,  326.75770,
					0.       ,  589.27674,  209.41198,
					0.       ,  0.       ,  1.;

	cv::Mat_<double> disCoeff1(5,1);
	disCoeff1<<-0.00404,   -0.14885,   -0.00176,   0.00102,  0.00000;

	cv::Mat_<double> cameraMatrix2(3,3);
	cameraMatrix2<< 579.99012,  0.       ,  317.64133,
					0.       ,  585.44507,  219.48944,
					0.       ,  0.       ,  1.;

	cv::Mat_<double> disCoeff2(5,1);
	disCoeff2<<0.00698,   -0.16722,   -0.00279,   0.00162,  0.00000;

	cv::Mat_<double> rm(3,1);
	rm<<0.01857,   0.00525,  -0.03124;

	cv::Mat_<double> Rm(3,3);
	cv::Rodrigues(rm, Rm);
//	Rm<< 9.9949590888302986e-01,  3.0555658693688051e-02,  8.6185757455006194e-03,
//		-3.0365830270320297e-02,  9.9931064739905584e-01, -2.1357582889317616e-02,
//		-9.2652295211821528e-03,  2.1085106513243395e-02,  9.9973475172429860e-01;

	cv::Mat_<double> Tv(3,1);
	Tv<<-80.14897,   1.97999,  -1.23445;

	cv::Mat Rr1,Rr2,Pr1,Pr2;
	cv::Rect roi1,roi2;
	cv::stereoRectify(cameraMatrix1, disCoeff1, cameraMatrix2, disCoeff2, cv::Size(width,height), Rm, Tv, Rr1, Rr2, Pr1, Pr2, Qw);
	LOGI("Rr1: %f, %f, %f, %f, %f, %f, %f, %f, %f", Rr1.at<double>(0,0), Rr1.at<double>(0,1), Rr1.at<double>(0,2),
			Rr1.at<double>(1,0), Rr1.at<double>(1,1), Rr1.at<double>(1,2), Rr1.at<double>(2,0), Rr1.at<double>(2,1), Rr1.at<double>(2,2));
	LOGI("Rr2: %f, %f, %f, %f, %f, %f, %f, %f, %f", Rr2.at<double>(0,0), Rr2.at<double>(0,1), Rr2.at<double>(0,2),
			Rr2.at<double>(1,0), Rr2.at<double>(1,1), Rr2.at<double>(1,2), Rr2.at<double>(2,0), Rr2.at<double>(2,1), Rr2.at<double>(2,2));
	LOGI("Pr1: %f, %f, %f, %f, %f, %f, %f, %f, %f", Pr1.at<double>(0,0), Pr1.at<double>(0,1), Pr1.at<double>(0,2),
			Pr1.at<double>(1,0), Pr1.at<double>(1,1), Pr1.at<double>(1,2), Pr1.at<double>(2,0), Pr1.at<double>(2,1), Pr1.at<double>(2,2));
	LOGI("Pr2: %f, %f, %f, %f, %f, %f, %f, %f, %f", Pr2.at<double>(0,0), Pr2.at<double>(0,1), Pr2.at<double>(0,2),
			Pr2.at<double>(1,0), Pr2.at<double>(1,1), Rr2.at<double>(1,2), Pr2.at<double>(2,0), Pr2.at<double>(2,1), Pr2.at<double>(2,2));
	LOGI("Qw: \n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f",
			Qw.at<double>(0,0), Qw.at<double>(0,1), Qw.at<double>(0,2), Qw.at<double>(0,3),
			Qw.at<double>(1,0), Qw.at<double>(1,1), Qw.at<double>(1,2), Qw.at<double>(1,3),
			Qw.at<double>(2,0), Qw.at<double>(2,1), Qw.at<double>(2,2), Qw.at<double>(2,3),
			Qw.at<double>(3,0), Qw.at<double>(3,1), Qw.at<double>(3,2), Qw.at<double>(3,3));
	LOGI("Center difference: %f", -Qw.at<double>(3,3)/Qw.at<double>(3,2));

	cv::initUndistortRectifyMap(cameraMatrix1, disCoeff1, Rr1, Pr1, cv::Size(width,height), CV_16SC2, mapx1, mapy1);
	cv::initUndistortRectifyMap(cameraMatrix2, disCoeff2, Rr2, Pr2, cv::Size(width,height), CV_16SC2, mapx2, mapy2);

	// create images for processing
	imgWidth=width;
	imgHeight=height;
	dispImg.create(height,2*width,CV_8UC4);
	depthMatrix.create(height,width,CV_32FC3);

	ndisp=numofdisps;
	winSize=win;

/*
	// output gpu device information
	cv::ocl::DevicesInfo devInfo;
	int res=cv::ocl::getOpenCLDevices(devInfo);
	if(res==0){
		LOGE("There is no OpenCL here!");
	}else{
		for(int i=0;i<devInfo.size();++i){
			LOGI("deviceProfile: %s",devInfo[i]->deviceProfile.c_str());
			LOGI("deviceVersion: %s",devInfo[i]->deviceVersion.c_str());
			LOGI("deviceName: %s",devInfo[i]->deviceName.c_str());
			LOGI("deviceVendor: %s",devInfo[i]->deviceVendor.c_str());
			LOGI("deviceDriverVersion: %s",devInfo[i]->deviceDriverVersion.c_str());
			LOGI("deviceExtensions: %s",devInfo[i]->deviceExtensions.c_str());
			LOGI("maxWorkGroupSize: %d",devInfo[i]->maxWorkGroupSize);
			LOGI("maxComputeUnits: %d",devInfo[i]->maxComputeUnits);
			LOGI("localMemorySize: %d",devInfo[i]->localMemorySize);
			LOGI("maxMemAllocSize: %d",devInfo[i]->maxMemAllocSize);
			LOGI("deviceVersionMajor: %d",devInfo[i]->deviceVersionMajor);
			LOGI("deviceVersionMinor: %d",devInfo[i]->deviceVersionMinor);
		}
	}
*/

#ifdef DUMP_DEPTH
	dumpCount = 0;
#endif
}

void OclBM::process(unsigned char* imgBuf){

	// convert to gray image
	cv::Mat clrImg(imgHeight,2*imgWidth,CV_8UC4,imgBuf);
	cv::Mat grayImg;

	cv::cvtColor(clrImg,grayImg,CV_BGRA2GRAY);
	LOGI("[colin] Created cv GRAY");

	// remaping image
	cv::Mat Limg=grayImg(cv::Range::all(),cv::Range(0,imgWidth));
	cv::Mat Rimg=grayImg(cv::Range::all(),cv::Range(imgWidth,2*imgWidth));
        process(Limg, Rimg);
}


void OclBM::process(cv::Mat Limg, cv::Mat  Rimg){
	cv::Mat LimgRemap,RimgRemap;
	cv::remap(Limg,LimgRemap,mapx1,mapy1,CV_INTER_LINEAR);
	cv::remap(Rimg,RimgRemap,mapx2,mapy2,CV_INTER_LINEAR);
	LOGI("[colin] Remapped L R");

	if(edge_method==HAS_EDGE)
	{
	    cv::Canny(LimgRemap, CannyMat, 64, 255);
	    cv:threshold(CannyMat, CannyMat, 128, 255,cv::THRESH_TRUNC);
	}

	// block matching ocl
	//cv::Mat dispMat;

	int method=stereo_method;

	switch(method)
	{
	case STEREO_BM:
	{
		//cv::StereoBM *bm;
		Ptr<StereoBM> bm = StereoBM::create(16,9);
		//StereoBM bm;
		bm->compute(LimgRemap,RimgRemap,dispMat);
	}
	case STEREO_BP_OCL:
	{
		cv::ocl::StereoBeliefPropagation bp;
		cv::ocl::oclMat oclLimg,oclRimg,oclDisp;
		oclLimg.upload(LimgRemap);
		oclRimg.upload(RimgRemap);
		LOGI("[colin] ocl uploaded");
		bp(oclLimg,oclRimg,oclDisp);
		LOGI("[colin] ocl bp called");
		oclDisp.download(dispMat);
		LOGI("[colin] ocl downloaded");
	}
	default:
	{
		cv::ocl::StereoBM_OCL bm;
		bm.ndisp=ndisp;
		bm.winSize=winSize;
		bm.avergeTexThreshold = 10;
		cv::ocl::oclMat oclLimg,oclRimg,oclDisp;
		oclLimg.upload(LimgRemap);
		oclRimg.upload(RimgRemap);
		bm(oclLimg,oclRimg,oclDisp);
		oclDisp.download(dispMat);
	}
	}

#ifdef DUMP_DEPTH
	time_t rawtime;
	struct tm *ltime = localtime(&rawtime);
	char fname[256];

	//sprintf(fname, "/sdcard/depth_dump/%d_%04d-%02d-%02d-%02d:%02d:%02d.dep", dumpCount++, ltime->tm_year, ltime->tm_mon, ltime->tm_mday, ltime->tm_hour, ltime->tm_min, ltime->tm_sec);
	sprintf(fname, "/sdcard/depth_dump/%04d_%02d-%02d_%02d-%02d-%02d.depth", dumpCount++,ltime->tm_mon, ltime->tm_mday, ltime->tm_hour, ltime->tm_min, ltime->tm_sec);
	LOGI("out depth file: %s, w:%d, h:%d, continouse: %d", fname, dispMat.cols, dispMat.rows, dispMat.isContinuous());
	std::string fmt("w");
	unsigned short size[2] = {dispMat.cols, dispMat.rows};
	FILE *f=fopen(fname,"wt+");
	if (f)
	{
		fwrite(size, sizeof(unsigned short), 2, f);
		for (int i = 0; i < dispMat.rows; i++)
		{
			fwrite(dispMat.ptr<char>(i), sizeof(char), dispMat.cols, f);

		}
		fclose(f);
	}
	else
	{
		LOGI("open depth file failed");
	}
#endif

    //cv::Mat dispGray;
//	if(method==STEREO_BM_OCL)
//	{
//		dispMat.copyTo(dispGrayMat);
//	}
//	else
	{
		cv::normalize(dispMat,dispGrayMat,0,ndisp,CV_MINMAX,CV_8U);
	}

	// construct image for display
    clrImg.copyTo(dispImg);
	LOGI("[colin] constructed display left");
    cv::Mat dispRight=dispImg(cv::Range::all(),cv::Range(imgWidth,2*imgWidth));
    cv::cvtColor(dispGrayMat,dispRight,CV_GRAY2BGRA);
	LOGI("[colin] constructed display right");
}

static void saveXYZ(const cv::Mat& depthMat,float *depth)
{
    float *pDepth = depth;
    for(int y = 0; y < depthMat.rows; y++)
    {
        for(int x = 0; x < depthMat.cols; x++)
        {
            cv::Vec3f point = depthMat.at<cv::Vec3f>(y, x);
            float *pTmp = pDepth;
            pTmp[0] = point[0]; pTmp[1] = -1*point[1]; pTmp[2] = -1*point[2];
            pDepth+=3;
        }
    }
}

void OclBM::getDepth(const cv::Mat& Disp,float*depth,int& dataSize){
	cv::Mat_<cv::Vec3f> XYZ(Disp.rows,Disp.cols);   // Output point cloud
	cv::Mat_<float> vec_tmp(4,1);
	cv::Mat_<float> vec_Qw(4,4);

	Qw.copyTo(vec_Qw);
	const double max_z = maxZ;
	int nonzero_num = 0;

	unsigned char*cannyMask = CannyMat.data;
	int i = 0;
	for(int y=0; y<Disp.rows; ++y) {
		for(int x=0; x<Disp.cols; ++x) {
			int masklabel = 1;
			if(edge_method == HAS_EDGE)
			{
				masklabel = (int)cannyMask[i];
			}
	        vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=(float)Disp.at<uchar>(y,x); vec_tmp(3)=1;
	        vec_tmp = vec_Qw*vec_tmp;
	        vec_tmp /= vec_tmp(3);
	        cv::Vec3f &point = XYZ.at<cv::Vec3f>(y,x);
	        if(fabs(vec_tmp(2) - max_z) < FLT_EPSILON || fabs(vec_tmp(2)) > max_z|| masklabel == 0)
	        {
	        	point[0] = 0; point[1] = 0; point[2] = 0;
	        }
	        else
	        {
	        	point[0] = vec_tmp(0);point[1] = vec_tmp(1);point[2] = vec_tmp(2);
	            nonzero_num++;
	         }
	        i++;
		}
	}
	saveXYZ(XYZ,depth);
	dataSize = nonzero_num*3;
}


void OclBM::getDispImage(unsigned char* dispImageBuf){
	memcpy(dispImageBuf,dispImg.data,2*imgWidth*imgHeight*4);
}

void OclBM::getResult(unsigned char* dispImageBuf,float *depth,int& dataSize)
{
	if(show_method == SHOW_DISP)
	{
		memcpy(dispImageBuf,dispImg.data,2*imgWidth*imgHeight*4);
	}
	else
	{
		//int datasize = imgHeight*imgWidth*3;
		//depth = new float[datasize];
		memcpy(dispImageBuf,dispImg.data,2*imgWidth*imgHeight*4);
//		LOGI("[colin] before getDepth");
//		getDepth(dispGrayMat,(float *)depth, dataSize);
//		LOGI("[colin] after getDepth");
	}
}
