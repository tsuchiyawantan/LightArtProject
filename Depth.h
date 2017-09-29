﻿#pragma once
#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <atlbase.h>
#include "NtKinect.h"


#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define HIGHT 200
#define WIDTH 200

class Depth : public NtKinect

{
	// ******** depth *******
private:
public:
	int depthMax;
	int depthMin;
	cv::Mat bodyDepthImage;
	cv::Mat normalizeDepthImage;
	cv::Mat contourImage;
	void findDepthMaxMin(int y, int x, int a){
		if (depthMax < a) depthMax = a;
		if (depthMin > a) depthMin = a;
	}
	void setBodyDepth(){
		updateDepthFrame();
		updateBodyIndexFrame();
		bodyDepthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		depthMax = -1;
		depthMin = INFINITE;
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			if (bodyIndexBuffer[i] == 255) {
				bodyDepthImage.at<UINT16>(y, x) = 65535;
			}
			else {
				bodyDepthImage.at<UINT16>(y, x) = depthBuffer[i];
				findDepthMaxMin(y, x, depthBuffer[i]);
			}
		}
	}
	void setNormalizeDepth(cv::Mat &srcImg){
		normalizeDepthImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		for (int i = 0; i < srcImg.rows*srcImg.cols; i++) {
			int y = i / srcImg.cols;
			int x = i % srcImg.cols;
			if (bodyIndexBuffer[i] == 255) normalizeDepthImage.at<UCHAR>(y, x) = 255;
			else {
				normalizeDepthImage.at<UCHAR>(y, x) = (199 * (srcImg.at<UINT16>(y, x)) / (depthMax - depthMin));
			}
		}
	}
	void setContour(cv::Mat &srcImg){
		cv::Mat image2 = srcImg.clone();
		cv::GaussianBlur(image2, image2, cv::Size(5, 5), 0, 0);
		contourImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		cv::Canny(image2, contourImage, 1, 30);

	}
};