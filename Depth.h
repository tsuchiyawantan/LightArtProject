#pragma once
#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <atlbase.h>

#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define HIGHT 200
#define WIDTH 200

class Depth : public KinectControl

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
	void setNormalizeDepth(cv::Mat &image){
		normalizeDepthImage = cv::Mat(image.rows, image.cols, CV_8UC1);
		for (int i = 0; i < image.rows*image.cols; i++) {
			int y = i / image.cols;
			int x = i % image.cols;
			if (bodyIndexBuffer[i] == 255) normalizeDepthImage.at<UCHAR>(y, x) = 255;
			else {
				normalizeDepthImage.at<UCHAR>(y, x) = (255 * (image.at<UINT16>(y, x) - depthMin)) / (depthMax - depthMin);
			}
		}
	}
	void setContour(cv::Mat &image){
		cv::Mat image2 = image.clone();
		contourImage = cv::Mat(image.rows, image.cols, CV_8UC1);
		//dilateÇ≈îíÇñcí£
		cv::dilate(image2, image2, cv::Mat(), cv::Point(-1, -1), 1);
		//erodeÇ≈îíÇèkè¨
		cv::erode(image2, image2, cv::Mat(), cv::Point(-1, -1), 3);
		//canny
		cv::Canny(image2, contourImage, 20, 75);
		//cv::Laplacian(image2, contourImage, image2.depth(), 5);
		//threshold(contourImage, contourImage, 150, 255, CV_THRESH_BINARY);
		//threshold(contourImage, contourImage, 130, 255, CV_THRESH_BINARY);
		
	}
};