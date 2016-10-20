#include <iostream>
#include <sstream>
#include <time.h>
#include "stdafx.h"
#include "Depth.h"
#include "Dot.h"
#include "NeonDesign.h"
#include "Log.h"
#include "CatmullSpline.h"
#include "NtKinect.h"
#define HUE 180
#define SPACESIZE 10
#define SCALESIZE 1
#define FILTERSIZE 81

string hstate[] = { "unknown", "nottracked", "Open", "Closed", "Lasso" };
string hconf[] = { "low", "high" };

string str(pair<int, int> p) {
	stringstream ss;
	ss << hstate[p.first] << ":" << hconf[p.second];
	return ss.str();
}

string str(pair<int, int> left, pair<int, int>right) {
	stringstream ss;
	ss << str(left) << " " << str(right);
	return ss.str();
}
void doCatmull(cv::Mat &srcImg, cv::Mat &resultImg, vector<vector<pair<int, int>>> &approximationLine){
	CatmullSpline catmull;
	for (int i = 0; i < approximationLine.size(); i++){
		catmull.drawLine(resultImg, approximationLine[i], HUE);
	}
	cv::blur(resultImg, resultImg, cv::Size(9, 9));
	catmull.drawInline(resultImg, HUE);
}
void doDot(cv::Mat &srcImg, cv::Mat &resultImg){
	Dot dot;
	dot.setWhiteDots(srcImg);
	dot.findStart(srcImg);
	dot.makeLine(srcImg);
	dot.makeSpace(SPACESIZE);
	dot.scalable(SCALESIZE);
	doCatmull(srcImg, resultImg, dot.approximationLine);
}

void main() {
	try {
		Depth depth;
		Log log;
		log.Initialize("logPOINTER.txt");
		//int count = 0;
		while (1) {
			clock_t start = clock();
			depth.setBodyDepth();
			clock_t end = clock();
			//cv::imshow("body index depth", depth.bodyDepthImage);
			depth.setNormalizeDepth(depth.bodyDepthImage);
			//cv::imshow("normalize depth image", depth.normalizeDepthImage);
			depth.setContour(depth.normalizeDepthImage);
			//cv::imshow("contour image", depth.contourImage);
			cv::Mat resultImg = cv::Mat(depth.contourImage.rows, depth.contourImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));

			doDot(depth.contourImage, resultImg);
			//cv::imshow("complete image", depth.contourImage);
		//cv:imwrite("image/img" + to_string(count) + ".png", resultImg);
			cv::imshow("Catmull Spline", resultImg);
		
			//count++;
			auto key = cv::waitKey(20);
			if (key == 'q') break;
		}
	}
	catch (exception& ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
}
