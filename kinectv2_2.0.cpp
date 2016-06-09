#include <iostream>
#include <sstream>
#include <time.h>
#include "stdafx.h"
#include "KinectControl.h"
#include "Depth.h"
#include "Dot.h"
#include "Bezier.h"
#include "NeonDesign.h"
#include "Log.h"
#define HUE 60
#define SPACESIZE 10
#define SCALESIZE 1

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
void doBezier(vector<vector<pair<int, int>>> &contours, cv::Mat &srcImg){
	Bezier bezier;
	bezier.bezierLike(srcImg, contours);
	bezier.drawBezier(contours, srcImg, HUE);
}
void doDot(cv::Mat &srcImg){
	Dot dot;
	dot.setWhiteDots(srcImg);
	dot.findStart(srcImg);
	dot.makeLine(srcImg);
	dot.makeSpace(SPACESIZE);
	dot.scalable(SCALESIZE);
	doBezier(dot.forBezier, srcImg);
}

void main() {
	try {
		KinectControl kinect;
		Depth depth;
		Log log;
		log.Initialize("log.txt");
		log.Write("---------------------------------------------");
		while (1) {
			clock_t start = clock();
			depth.setBodyDepth();
			clock_t end = clock();
			cv::imshow("body index depth", depth.bodyDepthImage);
			depth.setNormalizeDepth(depth.bodyDepthImage);
			cv::imshow("normalize depth image", depth.normalizeDepthImage);
			depth.setContour(depth.normalizeDepthImage);
			cv::imshow("contour image", depth.contourImage);
			doDot(depth.contourImage);
			cv::imshow("complete image", depth.contourImage);
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
