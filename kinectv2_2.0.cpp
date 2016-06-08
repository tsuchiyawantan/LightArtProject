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
void doBezier(vector<vector<pair<int, int>>> &forBezier, cv::Mat &image, int hue, Log log){
	Bezier bezier;
	bezier.bezierLike(forBezier, image);
	bezier.drawBezier(forBezier, image, hue);
}
void doDot(cv::Mat &image, int hue, Log log){
	Dot dot;
	dot.setWhiteDots(image, dot.dots);
	dot.findStart(image, dot.dots, dot.start);
	dot.makeLine(dot.contours, dot.start, dot.dots, dot.used, image);
	dot.makeSpace(dot.contours, dot.forBezier);
	dot.scalable(dot.forBezier);
	//dot.writeDots(dot.forBezier, image, dot.dots);
	doBezier(dot.forBezier, image, hue, log);
	dot.init();
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
			doDot(depth.contourImage, 60, log);
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
