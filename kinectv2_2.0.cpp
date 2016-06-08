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
	clock_t start = clock();
	bezier.bezierLike(forBezier, image);
	clock_t end = clock();
	log.Write("ÅúÅúBezierLike: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	start = clock();
	bezier.drawBezier(forBezier, image, hue);
	end = clock();
	log.Write("ÅúÅúdrawBezier: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
}
void doDot(cv::Mat &image, int hue, Log log){
	Dot dot;
	clock_t start = clock();
	dot.setWhiteDots(image, dot.dots);
	clock_t end = clock();
	log.Write("ÅúsetWhiteDots: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	start = clock();
	dot.findStart(image, dot.dots, dot.start);
	end = clock();
	log.Write("ÅúfindStart: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	start = clock();
	dot.makeLine(dot.contours, dot.start, dot.dots, dot.used, image);
	end = clock();
	log.Write("ÅúmakeLine: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	start = clock();
	dot.makeSpace(dot.contours, dot.forBezier);
	end = clock();
	log.Write("ÅúmakeSpace: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	start = clock();
	dot.scalable(dot.forBezier);
	end = clock();
	log.Write("Åúscalable: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	/*start = clock();
	//dot.writeDots(dot.forBezier, image, dot.dots);
	end = clock();
	log.Write(" forBezier: " + to_string((double)(end - start) / CLOCKS_PER_SEC));
	*/
	doBezier(dot.forBezier, image, hue, log);
	dot.init();
}

void main() {
	try {
		KinectControl kinect;
		Depth depth;
		//Dot dot;
		//Bezier bezier;
		//NeonDesign neon;
		Log log;
		log.Initialize("log.txt");
		while (1) {
			clock_t start = clock();
			depth.setBodyDepth();
			clock_t end = clock();
			cv::imshow("body index depth", depth.bodyDepthImage);
			//intÇ©ÇÁstringÇ™ÇΩÇ…ïœä∑ÇµÇ»Ç´Ç·ÇæÇﬂ
			//log.Write("setBodyDepth:"+((double)(end - start)/CLOCKS_PER_SEC));
			start = clock();
			depth.setNormalizeDepth(depth.bodyDepthImage);
			end = clock();
			cv::imshow("normalize depth image", depth.normalizeDepthImage);
			//log.Write(" setNormalizeDepth: " + to_string((double)(end - start)/CLOCKS_PER_SEC));
			start = clock();
			depth.setContour(depth.normalizeDepthImage);
			end = clock();
			cv::imshow("contour image", depth.contourImage);
			//log.Write(" contourImage: " + to_string((double)(end - start)/CLOCKS_PER_SEC));
			start = clock();
			doDot(depth.contourImage, 60, log);
			end = clock();
			cv::imshow("complete image", depth.contourImage);
			log.Write("complete Image: " + to_string((double)(end - start)/CLOCKS_PER_SEC));
			log.Write("---------------------------------------------");
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
