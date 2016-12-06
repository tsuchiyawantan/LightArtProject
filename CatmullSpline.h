#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"

using namespace std;

class CatmullSpline{
private:
public:
	vector<vector<pair<int, int>>> catmullLine;
	cv::Mat resultImg;

	CatmullSpline(){}
	~CatmullSpline(){}

	void init(){
		catmullLine.clear();
	}
	double catmullRom(double p0, double p1, double p2, double p3, double t){
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	double catmullRomFirstLast(double p1, double p2, double t){
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p1)*t + (2 * p1 - 5 * p1 + 4 * p2 - p2)*t2 + (-p1 + 3 * p1 - 3 * p2 + p2)*t3);
	}
	void adjust(vector<pair<int, int>> &yx){
		int j = yx.size() + (4 - yx.size() % 4);
		while (yx.size() < j){
			yx.push_back(make_pair(yx.back().first, yx.back().second));
		}
	}

	boolean check8(cv::Mat& srcImg, int y, int x) {
		int n[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		int count = 0;
		for (int i = 0; i < 8; i++) {
			int dy = y + n[i][0];
			int dx = x + n[i][1];
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			if (srcImg.at<cv::Vec3b>(dy, dx)[0] != 255 &&
				srcImg.at<cv::Vec3b>(dy, dx)[1] != 255 &&
				srcImg.at<cv::Vec3b>(dy, dx)[2] != 255) return true;
		}
		return false;
	}

	void drawInline(cv::Mat &srcImg, int hue){
		NeonDesign design;
		int b = 0, g = 0, r = 0;
		design.rgb(hue, 255 - 100, 255, b, g, r);
		for (int i = 0; i < catmullLine.size(); i++){
			for (int j = 0; j < catmullLine[i].size(); j++){
				int y = catmullLine[i].at(j).first;
				int x = catmullLine[i].at(j).second;
				circle(srcImg, cv::Point(x, y), 0.5, cv::Scalar(b, g, r), -1, 4);
			}
		}
	}

	void doGaussian(vector<pair<int, int>> &catCtr, cv::Mat &srcImg){
		int mid = catCtr.size() / 2;
		int filtersize = mid;
		int y = catCtr.at(mid).first;
		int x = catCtr.at(mid).second;
		int x1 = x - filtersize;
		int y1 = y - filtersize;
		int y2 = filtersize * 2;
		int x2 = filtersize * 2;

		fixSize(y1, x1, srcImg);
		fixSize2(y1, x1, y2, x2, srcImg);

		cv::Mat regionOfImage(srcImg, cv::Rect(x1, y1, x2, y2));
		cv::GaussianBlur(regionOfImage, regionOfImage, cv::Size(11, 11), 0, 0);
	}

	void fixSize(int &y, int &x, cv::Mat &srcImg){
		if (x < 0) { x = 0; }
		if (y < 0) { y = 0; }
		if (x >= srcImg.cols){ x = srcImg.cols - 1; }
		if (y >= srcImg.rows){ y = srcImg.rows - 1; }
	}
	void fixSize2(int y1, int x1, int &y_sub, int &x_sub, cv::Mat &srcImg){
		int y2 = y1 + y_sub;
		int x2 = x1 + x_sub;

		if (y2 >= srcImg.rows) {
			y2 = srcImg.rows - 1;
			y_sub = y2 - y1;
		}
		if (x2 >= srcImg.cols) {
			x2 = srcImg.cols - 1;
			x_sub = x2 - x1;
		}
	}

	void drawLine(cv::Mat &resultImg, vector<pair<int, int>> &contours, int hue){
		NeonDesign design;
		int b = 0, g = 0, r = 0;
		vector<pair<int, int>> ctr;

		design.rgb(hue, 255, 255 - 100, b, g, r);
		for (int i = 0; i < contours.size(); i++){
			int y = contours.at(i).first;
			int x = contours.at(i).second;
			if (i >= contours.size() || i + 1 >= contours.size() || i + 2 >= contours.size() || i + 3 >= contours.size()) break;
			if (i == 0){
				for (double t = 0; t <= 1.0; t += 0.1){
					y = catmullRomFirstLast(contours.at(0).first, contours.at(1).first, t);
					x = catmullRomFirstLast(contours.at(0).second, contours.at(1).second, t);
					ctr.push_back(make_pair(y, x));
					circle(resultImg, cv::Point(x, y), 5, cv::Scalar(b, g, r), -1, 8);
				}
			}
			for (double t = 0; t <= 1.0; t += 0.05){
				y = catmullRom(contours.at(i).first, contours.at(i + 1).first, contours.at(i + 2).first, contours.at(i + 3).first, t);
				x = catmullRom(contours.at(i).second, contours.at(i + 1).second, contours.at(i + 2).second, contours.at(i + 3).second, t);
				ctr.push_back(make_pair(y, x));
				circle(resultImg, cv::Point(x, y), 5, cv::Scalar(b, g, r), -1, 8);
			}
			if (i == contours.size() - 4){
				for (double t = 0; t <= 1.0; t += 0.1){
					y = catmullRomFirstLast(contours.at(contours.size() - 2).first, contours.at(contours.size() - 1).first, t);
					x = catmullRomFirstLast(contours.at(contours.size() - 2).second, contours.at(contours.size() - 1).second, t);
					ctr.push_back(make_pair(y, x));
					circle(resultImg, cv::Point(x, y), 5, cv::Scalar(b, g, r), -1, 8);
				}
			}

		}
		catmullLine.push_back(ctr);
	}
};