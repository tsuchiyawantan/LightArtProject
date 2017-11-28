#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"
#include "Node.h"

using namespace std;

class CatmullSpline{
private:
	const double T_SIZE = 0.05;
public:
	vector<vector<pair<int, int>>> catmullLine;
	cv::Mat resultImg;
	int test_count = 0;

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

	void drawInlineHanddraw(cv::Mat &srcImg, vector<vector<Node *>> &node_array, int hue){
		NeonDesign design;
		cv::Mat wow = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC3, cv::Scalar(0, 0, 0));

		int limit = 0;
		int b = 0, g = 0, r = 0;
		for (int i = 0; i < node_array.size(); i++){
			if (node_array[i].size() == 0 || catmullLine[i].size() == 0) continue;
			double size = 0.5;
			int k = 0;
			design.rgb(hue, 100, 255, b, g, r);

			for (int j = 0; j < node_array[i].size() - 1; j++){
				Node *first_node = node_array[i].at(j);
				Node *next_node = node_array[i].at(j + 1);

				if (!first_node->isAngularNode() && next_node->isAngularNode()){
					limit = k + (1.0 / T_SIZE);
					while (k < limit - 1){
						if (size < 4.0) size += 0.1;
						int first_y = catmullLine[i].at(k).first;
						int first_x = catmullLine[i].at(k).second;
						int next_y = catmullLine[i].at(k + 1).first;
						int next_x = catmullLine[i].at(k + 1).second;
						cv::line(srcImg, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						//	cv::line(wow, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						k++;
					}

				}
				else if (first_node->isAngularNode() && next_node->isAngularNode()){
					limit = k + (1.0 / T_SIZE);
					size = 4.0;
					while (k < limit - 1){
						int first_y = catmullLine[i].at(k).first;
						int first_x = catmullLine[i].at(k).second;
						int next_y = catmullLine[i].at(k + 1).first;
						int next_x = catmullLine[i].at(k + 1).second;
						cv::line(srcImg, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						//cv::line(wow, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						k++;
					}
				}
				else if (first_node->isAngularNode() && !next_node->isAngularNode()){
					limit = k + (1.0 / T_SIZE);
					while (k < limit - 1){
						if (size > 0.5) size -= 0.1;
						int first_y = catmullLine[i].at(k).first;
						int first_x = catmullLine[i].at(k).second;
						int next_y = catmullLine[i].at(k + 1).first;
						int next_x = catmullLine[i].at(k + 1).second;
						cv::line(srcImg, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						//	cv::line(wow, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						k++;
					}
				}
				else {
					limit = k + (1.0 / T_SIZE);
					size = 0.5;
					while (k < limit - 1){
						int first_y = catmullLine[i].at(k).first;
						int first_x = catmullLine[i].at(k).second;
						int next_y = catmullLine[i].at(k + 1).first;
						int next_x = catmullLine[i].at(k + 1).second;
						cv::line(srcImg, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						//cv::line(wow, cv::Point(first_x, first_y), cv::Point(next_x, next_y), cv::Scalar(b, g, r), size, 4);
						k++;
					}
				}
			}
		}
		//cv::imwrite("inlineimage/image" + to_string(test_count++) + ".png", wow);
		//cv::imshow("HAND-DRAWING-LIKE IMAGE", wow);
	}

	void drawInline(cv::Mat &srcImg, vector<vector<Node *>> node_array, int hue){
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

	void drawLine(cv::Mat &resultImg, vector<vector<Node *>> node_array, int hue){
		NeonDesign design;
		int b = 0, g = 0, r = 0;
		double size = 5;
		vector<pair<int, int>> ctr;
		cv::Point first;
		cv::Point second;
		cv::Point third;
		cv::Point forth;
		design.rgb(hue, 255 - 130, 70, b, g, r);

		for (int i = 0; i < node_array.size(); i++){
			ctr.clear();
			for (int j = 0; j < node_array[i].size(); j++){
				Node *node = node_array[i].at(j);
				int y = (*node).getNodeY();
				int x = (*node).getNodeX();
				if (j >= node_array[i].size() || j + 1 >= node_array[i].size() || j + 2 >= node_array[i].size() || j + 3 >= node_array[i].size()) break;
				if (j == 0){ //始点
					Node *first_node = node_array[i].at(0);
					Node *second_node = node_array[i].at(1);
					first.y = (*first_node).getNodeY();
					first.x = (*first_node).getNodeX();
					second.y = (*second_node).getNodeY();
					second.x = (*second_node).getNodeX();
					for (double t = 0; t <= 1.0; t += T_SIZE){
						y = catmullRomFirstLast(first.y, second.y, t);
						x = catmullRomFirstLast(first.x, second.x, t);
						ctr.push_back(make_pair(y, x));
						circle(resultImg, cv::Point(x, y), size, cv::Scalar(b, g, r), -1, 8);
					}
				}
				/*if (node_array[i].size() <= 2) break;
				if (node_array[i].size() == 3){
				Node *third_node = node_array[i].at(node_array[i].size() - 2);
				Node *forth_node = node_array[i].at(node_array[i].size() - 1);
				third.y = (*third_node).getNodeY();
				third.x = (*third_node).getNodeX();
				forth.y = (*forth_node).getNodeY();
				forth.x = (*forth_node).getNodeX();
				for (double t = 0; t <= 1.0; t += T_SIZE){
				y = catmullRomFirstLast(third.y, forth.y, t);
				x = catmullRomFirstLast(third.x, forth.x, t);
				ctr.push_back(make_pair(y, x));
				circle(resultImg, cv::Point(x, y), 5, cv::Scalar(b, g, r), -1, 8);
				}
				break;
				}*/
				Node *first_node = node_array[i].at(j);
				Node *second_node = node_array[i].at(j + 1);
				Node *third_node = node_array[i].at(j + 2);
				Node *forth_node = node_array[i].at(j + 3);
				first.y = (*first_node).getNodeY();
				first.x = (*first_node).getNodeX();
				second.y = (*second_node).getNodeY();
				second.x = (*second_node).getNodeX();
				third.y = (*third_node).getNodeY();
				third.x = (*third_node).getNodeX();
				forth.y = (*forth_node).getNodeY();
				forth.x = (*forth_node).getNodeX();

				for (double t = 0; t <= 1.0; t += T_SIZE){
					y = catmullRom(first.y, second.y, third.y, forth.y, t);
					x = catmullRom(first.x, second.x, third.x, forth.x, t);
					ctr.push_back(make_pair(y, x));
					circle(resultImg, cv::Point(x, y), size, cv::Scalar(b, g, r), -1, 8);
				}
				if (j == node_array[i].size() - 4){ //（終点-4）番目
					//size = 5.5;
					Node *third_node = node_array[i].at(node_array[i].size() - 2);
					Node *forth_node = node_array[i].at(node_array[i].size() - 1);
					third.y = (*third_node).getNodeY();
					third.x = (*third_node).getNodeX();
					forth.y = (*forth_node).getNodeY();
					forth.x = (*forth_node).getNodeX();
					for (double t = 0; t <= 1.0; t += T_SIZE){
						y = catmullRomFirstLast(third.y, forth.y, t);
						x = catmullRomFirstLast(third.x, forth.x, t);
						ctr.push_back(make_pair(y, x));
						circle(resultImg, cv::Point(x, y), 5, cv::Scalar(b, g, r), -1, 8);
					}
					break;
				}
			}
			catmullLine.push_back(ctr);
		}
	}
};