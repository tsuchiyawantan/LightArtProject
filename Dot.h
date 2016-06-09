#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include <set>

using namespace std;

#define SPACE 10
#define SCALE 1

class Dot{
private:
public:
	set<pair<int, int>> used;
	set<pair<int, int>> dots;
	vector<pair<int, pair<int, int>>> start;
	vector<vector<pair<int, int>>> contours;
	vector<vector<pair<int, int>>> forBezier;
	Dot(){}
	~Dot(){}
	void init(){
		used.clear();
		dots.clear();
		start.clear();
		contours.clear();
		forBezier.clear();
	}
	void scalable(vector<vector<pair<int, int>>> &forBezier){
		for (int i = 0; i < forBezier.size(); i++){
			for (int j = 0; j < forBezier[i].size(); j++){
				int y = (forBezier[i].at(j).first)*SCALE;
				int x = (forBezier[i].at(j).second)*SCALE;
				forBezier[i].at(j) = make_pair(y, x);
			}
		}
	}
	
	void makeSpace(vector<vector<pair<int, int>>> &contours, vector<vector<pair<int, int>>> &forBezier){
		for (int i = 0; i < contours.size(); i++){
			vector<pair<int, int>> newCtr;
			int j = 0;
			newCtr.push_back(make_pair(contours[i].at(0).first, contours[i].at(0).second));
			for (j = SPACE; j < contours[i].size(); j = j + SPACE){
				newCtr.push_back(make_pair(contours[i].at(j).first, contours[i].at(j).second));
			}
			if (j > contours[i].size()) newCtr.push_back(make_pair(contours[i].back().first, contours[i].back().second));
			forBezier.push_back(newCtr);
		}
	}
	
	void setWhiteDots(cv::Mat &image, set<pair<int, int>> &xypair){
		/*	image.forEach<uchar>([&](uchar &p, const int position[]) -> void{
			if (p == 255) xypair.insert(make_pair(position[0], position[1]));
		}
	);*/
		for (int y = 0; y < image.rows; y++){
			for (int x = 0; x < image.cols; x++){
				unsigned int z = (unsigned int)image.at<uchar>(y, x);
				if (z == 255){
					xypair.insert(make_pair(y, x));
				}
			}
		}
	}
	int countW8(cv::Mat& image, int y, int x) {
		int n[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		int count = 0;
		for (int i = 0; i < 8; i++) {
			int dy = y + n[i][0];
			int dx = x + n[i][1];
			if (dy < 0 || dy >= image.rows || dx < 0 || dx >= image.cols) continue;
			if (image.at<uchar>(dy, dx) == 255) count++;
		}
		return count;
	}
	void findStart(cv::Mat &image, set<pair<int, int>> &xypair, vector<pair<int, pair<int, int>>> &start){
		for (auto itr = xypair.begin(); itr != xypair.end(); ++itr) {
			int y = (*itr).first;
			int x = (*itr).second;
			start.push_back(make_pair(countW8(image, y, x), make_pair(y, x)));
		}
		sort(start.begin(), start.end());
	}
	bool isExistS(int y, int x, set<pair<int, int>> &s){
		if (s.find(make_pair(y, x)) == s.end()) return 0;
		return 1;
	}
	bool isExistV(int y, int x, vector<pair<int, int>> &v){
		auto itr = find(v.begin(), v.end(), make_pair(y, x));
		if (itr == v.end()) return 0;
		return 1;
	}
	bool insertYX(cv::Mat &image, vector<pair<int, int>> &ctr, int y, int x, vector<int> &dir, set<pair<int, int>> &used){
		vector<pair<int, int>> n = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		vector<int> j = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7, -7 };
		int i = 0;
		while (i < j.size()){
			int d = dir.back();
			d = d + j.at(i);
			if (d < 0) d = d + 8;
			if (d > 7) d = d % 8;
			int dy = y + n.at(d).first;
			int dx = x + n.at(d).second;
			if (dy < 0 || dy >= image.rows || dx < 0 || dx >= image.cols);
			else if (image.at<uchar>(dy, dx) == 255 && !isExistS(dy, dx, used)) {
				ctr.push_back(make_pair(dy, dx));
				used.insert(make_pair(dy, dx));
				dir.push_back(d);
				return 1;
			}
			i++;
		}
		return 0;
	}
	void check8(set<pair<int, int>> &used, vector<pair<int, int>> &ctr, vector<int> &dir, int y, int x, cv::Mat &image){
		vector<pair<int, int>> n = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		vector<int> j = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7, -7 };
		int i = 0;
		while (i<j.size()) {
			int d = dir.back();
			d = d + j.at(i);
			if (d < 0) d = d + 8;
			if (d > 7) d = d % 8;
			int dy = y + n.at(d).first;
			int dx = x + n.at(d).second;
			if (dy < 0 || dy >= image.rows || dx < 0 || dx >= image.cols);
			else if (image.at<uchar>(dy, dx) == 255 && isExistS(dy, dx, used) && !isExistV(dy, dx, ctr)){
				ctr.push_back(make_pair(dy, dx));
				break;
			}
			i++;
		}
	}
	void makeLine(vector<vector<pair<int, int>>> &contours, vector<pair<int, pair<int, int>>> &start, set<pair<int, int>> &dots, set<pair<int, int>> &used, cv::Mat &image){
		vector<pair<int, int>> ctr;//一つの点列の入れ物
		for (auto itr = start.begin(); itr != start.end(); ++itr){
			//startがすでに使った点でなければ
			if (!isExistS((*itr).second.first, (*itr).second.second, used)){
				vector<int> dir;
				//点のx,y座標
				int y = (*itr).second.first;
				int x = (*itr).second.second;
				//端点であろうx, y
				ctr.push_back(make_pair(y, x));
				used.insert(make_pair(y, x));
				dir.push_back(0);
				//ここで8近傍見る、他の点列の点がいれば、それをctrのスタートの前に入れる
				check8(used, ctr, dir, y, x, image);
				while (insertYX(image, ctr, y, x, dir, used)){
					y = ctr.back().first;
					x = ctr.back().second;
				}
				//ここでも見る。他の点がいればctrの最後に入れる
				check8(used, ctr, dir, y, x, image);
				if (ctr.size()>10)
					contours.push_back(ctr);
				ctr.clear();
				dir.clear();
			}
		}
	}

	void writeDots(vector<vector<pair<int, int>>> &contours, cv::Mat &image, set<pair<int, int>> &set){
		//cv::Mat dotResult(image.rows*SCALE, image.cols*SCALE, CV_8UC3, cv::Scalar(0, 0, 0));
		image = cv::Mat(image.rows*SCALE, image.cols*SCALE, CV_8UC3, cv::Scalar(0));
		srand((unsigned)time(NULL));
		for (int i = 0; i < contours.size(); i++){
			int b = rand() % 256;
			int g = rand() % 256;
			int r = rand() % 256;
			for (auto itr = contours[i].begin(); itr != contours[i].end(); ++itr){
				int y = (*itr).first;
				int x = (*itr).second;
				image.at<cv::Vec3b>(y, x)[0] = 255; // b; //青
				image.at<cv::Vec3b>(y, x)[1] = 255; // g; //緑
				image.at<cv::Vec3b>(y, x)[2] = 255; // r; //赤
			}
		}
			cv::imshow("writeDots", image);
	}
};