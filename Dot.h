#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include <set>

using namespace std;

class Dot{
private:
public:
	set<pair<int, int>> used_dots;
	set<pair<int, int>> white_dots;
	vector<pair<int, pair<int, int>>> priority_start;
	vector<vector<pair<int, int>>> contours;
	vector<vector<pair<int, int>>> approximation_line;
	vector<vector<cv::Point>> divide_contours;

	Dot(){
		init();
	}
	~Dot(){}

	void init(){
		used_dots.clear();
		white_dots.clear();
		priority_start.clear();
		contours.clear();
		approximation_line.clear();
		divide_contours.clear();
		
		priority_start.shrink_to_fit();
		contours.shrink_to_fit();
		approximation_line.shrink_to_fit();
		divide_contours.shrink_to_fit();
	}

	void scalable(int scaleSize){
		for (int i = 0; i < approximation_line.size(); i++){
			for (int j = 0; j < approximation_line[i].size(); j++){
				int y = (approximation_line[i].at(j).first)*scaleSize;
				int x = (approximation_line[i].at(j).second)*scaleSize;
				approximation_line[i].at(j) = make_pair(y, x);
			}
		}
	}

	void makeSpace(int spaceSize){
		for (int i = 0; i < contours.size(); i++){
			vector<pair<int, int>> ctr;
			int j = 0;
			ctr.push_back(make_pair(contours[i].at(0).first, contours[i].at(0).second));
			for (j = spaceSize; j < contours[i].size(); j = j + spaceSize){
				ctr.push_back(make_pair(contours[i].at(j).first, contours[i].at(j).second));
			}
			if (j > contours[i].size()) ctr.push_back(make_pair(contours[i].back().first, contours[i].back().second));
			approximation_line.push_back(ctr);
		}
	}

	void setWhiteDots(cv::Mat &srcImg){
		for (int y = 0; y < srcImg.rows; y++){
			unsigned char *p = &srcImg.at<uchar>(y, 0);
			for (int x = 0; x < srcImg.cols; x++){
				if (*p == 255){
					white_dots.insert(make_pair(y, x));
				}
				p++;
			}
		}
	}
	int countW8(cv::Mat& srcImg, int y, int x) {
		int n[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		int count = 0;
		for (int i = 0; i < 8; i++) {
			int dy = y + n[i][0];
			int dx = x + n[i][1];
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			if (srcImg.at<uchar>(dy, dx) == 255) count++;
		}
		return count;
	}
	void findStart(cv::Mat &srcImg){
		for (auto itr = white_dots.begin(); itr != white_dots.end(); ++itr) {
			int y = (*itr).first;
			int x = (*itr).second;
			priority_start.push_back(make_pair(countW8(srcImg, y, x), make_pair(y, x)));
		}
		sort(priority_start.begin(), priority_start.end());
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
	bool insertYX(cv::Mat &srcImg, vector<pair<int, int>> &ctr, int y, int x, vector<int> &dir){
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
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols);
			else if (srcImg.at<uchar>(dy, dx) == 255 && !isExistS(dy, dx, used_dots)) {
				ctr.push_back(make_pair(dy, dx));
				used_dots.insert(make_pair(dy, dx));
				dir.push_back(d);
				return 1;
			}
			i++;
		}
		return 0;
	}

	void checkUsed8(cv::Mat &srcImg, vector<pair<int, int>> &ctr, vector<int> &dir, int y, int x){
		vector<pair<int, int>> n = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		vector<int> j = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 7, -7 };
		int i = 0;
		while (i < j.size()) {
			int d = dir.back();
			d = d + j.at(i);
			if (d < 0) d = d + 8;
			if (d > 7) d = d % 8;
			int dy = y + n.at(d).first;
			int dx = x + n.at(d).second;
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols);
			else if (srcImg.at<uchar>(dy, dx) == 255 && isExistS(dy, dx, used_dots) && !isExistV(dy, dx, ctr)){
				ctr.push_back(make_pair(dy, dx));
				break;
			}
			i++;
		}
	}

	void makeLine(cv::Mat &srcImg){
		vector<pair<int, int>> ctr;//一つの点列の入れ物
		vector<int> dir;
		for (auto itr = priority_start.begin(); itr != priority_start.end(); ++itr){
			//startがすでに使った点でなければ
			if (!isExistS((*itr).second.first, (*itr).second.second, used_dots)){
				int y = (*itr).second.first;
				int x = (*itr).second.second;
				//端点であろうx, y
				ctr.push_back(make_pair(y, x));
				used_dots.insert(make_pair(y, x));
				dir.push_back(0);
				//ここで8近傍見る、他の点列の点がいれば、それをctrのスタートの前に入れる
				checkUsed8(srcImg, ctr, dir, y, x);
				while (insertYX(srcImg, ctr, y, x, dir)){
					y = ctr.back().first;
					x = ctr.back().second;
				}
				//ここでも見る。他の点がいればctrの最後に入れる
				checkUsed8(srcImg, ctr, dir, y, x);
				if (ctr.size() > 40)
					contours.push_back(ctr);
				ctr.clear();
				dir.clear();
			}
		}
	}

	//点をspaceSizeだけ間引く
	void divideCon(int spaceSize){
		for (int i = 0; i < contours.size(); i++){
			vector<cv::Point> ctr;

			for (int j = 0; j < contours[i].size(); j = j + spaceSize){
				ctr.push_back(cv::Point(contours[i].at(j).second, contours[i].at(j).first));
			}
			ctr.push_back(cv::Point(contours[i].back().second, contours[i].back().first));
			divide_contours.push_back(ctr);
		}
	}
};