#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

#define AFTER_FRAME 3

class Effect{
private:
public:
	Effect(){}
	~Effect(){}
	
	void applyFilteringMulti(cv::Mat &src_img, cv::Mat &result_img, double filter){
		for (int i = 0; i < src_img.rows; i++){
			for (int j = 0; j < src_img.cols; j++){
				cv::Vec3b *src = src_img.ptr<cv::Vec3b>(i); //i行目の先頭画素のポインタを取得
				cv::Vec3b *rsl = result_img.ptr<cv::Vec3b>(i); //i行目の先頭画素のポインタを取得
				src[j]; //j番目にアクセス
				rsl[j][0] = src[j][0] * filter;
				rsl[j][1] = src[j][1] * filter;
				rsl[j][2] = src[j][2] * filter;
			}
		}
	}

	void applyFilteringAdd(cv::Mat &src_img, double filter){
		for (int i = 0; i < src_img.rows; i++){
			for (int j = 0; j < src_img.cols; j++){
				cv::Vec3b *src = src_img.ptr<cv::Vec3b>(i); //i行目の先頭画素のポインタを取得
				src[j]; //j番目にアクセス
				src[j][0] = src[j][0] + filter;
				src[j][1] = src[j][1] + filter;
				src[j][2] = src[j][2] + filter;
			}
		}
	}

	void addAfterImg(cv::Mat &src_img, vector<cv::Mat> &afterimg_array){
		cv::Mat src_multi_img = src_img.clone();

		if (afterimg_array.size() != 0){
			if (afterimg_array.size() > AFTER_FRAME) afterimg_array.erase(afterimg_array.begin());
			//afterimg_array配列に1/Xを足し算していく
			for (int i = 0; i < afterimg_array.size(); i++){
			//	applyFilteringAdd(afterimg_array.at(i), 1.0 / AFTER_FRAME);
			}
		}
		applyFilteringMulti(src_img, src_multi_img, 1.0 / AFTER_FRAME);
		afterimg_array.push_back(src_multi_img);
	}
};