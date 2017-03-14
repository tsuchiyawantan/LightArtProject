#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

class Effect{
private:
public:
	Effect(){}
	~Effect(){}
	

	void ororMat(cv::Mat &img1, cv::Mat &img2, cv::Mat &result_img){
		bitwise_or(img1, img2, result_img);
	}
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
};