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
#include "Effect.h"
#define HUE 10
#define SPACESIZE 10
#define SCALESIZE 1
#define FILTERSIZE 81
#define AFTER_FRAME 5

Dot dot;
CatmullSpline catmull;
Effect effect;


void doCatmull(cv::Mat &srcImg, cv::Mat &resultImg, vector<vector<pair<int, int>>> &approximationLine){
	catmull.init();
	for (int i = 0; i < approximationLine.size(); i++){
		catmull.drawLine(resultImg, approximationLine[i], HUE);
	}
	//cv::blur(resultImg, resultImg, cv::Size(9, 9));
	catmull.drawInline(resultImg, HUE);
}
void doDot(cv::Mat &srcImg, cv::Mat &resultImg){
	dot.init();
	dot.setWhiteDots(srcImg);
	dot.findStart(srcImg);
	dot.makeLine(srcImg);
	dot.makeSpace(SPACESIZE);
	dot.scalable(SCALESIZE);
	doCatmull(srcImg, resultImg, dot.approximationLine);
}

void doEffect(cv::Mat &src_img, vector<cv::Mat> &afterimg_array){
	cv::Mat tmp_img = src_img.clone();
	cv::Mat src_multi_img = src_img.clone();

	if (afterimg_array.size() != 0){
		tmp_img = afterimg_array.at(0);
		//afterimg_arrayに入ってる画像とsrcをor演算子でひとまとめにする。tmp_imgで返す
		//afterimg_array配列に1/Xを足し算していく
		for (int i = 0; i < afterimg_array.size(); i++){
			bitwise_or(tmp_img, afterimg_array.at(i), tmp_img);
		}

		if(afterimg_array.size() > AFTER_FRAME) afterimg_array.erase(afterimg_array.begin());

		//afterimg_array配列に1/Xを足し算していく
		for (int i = 0; i < afterimg_array.size(); i++){
			effect.applyFilteringAdd(afterimg_array.at(i), 1.0 / AFTER_FRAME);

		}
	}
	effect.applyFilteringMulti(src_img, src_multi_img, 1.0 / AFTER_FRAME);
	afterimg_array.push_back(src_multi_img);
	src_img = tmp_img;
}
void main() {
	try {
		Depth depth;
		Log log;
		log.Initialize("logPOINTER.txt");
		int count = 1;
		cv::Mat result_img;
		vector<cv::Mat> afterimg_array;
		cv::Mat black_img;
		while (1) {
			clock_t start = clock();
			depth.setBodyDepth();
			clock_t end = clock();
			//cv::imshow("body index depth", depth.bodyDepthImage);
			depth.setNormalizeDepth(depth.bodyDepthImage);
			//cv::imshow("normalize depth image", depth.normalizeDepthImage);
			depth.setContour(depth.normalizeDepthImage);
			//cv::imshow("contour image", depth.contourImage);
			result_img = cv::Mat(depth.contourImage.rows, depth.contourImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));
			doDot(depth.contourImage, result_img);
			//cv::imshow("complete image", depth.contourImage);
			//cv:imwrite("image/img" + to_string(count) + ".png", resultImg);
			cv::imshow("before afterimg", result_img);

			//if (count<AFTER_FRAME){
			//	cv::Mat resultimg_cp = result_img.clone();
			//	//RGBに1/3、2/3かける関数
			//	effect.applyFilteringMulti(result_img, resultimg_cp, (double)((AFTER_FRAME-count)/AFTER_FRAME));
			//	//cv::GaussianBlur(resultimg_cp, resultimg_cp, cv::Size(11, 11), 0, 0);
			//	afterimg_array.push_back(resultimg_cp);
			//}
			//else
				doEffect(result_img, afterimg_array);

			cv::imshow("Catmull Spline", result_img);
			count++;
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