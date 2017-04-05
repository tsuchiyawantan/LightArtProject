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
#include "Node.h"
#include "Graph.h"

#define HUE 80
#define SPACESIZE 10
#define EFFECT_FLAG 1

Dot dot;
CatmullSpline catmull;
Graph graph;
Effect effect;
int test_count = 1;

void doCatmull(cv::Mat &result_img, vector<vector<Node *>> node_array){
	catmull.init();
	catmull.drawLine(result_img, node_array, HUE);
	catmull.drawInline(result_img, HUE);
}

void doGraph(cv::Mat &src_img, vector<vector<Node *>> &prenode_array, vector<vector<Node *>> &node_array){
	graph.toGraph(src_img, dot.divideContours, prenode_array);
	graph.setCorner(src_img, prenode_array, node_array);
	//graph.deformeNode(node_array);
}

void doImwrite(vector<vector<Node *>> node_array, int rows, int cols){
	cv::Mat image = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
	for (int i = 0; i < node_array.size(); i++){
		for (int j = 0; j < node_array[i].size(); j++){
			Node *node = node_array[i].at(j);
			int y = (*node).getNodeY();
			int x = (*node).getNodeX();
			circle(image, cv::Point(x, y), 5, cv::Scalar(0, 255, 0), -1, 8);
		}
	}
	cv::imwrite("image/image" + to_string(test_count++) + ".png", image);
}

void doDot(cv::Mat &src_img, cv::Mat &result_img){
	vector<vector<Node *>> prenode_array;
	vector<vector<Node *>> node_array;
	dot.init();
	dot.setWhiteDots(src_img);
	dot.findStart(src_img);
	dot.makeLine(src_img);
	dot.divideCon(SPACESIZE);
	doGraph(src_img, prenode_array, node_array);
	doImwrite(node_array, src_img.rows, src_img.cols);
	doCatmull(result_img, node_array);
}

void doAfterImg(cv::Mat &result_img, cv::Mat depthcontour_img, vector<cv::Mat> &afterimg_array){
	//1回目はdotから始めて、2回目以降はeffectかけたresultがほしいので、effectから始める
	if (afterimg_array.size() == 0){
		doDot(depthcontour_img, result_img);
		//1枚目を明るさ下げてarrayに保存
		effect.addAfterImg(result_img, afterimg_array);
	}
	else {
		//arrayに入っている画像をor演算子でつなげて背景にする
		for (int i = 0; i < afterimg_array.size(); i++){
			bitwise_or(result_img, afterimg_array.at(i), result_img);
		}

		//上で得られたresult_imgを背景にして線を上書きする
		doDot(depthcontour_img, result_img);
		//この時の線をarrayに追加する
		effect.addAfterImg(result_img, afterimg_array);
	}
}

void main() {
	try {
		Depth depth;
		Log log;
		log.Initialize("logPOINTER.txt");
		cv::Mat rgb_img;
		cv::Mat result_img;
		vector<cv::Mat> afterimg_array;
		int count = 0;
		while (1) {

			depth.setBodyDepth();
			depth.setNormalizeDepth(depth.bodyDepthImage);
			depth.setContour(depth.normalizeDepthImage);
			result_img = cv::Mat(depth.contourImage.rows, depth.contourImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));
			if (EFFECT_FLAG){
				doAfterImg(result_img, depth.contourImage, afterimg_array);
			}
			else
				/* 残像なしversion */
				doDot(depth.contourImage, result_img);
			//フレームレート落とせてる？
			if (count % 2 == 0){
				cv::imshow("Result", result_img);
			}
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