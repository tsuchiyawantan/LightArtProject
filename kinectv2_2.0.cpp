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

#define HUE 40
#define SPACESIZE 10
#define SCALESIZE 1
#define AFTER_FRAME 3

Dot dot;
CatmullSpline catmull;
Effect effect;


void doNodeEdge(cv::Mat& src_img, vector<vector<cv::Point>> divcon, vector<vector<Node *>> &node_array){
	//ノードの用意
	for (int i = 0; i < divcon.size(); i++){
		vector<Node *> node_array_child;
		cv::Point node;

		//エッジがない場合＝ノードが右隣にいない
		if (divcon[i].size() == 1) {
			node = divcon[i].at(0);
			node_array_child.push_back(new Node(node, 0));
			continue;
		}

		//ノードの生成
		//エッジを一個持ってる状態
		for (int j = 0; j < divcon[i].size(); j++){
			node = divcon[i].at(j);
			node_array_child.push_back(new Node(node, 1));
		}
		//終点はノードが右隣にいない
		node = divcon[i].at(divcon[i].size() - 1);
		node_array_child.push_back(new Node(node, 0));

		//ノードの連結操作
		Node *this_node;
		Node *prev_node;
		Node *next_node;

		for (int l = 0; l < node_array_child.size(); l++){
			if (l == 0){ //始点
				this_node = node_array_child.at(l);
				next_node = node_array_child.at(l + 1);
				(*this_node).addEdge(next_node, 0);
			}
			else if (l == node_array_child.size() - 1){ //終点
				this_node = node_array_child.at(l);
				prev_node = node_array_child.at(l - 1);
				int edgearray_num = (*prev_node).hasEdge(this_node);
				if (edgearray_num >= 0){
					Edge *edge = (*prev_node).getEdge(edgearray_num);
					(*this_node).setEdge(edge);
				}
			}
			else {
				this_node = node_array_child.at(l);
				prev_node = node_array_child.at(l - 1);
				next_node = node_array_child.at(l + 1);
				(*this_node).addEdge(next_node, 0);
				int edgearray_num = (*prev_node).hasEdge(this_node);
				if (edgearray_num >= 0){
					Edge *edge = (*prev_node).getEdge(edgearray_num);
					(*this_node).setEdge(edge);
				}
			}
		}
		node_array.push_back(node_array_child);
	}
}

void doCatmull(cv::Mat &srcImg, cv::Mat &resultImg, vector<vector<pair<int, int>>> &approximationLine){
	catmull.init();
	for (int i = 0; i < approximationLine.size(); i++){
		catmull.drawLine(resultImg, approximationLine[i], HUE);
	}
	//cv::blur(resultImg, resultImg, cv::Size(9, 9));
	catmull.drawInline(resultImg, HUE);
}

//8近傍にforwardがあればtrue
boolean dotExist(cv::Mat& src_img, cv::Point mid, cv::Point forward){
	int n[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
	int count = 0;
	int y = mid.y;
	int x = mid.x;
	//中点がforwardの場合
	if (y == forward.y && x == forward.x) return true;
	for (int i = 0; i < 8; i++) {
		int dy = y + n[i][0];
		int dx = x + n[i][1];
		if (dy < 0 || dy >= src_img.rows || dx < 0 || dx >= src_img.cols) continue;
		if (dy == forward.y && dx == forward.x) return true;
	}
	return false;
}

//点列の角であろう点だけをset
void setCorner(cv::Mat& src_img, vector<vector<Node *>> &node_array, vector<vector<Node *>> &newnode_array){
	cv::Point start;
	cv::Point goal;
	cv::Point mid;
	cv::Point forward;
	vector<cv::Point2f> corner;
	vector<Node *> node_array_child;
	Node *start_node;
	Node *goal_node;
	Node *forward_node;
	int di = 0;
	int j = 0;

	for (int i = 0; i < node_array.size(); i++){
		corner.clear();
		node_array_child.clear();
		//スタートの点
		//corner.push_back(cv::Point2f(divideContours[i].at(0).x, divideContours[i].at(0).y));
		di = 1;
		//2個先の点と直線を引く
		//直線の中点の8近傍に1個先の点がいなければ、1個先の点は角の可能性あり
		for (j = 0; j < node_array[i].size() - 2; j++){
			start_node = node_array[i].at(j);
			goal_node = node_array[i].at(j + 2);
			forward_node = node_array[i].at(j + 1);
			start.y = (*start_node).getNodeY();
			start.x = (*start_node).getNodeX();
			goal.y = (*goal_node).getNodeY();
			goal.x = (*goal_node).getNodeX();
			forward.y = (*forward_node).getNodeY();
			forward.x = (*forward_node).getNodeX();
			mid.y = (start.y + goal.y) / 2;
			mid.x = (start.x + goal.x) / 2;

			if (!dotExist(src_img, mid, forward)){
				corner.push_back(cv::Point2f(forward.x, forward.y));
				di = 1;
			}
			//角じゃない点が3回続いたら、1点間引く
			//詰まった線ではなく、シュッとした線になる
			if (di % 3 == 0){
				di = 0;
				node_array_child.pop_back();
			}
			node_array_child.push_back(start_node);
			di++;
		}
		//残った2ノード
		while (j < node_array[i].size()){
			node_array_child.push_back(node_array[i].at(j));
			j++;
		}
		newnode_array.push_back(node_array_child);
	}
}

void deformeNode(vector<vector<Node *>> &node_array){
	for (int i = 0; i < node_array.size(); i++){
		for (int j = 0; j < node_array[i].size(); j++){
			Node *node = node_array[i].at(j);
			(*node).circleNode();
		}
	}
}

void doCatmull(cv::Mat &result_img, vector<vector<Node *>> node_array){
	catmull.init();
	catmull.drawLine(result_img, node_array, HUE);
	catmull.drawInline(result_img, HUE);
}

void doDot(cv::Mat &src_img, cv::Mat &result_img){
	vector<vector<Node *>> prenode_array;
	vector<vector<Node *>> node_array;
	dot.init();
	dot.setWhiteDots(src_img);
	dot.findStart(src_img);
	dot.makeLine(src_img);
	dot.divideCon(SPACESIZE);
	doNodeEdge(src_img, dot.divideContours, prenode_array);
	setCorner(src_img, prenode_array, node_array);
	deformeNode(node_array);
	doCatmull(result_img, node_array);
}

void addAfterImg(cv::Mat &src_img, vector<cv::Mat> &afterimg_array){
	cv::Mat src_multi_img = src_img.clone();

	if (afterimg_array.size() != 0){
		if(afterimg_array.size() > AFTER_FRAME) afterimg_array.erase(afterimg_array.begin());
		//afterimg_array配列に1/Xを足し算していく
		for (int i = 0; i < afterimg_array.size(); i++){
			effect.applyFilteringAdd(afterimg_array.at(i), 1.0 / AFTER_FRAME);

		}
	}
	effect.applyFilteringMulti(src_img, src_multi_img, 1.0 / AFTER_FRAME);
	afterimg_array.push_back(src_multi_img);
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
			if (count++ % 1000 == 0){
				clock_t start = clock();
				depth.setBodyDepth();
				clock_t end = clock();
				//cv::imshow("body index depth", depth.bodyDepthImage);
				depth.setNormalizeDepth(depth.bodyDepthImage);
				//cv::imshow("normalize depth image", depth.normalizeDepthImage);
				depth.setContour(depth.normalizeDepthImage);
				//cv::imwrite("image/edge" + to_string(count++) + ".png", depth.contourImage);
				cv::imshow("contour image", depth.contourImage);
				result_img = cv::Mat(depth.contourImage.rows, depth.contourImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));

				/*	残像ありversion */
				//1回目はdotから始めて、2回目以降はeffectかけたresultがほしいので、effectから始める
				if (afterimg_array.size() == 0){
					doDot(depth.contourImage, result_img);
					//1枚目を明るさ下げてarrayに保存
					addAfterImg(result_img, afterimg_array);
				}
				else {
					//arrayに入っている画像をor演算子でつなげて背景にする
					for (int i = 0; i < afterimg_array.size(); i++){
						bitwise_or(result_img, afterimg_array.at(i), result_img);
					}

					//上で得られたresult_imgを背景にして線を上書きする
					doDot(depth.contourImage, result_img);
					//この時の線をarrayに追加する
					addAfterImg(result_img, afterimg_array);
				}
				/* 残像あり終わり */

				/* 残像なしversion */
				//doDot(depth.contourImage, result_img);
				/* 残像なし終わり */

				cv::imshow("Result", result_img);
				auto key = cv::waitKey(20);
				if (key == 'q') break;
			}
		}
	}
	catch (exception& ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
}