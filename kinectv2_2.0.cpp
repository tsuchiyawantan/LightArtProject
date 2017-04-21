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
vector<vector<Node *>> former_node_array;
int test_count = 1;

void doCatmull(cv::Mat &result_img, vector<vector<Node *>> node_array){
	catmull.init();
	catmull.drawLine(result_img, node_array, HUE);
	catmull.drawInline(result_img, HUE);
}

void doGraph(cv::Mat &src_img, vector<vector<Node *>> &prenode_array, vector<vector<Node *>> &node_array){
	graph.toGraph(src_img, dot.divide_contours, prenode_array);
	graph.setCorner(src_img, prenode_array, node_array);
	graph.deformeNode(src_img, node_array, ::former_node_array);
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

void removeNodes(vector<vector<Node *>> &arr){
	for (vector<vector<Node *>>::iterator it = arr.begin(); it != arr.end(); it++){
		for (vector<Node *>::iterator itra = it->begin(); itra != it->end(); itra++){
			//if ((*itra) == NULL) break;
			delete (*itra);
		}
	}
}

void removeFormerNodes(){
	for (vector<vector<Node *>>::iterator it = ::former_node_array.begin(); it != ::former_node_array.end(); it++){
		for (vector<Node *>::iterator itra = it->begin(); itra != it->end(); itra++){
			//if ((*itra) == NULL) break;
			delete (*itra);
		}
	}
	::former_node_array.clear();
	::former_node_array.shrink_to_fit();
}

void copyNodes(vector<vector<Node *>> node_array){
	for (int i = 0; i < node_array.size(); i++){
		vector<Node *> node_array_child;
		for (int j = 0; j < node_array[i].size(); j++){
			Node node = (*node_array[i].at(j));
			node_array_child.push_back(new Node(node));
		}

		//ノードの連結操作
		Node *this_node;
		Node *prev_node;
		Node *next_node;

		for (int l = 0; l < node_array_child.size(); l++){
			if (l == 0){ //始点
				this_node = node_array_child.at(l);
				next_node = node_array_child.at(l + 1);
				(*this_node).addEdgeNode2(next_node, 0);
			}
			else if (l == node_array_child.size() - 1){ //終点
				this_node = node_array_child.at(l);
				prev_node = node_array_child.at(l - 1);
				int edgearray_num = (*prev_node).hasEdge(this_node);
				if (edgearray_num >= 0){
					Edge *edge = (*prev_node).getEdge(edgearray_num);
					(*this_node).addEdge(edge);
				}
			}
			else {
				this_node = node_array_child.at(l);
				prev_node = node_array_child.at(l - 1);
				next_node = node_array_child.at(l + 1);
				(*this_node).addEdgeNode2(next_node, 0);
				int edgearray_num = (*prev_node).hasEdge(this_node);
				if (edgearray_num >= 0){
					Edge *edge = (*prev_node).getEdge(edgearray_num);
					(*this_node).addEdge(edge);
				}
			}
		}
		::former_node_array.push_back(node_array_child);
	}
}

void doDot(cv::Mat &src_img, cv::Mat &result_img){
	vector<vector<Node *>> prenode_array;
	vector<vector<Node *>> node_array;

	dot.init();
	dot.setWhiteDots(src_img);
	dot.findStart(src_img);
	dot.makeLine(src_img);
	dot.divideCon(SPACESIZE);
	doGraph(src_img, node_array, prenode_array);
	doCatmull(result_img, node_array);

	if (former_node_array.size()) {
		removeFormerNodes();
	}

	//メモリ解放
	if (prenode_array.size() > 0) {
		copyNodes(node_array);
		removeNodes(prenode_array);
		prenode_array.clear();
		node_array.clear();
		node_array.shrink_to_fit();
		prenode_array.shrink_to_fit();
	}
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
			/* EFFECT_FLAG=1ならば、残像ありversion */
			if (EFFECT_FLAG){
				doAfterImg(result_img, depth.contourImage, afterimg_array);
			}
			else
				/* 残像なしversion */
				doDot(depth.contourImage, result_img);

			//フレームレート落として表示
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