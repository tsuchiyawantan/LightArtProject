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

void removeNodes(vector<vector<Node *>> &pre_node, vector<vector<Node *>> &node){
	for (vector<vector<Node *>>::iterator it = pre_node.begin(); it != pre_node.end(); it++){
		for (vector<Node *>::iterator itra = it->begin(); itra != it->end(); itra++){
			delete (*itra);
			(*itra) = NULL;
		}
	}
	pre_node.clear();
	node.clear();

	pre_node.shrink_to_fit();
	node.shrink_to_fit();
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
	doCatmull(result_img, node_array);
	
	if (former_node_array.size()) {
		former_node_array.clear();
		former_node_array.shrink_to_fit();
	}
	copy(node_array.begin(), node_array.end(), back_inserter(former_node_array));
	
	//���������
	if (prenode_array.size() > 0) removeNodes(prenode_array, node_array);

}

void doAfterImg(cv::Mat &result_img, cv::Mat depthcontour_img, vector<cv::Mat> &afterimg_array){
	//1��ڂ�dot����n�߂āA2��ڈȍ~��effect������result���ق����̂ŁAeffect����n�߂�
	if (afterimg_array.size() == 0){
		doDot(depthcontour_img, result_img);
		//1���ڂ𖾂邳������array�ɕۑ�
		effect.addAfterImg(result_img, afterimg_array);
	}
	else {
		//array�ɓ����Ă���摜��or���Z�q�łȂ��Ĕw�i�ɂ���
		for (int i = 0; i < afterimg_array.size(); i++){
			bitwise_or(result_img, afterimg_array.at(i), result_img);
		}

		//��œ���ꂽresult_img��w�i�ɂ��Đ����㏑������
		doDot(depthcontour_img, result_img);
		//���̎��̐���array�ɒǉ�����
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
			/* EFFECT_FLAG=1�Ȃ�΁A�c������version */
			if (EFFECT_FLAG){
				doAfterImg(result_img, depth.contourImage, afterimg_array);
			}
			else
				/* �c���Ȃ�version */
				doDot(depth.contourImage, result_img);
			
			//�t���[�����[�g���Ƃ��ĕ\��
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