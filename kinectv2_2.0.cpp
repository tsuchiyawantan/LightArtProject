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
#include "People.h"

#define HUE 300
#define SPACESIZE 10
#define EFFECT_FLAG 0
#define BOX_WIDTH 20
#define BOX_HEIGHT 20
#define ISVIDEO false

Dot dot;
CatmullSpline catmull;
Graph graph;
Effect effect;
vector<vector<Node *>> former_node_array;
vector<vector<vector<Node *>>> box_node;

int test_count = 0;

void doIm(cv::Mat &result_img, vector<vector<Node *>> node_array, int rows, int cols){

	cv::Mat image = result_img.clone();
	for (int i = 0; i < node_array.size(); i++){
		for (int j = 0; j < node_array[i].size(); j++){
			Node *node = node_array[i].at(j);
			int y = (*node).getNodeY();
			int x = (*node).getNodeX();
			if (node->isAngularNode())
				circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1, 8);
		}
	}
	//cv::imwrite("cornerimage/image" + to_string(test_count++) + ".png", image);
	cv::imshow("corner", image);
}

void doCatmull(cv::Mat &result_img, vector<vector<Node *>> node_array){
	catmull.init();
	catmull.drawLine(result_img, node_array, HUE);
	catmull.drawInlineHanddraw(result_img, node_array, HUE);
	/* 手描き風じゃない線の表示 */
	//catmull.drawInline(result_img, node_array, HUE);
	//doIm(result_img, node_array, result_img.rows, result_img.cols);
}


void doGraph(vector<vector<Node *>> &node_array){
	graph.toGraph(dot.divide_contours, node_array);
	//始点へのエッジを二番目へのエッジに変更する関数はここに入れる
	graph.setEdgeToOtherNode(node_array);
	graph.setCorner(node_array);
	graph.setEdge(node_array);
	graph.deformeNode(node_array, ::box_node, BOX_WIDTH, BOX_HEIGHT);
}

void removeNodes(vector<vector<Node *>> &node_array){
	for (int i = 0; i < node_array.size(); i++){
		int end = node_array[i].size();
		if (node_array[i].size() > 2){
			int second_x = node_array[i].at(1)->getNodeX();
			int second_y = node_array[i].at(1)->getNodeY();
			int end_x = node_array[i].at(node_array[i].size() - 1)->getNodeX();
			int end_y = node_array[i].at(node_array[i].size() - 1)->getNodeY();
			if (second_x == end_x && second_y == end_y) {
				end = node_array[i].size() - 1;
			}
		}
		for (int j = 0; j < end; j++){
			Node *node = node_array[i].at(j);
			delete (node);
		}
	}
}

void removeFormerNodes(){
	for (int i = 0; i < ::former_node_array.size(); i++){
		int end = ::former_node_array[i].size();
		if (::former_node_array[i].size() > 2){
			int second_x = ::former_node_array[i].at(1)->getNodeX();
			int second_y = ::former_node_array[i].at(1)->getNodeY();
			int end_x = ::former_node_array[i].at(::former_node_array[i].size() - 1)->getNodeX();
			int end_y = ::former_node_array[i].at(::former_node_array[i].size() - 1)->getNodeY();
			if (second_x == end_x && second_y == end_y) {
				end = ::former_node_array[i].size() - 1;
			}
		}
		for (int j = 0; j < end; j++){
			Node *node = ::former_node_array[i].at(j);
			delete (node);
		}
	}
	::former_node_array.clear();
	::former_node_array.shrink_to_fit();
}

void copyNodes(vector<vector<Node *>> node_array, vector<vector<Node *>> &former_node_array){
	for (int i = 0; i < node_array.size(); i++){
		vector<Node *> node_array_child;
		for (int j = 0; j < node_array[i].size(); j++){
			Node node = (*node_array[i].at(j));
			if (j == node_array[i].size() - 1 && node_array[i].size() > 2){
				int second_x = node_array[i].at(1)->getNodeX();
				int second_y = node_array[i].at(1)->getNodeY();
				int end_x = node_array[i].at(node_array[i].size() - 1)->getNodeX();
				int end_y = node_array[i].at(node_array[i].size() - 1)->getNodeY();
				if (second_x == end_x && second_y == end_y) {
					Node *node_b = node_array_child.at(1); //合流されるノード
					node_array_child.push_back(node_b);
					break;
				}
			}
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
				int edgearray_no = (*prev_node).hasEdge(this_node);
				if (edgearray_no >= 0){
					Edge *edge = (*prev_node).getEdge(edgearray_no);
					(*this_node).addEdge(edge);
				}
			}
		}
		former_node_array.push_back(node_array_child);
	}
}

//for finding near points
//近似点を探すための3次元vectorを生成
void mkBoxNode(cv::Mat src_img, vector<vector<vector<Node *>>> &box_node){
	int wimg = 0;
	int himg = 0;

	box_node.clear();
	box_node.shrink_to_fit();
	for (int i = 0; i < src_img.cols; i += BOX_WIDTH){
		wimg++;
	}
	for (int i = 0; i < src_img.rows; i += BOX_HEIGHT){
		himg++;
	}
	box_node.resize(himg + 5);
	for (int i = 0; i < himg + 1; i++){
		box_node[i].resize(wimg + 5);
	}
}

void copyNodesInfo(cv::Mat &src_img, vector<vector<vector<Node *>>> &box_node, vector<vector<Node *>> former_node_array){
	//former_node_arrayの点を入れていく
	for (int i = 0; i < former_node_array.size(); i++){
		for (int j = 0; j < former_node_array[i].size(); j++){
			Node *node = former_node_array[i].at(j);
			int x = node->getNodeX() / BOX_WIDTH;
			int y = node->getNodeY() / BOX_HEIGHT;
			box_node[y].at(x).push_back(node);
		}
	}
}

void doDot(cv::Mat &src_img, cv::Mat &result_img){
	vector<vector<Node *>> node_array;

	dot.init();
	dot.setWhiteDots(src_img);
	dot.findStart(src_img);
	dot.makeLine(src_img);
	dot.divideCon(SPACESIZE);
	doGraph(node_array);
	doCatmull(result_img, node_array);

	if (former_node_array.size()) {
		removeFormerNodes();
	}

	//メモリ解放
	if (node_array.size() > 0) {
		mkBoxNode(src_img, ::box_node);
		copyNodes(node_array, former_node_array);
		copyNodesInfo(src_img, ::box_node, former_node_array);
		removeNodes(node_array);
		node_array.clear();
		node_array.shrink_to_fit();
	}
}

void doAfterImg(cv::Mat &result_img, cv::Mat depthcontour_img, vector<cv::Mat> &afterimg_array, int count){
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

//string ty = type2str(alpha_img.type());
//printf("Matrix: %s %dx%d \n", ty.c_str(), alpha_img.cols, alpha_img.rows);
string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

void makeOverwriteImage(cv::Mat src_image, cv::Mat &foreground_image, cv::Mat &alpha_image){
	cv::dilate(src_image, src_image, cv::Mat(), cv::Point(-1, -1), 3);
	cv::Mat result_image = src_image.clone();
	//string ty = type2str(dummy.type());
	//printf("Matrix: %s %dx%d \n", ty.c_str(), dummy.cols, dummy.rows);
	cv::threshold(result_image, alpha_image, 200, 255, cv::THRESH_BINARY_INV);
	cv::threshold(alpha_image, foreground_image, 0, 255, cv::THRESH_BINARY_INV);
	cv::cvtColor(alpha_image, alpha_image, cv::COLOR_GRAY2BGR);
	cv::cvtColor(foreground_image, foreground_image, cv::COLOR_GRAY2BGR);
}

void alphaBlend(cv::Mat foreground_image, cv::Mat background_image, cv::Mat alpha_image, cv::Mat &result_image){
	cv::multiply(alpha_image, foreground_image, foreground_image);
	cv::multiply(cv::Scalar::all(1.0) - alpha_image, background_image, background_image);
	cv::add(foreground_image, background_image, result_image);
}

void createBackGroundVideos(vector<People> &videos, vector<cv::Mat> &videos_forback, bool backgroundisVideo){
	int count = 1;
	cv::Mat cap_image;
	//Capture recorded vids
	while (true){
		cv::VideoCapture cap("ppls/ppl_" + to_string(count) + ".avi");
		if (!cap.isOpened()) {
			cout << "Unable to open the camera\n";
			break;
		}
		People people(cap);
		videos.push_back(people);
		count++;
	}

	if (backgroundisVideo){
		cv::VideoCapture cap("fish.avi");
		if (!cap.isOpened()) {
			cout << "Unable to open the camera\n";
			return exit(-1);
		}

		while (true) {
			cap >> cap_image;
			if (cap_image.empty()) {
				std::cout << "Can't read frames from your camera\n";
				break;
			}
			cv::cvtColor(cap_image, cap_image, cv::COLOR_BGR2GRAY);
			cv::cvtColor(cap_image, cap_image, cv::COLOR_GRAY2BGR);
			videos_forback.push_back(cap_image);
		}
	}
}

int getRandomNumfromVids(vector<int> check){
	int random_num = rand() % check.size();
	if (check.at(random_num) > 0)  return getRandomNumfromVids(check);
	return random_num;
}

bool checkisAvailable(vector<int> check){
	for (auto itr = check.begin(); itr != check.end(); ++itr){
		if (*itr < 0) return true;
	}
	return false;
}

//-1: empty
//1: not empty
bool checkisEmpty(vector<int> check){
	for (auto itr = check.begin(); itr != check.end(); ++itr){
		if (*itr > 0) return false;
	}
	return true;
}

void createPeopleBackground(cv::Mat &result_image, vector<People> &videos, vector<int> &check, int count, int ppl_count, int depth_min){
	if (ppl_count > 20){
		if (checkisAvailable(check) && (count % 50 == 0 || (depth_min < 2000 && count % 15 == 0))){
			int i = getRandomNumfromVids(check);
			check.at(i) = 1;
		}
		int l = 0;
		for (auto itr = check.begin(); itr != check.end(); ++itr, l++){
			if (*itr < 0) continue;
			cv::Mat image;
			videos.at(l).getPics(image);
			if (!image.empty()){
				cv::bitwise_or(image, result_image, result_image);
			}
			else {
				*itr = -1;
			}
		}
	}

	//fade
	if (ppl_count < 0){
		check.clear();
		check.shrink_to_fit();
		check.resize(videos.size(), -1);
	}
}

void createBackground(cv::Mat &result_img, int depth_min, double &filter, Effect effect, bool ppl_flag){
	if (!ppl_flag){
		if (filter > 0) filter -= 0.1;
		if (filter < 0) filter = 0;
		effect.applyFilteringMulti(result_img, result_img, filter);
	}
	else{
		if (depth_min < 1000) filter = filter;
		else { 
			double dummy_filter = 1.0 / (depth_min / 1000.0); 
			if (dummy_filter - filter > 0.1) filter += 0.1;
			else filter = dummy_filter;
		}
		effect.applyFilteringMulti(result_img, result_img, filter);
	}
}

void getBackground(cv::Mat &result_image, vector<cv::Mat> videos_forback, int &count, bool is_video){
	if (is_video){
		cv::Mat clone;
		if (count >= videos_forback.size()) count = 0;
		clone = videos_forback.at(count).clone();
		result_image = clone;
	}
	else{
		//result_image = cv::Mat(depth.depthHeight, depth.depthWidth, CV_8UC3, cv::Scalar(0, 0, 0));
		result_image = cv::imread("street.jpg", cv::IMREAD_UNCHANGED);
	}
}

void main() {
	try {
		srand(time(NULL));
		Depth depth;
		Effect effect;
		vector<People> videos;
		vector<cv::Mat> videos_forback;
		cv::Mat rgb_img, result_img, ppl_img, temp_img, dummy, alpha_img, foreground_img;
		vector<cv::Mat> afterimg_array;
		vector<int> check;
		int fps = 30;
		int count = 0;
		int video_count = 0;
		double filter = 0.0;
		int ppl_count = -1;
		bool ppl_flag;

		createBackGroundVideos(videos, videos_forback, ISVIDEO);
		check.resize(videos.size(), -1);

		while (1) {
			ppl_flag = false;
			depth.setRGB(rgb_img);
			depth.setBodyDepth(ppl_flag);
			if (count > 10){
				if (ppl_flag) ppl_count++;
				else ppl_count = -1;
				depth.setNormalizeDepth(depth.bodyDepthImage);
				depth.setContour(depth.normalizeDepthImage);
				result_img = cv::Mat(depth.depthHeight, depth.depthWidth, CV_8UC3, cv::Scalar(0, 0, 0));

				getBackground(result_img, videos_forback, video_count, ISVIDEO);
				createPeopleBackground(result_img, videos, check, count, ppl_count, depth.depthMin);
				createBackground(result_img, depth.depthMin, filter, effect, ppl_flag);
				makeOverwriteImage(depth.normalizeDepthImage, foreground_img, alpha_img);

				if (EFFECT_FLAG){			/* EFFECT_FLAG=1ならば、残像ありversion */
					doAfterImg(result_img, depth.contourImage, afterimg_array, count);
				}
				else
					/* 残像なしversion */
					doDot(depth.contourImage, result_img);

				//フレームレート落として表示
				if (count % 2 == 0){
					alphaBlend(foreground_img, result_img, alpha_img, result_img);
					cv::namedWindow("RESULT IMAGE", cv::WINDOW_NORMAL);
					cv::imshow("RESULT IMAGE", result_img);
				}
			}
			count++;
			video_count++;
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