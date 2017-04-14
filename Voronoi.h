#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include <set>
#include <unordered_map>
#include "Node.h"

using namespace std;

class Voronoi{
private:
public:
	set<pair<int, int>> usedDots;
	set<pair<int, int>> whiteDots;
	vector<pair<int, pair<int, int>>> priorityStart;
	vector<vector<pair<int, int>>> contours;
	vector<vector<cv::Point2f>> corners;
	vector<cv::Point> start_goal;
	cv::Subdiv2D subdiv;

	Voronoi(){
	}
	~Voronoi(){}


	//Contourのdotならばtrue, それ以外false
	boolean isContourDot(vector<pair<int, int>> &contour, int x, int y){
		vector<pair<int, int>>::iterator itr = find(contour.begin(), contour.end(), make_pair(y, x));
		if (itr != contour.end()) return true;
		return false;
	}

	//node_arrayにx,yが入ればtrue,いなければfalse
	boolean isMyNode(vector<Node *> &node_array, int x, int y){
		for (int i = 0; i < node_array.size(); i++){
			int nx = node_array.at(i)->getNodeX();
			int ny = node_array.at(i)->getNodeY();
			if (x == nx && y == ny) return true;
		}
		return false;
	}

	//x,yに最も近い点かつ自身の点列ではない点をスタート前に入れる
	void getNearestLength(unordered_map<string, vector<cv::Point>> &edge_map, vector<cv::Point2f> &points, vector<pair<int, int>> &contour, float x, float y, int start_or_end){
		string key = to_string(x) + ',' + to_string(y);
		float min = INFINITY;
		float min_x = 0;
		float min_y = 0;
		//min_x, min_yに該当する点がある場合=true, ない場合=false;
		boolean nodot = false;
		//edge_mapはx, yに近い点たちが入ってる
		for (int i = 0; i < edge_map[key].size(); i++){
			float end_x = edge_map[key].at(i).x;
			float end_y = edge_map[key].at(i).y;
			float length = sqrt((x - end_x)*(x - end_x) + (y - end_y)*(y - end_y));
			//minより小さくて、かつ自身の点列の点じゃなければ
			if (min > length && !isContourDot(contour, end_x, end_y)) {
				min_x = end_x;
				min_y = end_y;
				min = length;
				nodot = true;
			}
		}

		//min_x, min_yに該当する点がない場合
		if (!nodot) return;

		//スタート点ならばstart_or_end=1
		//点列のスタート点の前に挿入する
		if (start_or_end){
			vector<pair<int, int>>::iterator itr_con;
			itr_con = contour.begin();
			itr_con = contour.insert(itr_con, make_pair((int)min_y, (int)min_x));
		}
		//ゴール点はstart_or_end=0
		//点列の最後に挿入する
		else{
			contour.push_back(make_pair((int)min_y, (int)min_x));
		}
	}

	void makeEdgeListMap(cv::Mat src_img, vector<cv::Vec4f> &edge_list, unordered_map<string, vector<cv::Point>> &edge_map){
		for (auto itr = edge_list.begin(); itr != edge_list.end(); ++itr) {
			if ((*itr).val[0] < 0 || (*itr).val[1] < 0 || (*itr).val[2] < 0 || (*itr).val[3] < 0) continue;
			if ((int)(*itr).val[0] > src_img.cols || (int)(*itr).val[1] > src_img.rows || (int)(*itr).val[2] > src_img.cols || (int)(*itr).val[3] > src_img.rows) continue;
			//スタートのxyをキーとして、ゴールのxyを値とする
			string key = to_string((*itr).val[0]) + ',' + to_string((*itr).val[1]);
			edge_map[key].push_back(cv::Point2f((*itr).val[2], (*itr).val[3]));
			//ゴールのxyをキーとして、スタートのxyを値とする
			key = to_string((*itr).val[2]) + ',' + to_string((*itr).val[3]);
			edge_map[key].push_back(cv::Point2f((*itr).val[0], (*itr).val[1]));
		}
	}

	void connectNearest(cv::Mat &src_img, cv::Mat &result_img, cv::Subdiv2D &subdiv, vector<vector<cv::Point2f>> &points, vector<vector<pair<int, int>>> &contours){
		vector<cv::Vec4f> edge_list;
		unordered_map<string, vector<cv::Point>> edge_map;
		subdiv.getEdgeList(edge_list);
		makeEdgeListMap(src_img, edge_list, edge_map);

		for (int i = 0; i < points.size(); i++){
			//点列の始点終点
			float start_y = points[i].at(0).y;
			float start_x = points[i].at(0).x;
			float end_y = points[i].back().y;
			float end_x = points[i].back().x;
			//始点と一番近い点をcontours(devcon)に格納
			getNearestLength(edge_map, points[i], contours[i], start_x, start_y, 1);
			//終点と一番近い点をcontours(devcon)に格納
			getNearestLength(edge_map, points[i], contours[i], end_x, end_y, 0);
		}

		//描画処理
		for (int i = 0; i < contours.size(); i++){
			for (int j = 0; j < contours[i].size() - 1; j++){
				int y1 = contours[i].at(j).first;
				int x1 = contours[i].at(j).second;
				int y2 = contours[i].at(j + 1).first;
				int x2 = contours[i].at(j + 1).second;
				line(result_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), .5);
			}
		}
	}

	void deforme(cv::Mat src_img, vector<Node *> &node_array, cv::Subdiv2D subdiv){
		vector<cv::Vec4f> edge_list;
		unordered_map<string, vector<cv::Point>> edge_map;
		subdiv.getEdgeList(edge_list);
		makeEdgeListMap(src_img, edge_list, edge_map);


		for (int i = 0; i < node_array.size(); i++){
			int x = node_array.at(i)->getNodeX();
			int y = node_array.at(i)->getNodeY();
			string key = to_string(x) + ',' + to_string(y);
			int min = INFINITY;
			int min_x = 0;
			int min_y = 0;
			//min_x, min_yに該当する点がある場合=true, ない場合=false;
			boolean isdot = false;
			//edge_mapはx, yに近い点たちが入ってる
			for (int i = 0; i < edge_map[key].size(); i++){
				float end_x = edge_map[key].at(i).x;
				float end_y = edge_map[key].at(i).y;
				float length = sqrt((x - end_x)*(x - end_x) + (y - end_y)*(y - end_y));
				//点間隔が10未満でminより小さくて、かつ自身の点列の点じゃなければ
				if (length < 10 && min > length && !isMyNode(node_array, end_x, end_y)) {
					min_x = end_x;
					min_y = end_y;
					min = length;
					isdot = true;
				}
			}
			if (isdot) {/* 
						前フレームの点の周りに点を配置
						前フレームの点に円を作って、その円の中に点を配置したいかも	
						*/
				node_array.at(i)->circleNode(min_x, min_y);
			}
		}
	}

	void mkVoronoiDelaunay(cv::Mat src_img, vector<vector<Node *>> former_node_array, vector<Node *> node_array){
		subdiv.initDelaunay(cv::Rect(0, 0, 600, 600));

		//former_node
		for (int i = 0; i < former_node_array.size(); i++){
			for (int j = 0; j < former_node_array[i].size(); j++){
				int x = former_node_array[i].at(j)->getNodeX();
				int y = former_node_array[i].at(j)->getNodeY();
				subdiv.insert(cv::Point2f(x, y));
			}
		}
		//node_array
		for (int i = 0; i < node_array.size(); i++){
			int x = node_array.at(i)->getNodeX();
			int y = node_array.at(i)->getNodeY();
		//	subdiv.insert(cv::Point2f(x, y));
		}

		vector<int> idx;
		vector<std::vector<cv::Point2f>> facetLists;
		vector<cv::Point2f> facetCenters;
		subdiv.getVoronoiFacetList(idx, facetLists, facetCenters);

		// 辺のリストを取得
		vector<cv::Vec4f> edgeList;
		cv::Mat imgEdges;
		subdiv.getEdgeList(edgeList);


		// 描画
		for (auto edge = edgeList.begin(); edge != edgeList.end(); edge++)
		{
			cv::Point p1(edge->val[0], edge->val[1]);
			cv::Point p2(edge->val[2], edge->val[3]);
			cv::line(imgEdges, p1, p2, cv::Scalar(48, 128, 48));
		}

		// ドロネー三角形のリストを取得
		std::vector<cv::Vec6f> triangles;
		subdiv.getTriangleList(triangles);

	}
};