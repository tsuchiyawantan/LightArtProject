#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>
#include "Node.h"
#include "Edge.h"

using namespace std;

class Graph{
private:
public:
	Graph(){}
	~Graph(){}

	void toGraph(cv::Mat& src_img, vector<vector<cv::Point>> divcon, vector<vector<Node *>> &node_array){
		//ノードの用意
		for (int i = 0; i < divcon.size(); i++){
			vector<Node *> node_array_child;
			cv::Point node;

			//エッジを一個持ってる状態
			for (int j = 0; j < divcon[i].size(); j++){
				node = divcon[i].at(j);
				node_array_child.push_back(new Node(node));
			}
			node_array.push_back(node_array_child);
		}
	}

	void setEdge(cv::Mat& src_img, vector<vector<Node *>> &node_array){
		for (int i = 0; i < node_array.size(); i++){
			for (int j = 0; j < node_array[i].size(); j++){
				Node *this_node;
				Node *prev_node;
				Node *next_node;

				if (j == 0){ //始点
					this_node = node_array[i].at(j);
					next_node = node_array[i].at(j + 1);
					(*this_node).addEdgeNode2(next_node, 0);
				}
				else if (j == node_array[i].size() - 1){ //終点
					this_node = node_array[i].at(j);
					prev_node = node_array[i].at(j - 1);
					this_node->setRightEdge(false);
					int edgearray_num = (*prev_node).hasEdge(this_node);
					if (edgearray_num >= 0){
						Edge *edge = (*prev_node).getEdge(edgearray_num);
						(*this_node).addEdge(edge);
					}
				}
				else {
					this_node = node_array[i].at(j);
					prev_node = node_array[i].at(j - 1);
					next_node = node_array[i].at(j + 1);
					(*this_node).addEdgeNode2(next_node, 0);
					int edgearray_num = (*prev_node).hasEdge(this_node);
					if (edgearray_num >= 0){
						Edge *edge = (*prev_node).getEdge(edgearray_num);
						(*this_node).addEdge(edge);
					}
				}
			}
		}
	}

	void setEdgeToOtherNode(cv::Mat& src_img, vector<vector<Node *>> &node_array){
		for (int i = 0; i < node_array.size(); i++){
			int start_x = node_array[i].at(0)->getNodeX();
			int start_y = node_array[i].at(0)->getNodeY();
			int end_x = node_array[i].at(node_array[i].size() - 1)->getNodeX();
			int end_y = node_array[i].at(node_array[i].size() - 1)->getNodeY();
			if (node_array[i].size() > 2) {
				if (start_x == end_x && start_y == end_y) {
					//node_arrayの最後をCからBに変更する
					//NodeCはdeleteする
					Node *node_a = node_array[i].at(node_array[i].size() - 2); //合流するノード
					Node *node_b = node_array[i].at(1); //合流されるノード
					Node *node_c = node_array[i].at(node_array[i].size() - 1); //終点ノード
					//node_a->addEdgeNode2(node_b, 0); //エッジのノードをCからBへ変更
					delete node_c;
					node_array[i].pop_back();
					node_array[i].push_back(node_b);
				}
			}
		}
	}

	//8近傍にforwardがあればtrue
	bool dotExist(cv::Mat& src_img, cv::Point mid, cv::Point forward){
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
	void setCorner(cv::Mat& src_img, vector<vector<Node *>> &node_array){
		cv::Point start;
		cv::Point goal;
		cv::Point mid;
		cv::Point next;
		Node *start_node;
		Node *goal_node;
		Node *next_node;

		for (int i = 0; i < node_array.size(); i++){
			if (node_array[i].size() < 3) continue;
			for (int j = 0; j < node_array[i].size()-2; j++){
				start_node = node_array[i].at(j);
				goal_node = node_array[i].at(j + 2);
				next_node = node_array[i].at(j + 1);
				start.y = (*start_node).getNodeY();
				start.x = (*start_node).getNodeX();
				goal.y = (*goal_node).getNodeY();
				goal.x = (*goal_node).getNodeX();
				next.y = (*next_node).getNodeY();
				next.x = (*next_node).getNodeX();
				mid.y = (start.y + goal.y) / 2;
				mid.x = (start.x + goal.x) / 2;
				double n = sqrt((next.x - mid.x)*(next.x - mid.x) + (next.y - mid.y)*(next.y - mid.y));

				if (n>5.0){
					next_node->setAngularNode();
				}
			}
		}
	}

	Node *findNearNode(Node *node, vector<Node *> near_node){
		int min = INFINITY;
		Node *ans_node = NULL;
		int x = node->getNodeX();
		int y = node->getNodeY();
		for (int i = 0; i < near_node.size(); i++){
			Node *n_node = near_node.at(i);
			int n_x = n_node->getNodeX();
			int n_y = n_node->getNodeY();
			float dist = sqrt((x - n_x)*(x - n_x) + (y - n_y)*(y - n_y));
			if (dist < min) {
				min = dist;
				ans_node = n_node;
			}
		}
		return ans_node;
	}

	void deformeNode(cv::Mat src_img, vector<vector<Node *>> &node_array, vector<vector<vector<Node *>>> box_node, int bw, int bh){
		if (box_node.size() != 0){
			for (int i = 0; i < node_array.size(); i++){
				for (int j = 0; j < node_array[i].size(); j += 1){
					Node *node = node_array[i].at(j);
					if (node->isAngularNode()){
						int x = node_array[i].at(j)->getNodeX();
						int y = node_array[i].at(j)->getNodeY();
						Node *near_node = findNearNode(node, box_node[y / bh].at(x / bw));

						if (near_node == NULL) { node->circleNode(x, y); }
						else { node->circleNode(near_node->getNodeX(), near_node->getNodeY()); }
					}
				}
			}
		}
	}
};