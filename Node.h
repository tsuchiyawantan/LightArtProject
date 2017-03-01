#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>
#include "Edge.h"

using namespace std;

#define R 5

class Node{
private:
	cv::Point node;
	vector<Edge *> edge_array;
public:

	Node(cv::Point mynode, int has_edge){
	//	srand((unsigned int)time(NULL));
		node = mynode;
		if (has_edge) edge_array.push_back(new Edge(this));
	}
	~Node(){}

	//ƒGƒbƒW‚Ìnode2‚ð–„‚ß‚é
	void addEdge(Node *node, int i){
		(*edge_array.at(i)).setNode(node);
	}

	void setEdge(Edge *edge){
		edge_array.push_back(edge);
	}

	Edge *getEdge(int n){
		return edge_array.at(n);
	}

	int getNodeY(){
		return node.y;
	}

	int getNodeX(){
		return node.x;
	}

	void getMyNode(cv::Point &mynode){
		mynode.y = node.y;
		mynode.x = node.x;
	}
	
	int getMyEdgeNum(){
		return edge_array.size();
	}

	int hasEdge(Node *node){
		Edge *edge;
		for (int i = 0; i < edge_array.size(); i++){
			edge = edge_array.at(i);
			if ((*edge).getNode() == node) return i;
		}
		return -1;
	}

	int getRandom(int min, int max)
	{
		return min + (int)(rand()*(max - min + 1.0) / (1.0 + RAND_MAX));
	}

	void circleNode(){
		node.x = node.x + getRandom(-R, R);
		node.y = node.y + getRandom(-R, R);
	}
};
