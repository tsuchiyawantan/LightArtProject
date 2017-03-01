#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

//相互参照対策
class Node;

class Edge{
private:
	Node *node1;
	Node *node2;
public:
	//エッジのノード
	Edge(Node *mynode){
		node1 = mynode;
	}

	void setNode(Node *node){
		node2 = node;
	}

	//ポインタ返し
	Node *getNode(){
		return node2;
	}
};