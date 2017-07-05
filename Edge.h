#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

//相互参照対策
class Node;

class Edge{
private:
	const Node *node1;
	const Node *node2;
public:
	//エッジのノード
	Edge(Node *mynode){
		node1 = mynode;
	}

	Edge(const Node& mynode){
		node1 = &mynode;
	}

	void setNode1(Node *node){
		node1 = node;
	}
	
	void setNode2(Node *node){
		node2 = node;
	}

	//ポインタ返し
	const Node *getNode1(){
		return node1;
	}
	
	//ポインタ返し
	const Node *getNode2(){
		return node2;
	}
};