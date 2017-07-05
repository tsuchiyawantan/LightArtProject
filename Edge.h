#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

//���ݎQ�Ƒ΍�
class Node;

class Edge{
private:
	const Node *node1;
	const Node *node2;
public:
	//�G�b�W�̃m�[�h
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

	//�|�C���^�Ԃ�
	const Node *getNode1(){
		return node1;
	}
	
	//�|�C���^�Ԃ�
	const Node *getNode2(){
		return node2;
	}
};