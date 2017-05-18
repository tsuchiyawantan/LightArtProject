#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>
#include "Edge.h"

using namespace std;

class Node{
private:
	cv::Point node;
	//0�Ԗڂ͉E�ׂ̃G�b�W�A1�Ԗڂ͍��ׂւ̃G�b�W
	vector<Edge *> edge_array;
	bool has_right_edge = false;
public:

	Node(cv::Point mynode, int has_edge){
	//	srand((unsigned int)time(NULL));
		node = mynode;
		if (has_edge) {
			has_right_edge = true;
			edge_array.push_back(new Edge(this));
		}
	}
	Node(){}
	~Node(){}


	//�R�s�[�R���X�g���N�^
	Node(const Node& copy_node){
		//edgearray��edge�𐶐����āAedge��������
		//eddge�̃R�s�[�R���X�g���N�^���K�v
		copy_node.getMyNode(node);
		if (copy_node.hasRightEdge()) {
			has_right_edge = true;
			edge_array.push_back(new Edge(this));
		}
	}

	//������Z�q
	Node& operator=(const Node& copy_node){
		if (this == &copy_node) return *this;   //���ȑ��
		if (edge_array.size()){
			for (int i = 0; i < edge_array.size(); i++){
				delete edge_array.at(i);
			}
			edge_array.clear();
			edge_array.shrink_to_fit();
		}
		copy_node.getMyNode(node);
		if (copy_node.hasRightEdge()) {
			has_right_edge = true;
			Edge *o_edge = new Edge(copy_node);
			edge_array.push_back(o_edge);
		}
		return *this;
	}

	//i�Ԗڂ̃G�b�W��node2�𖄂߂�
	void addEdgeNode2(Node *node2, int i){
		(*edge_array.at(i)).setNode2(node2);
	}

	//�V���ȃG�b�W��ǉ�
	void addEdge(Edge *edge){
		edge_array.push_back(edge);
	}

	//�V���ȃG�b�W��ǉ�
	void setEdge(Edge *edge, int num){
		edge_array.at(num) = edge;
	}

	Edge *getEdge(int n) const{
		return edge_array.at(n);
	}

	int getNodeY(){
		return node.y;
	}

	int getNodeX(){
		return node.x;
	}

	void setNodeY(int mynode){
		node.y = mynode;
	}
	
	void setNodeX(int mynode){
		node.x = mynode;
	}

	void setNode(cv::Point mynodes){
		setNodeY(mynodes.y);
		setNodeX(mynodes.x);
	}

	void getMyNode(cv::Point &mynode) const{
		mynode.y = node.y;
		mynode.x = node.x;
	}
	
	int getEdgeNum() const{
		return edge_array.size();
	}

	int hasEdge(Node *node2){
		Edge *edge;
		for (int i = 0; i < edge_array.size(); i++){
			edge = edge_array.at(i);
			if ((*edge).getNode2() == node2) return i;
		}
		return -1;
	}

	int getRandom(int min, int max)
	{
		return min + (int)(rand()*(max - min + 1.0) / (1.0 + RAND_MAX));
	}

	void circleNode(int x, int y, int R=3){
		node.x = x + getRandom(-R, R);
		node.y = y + getRandom(-R, R);
		if (node.x < 0 || node.y < 0) circleNode(x, y, R);
	}

	bool hasRightEdge() const{
		return has_right_edge;
	}
};
