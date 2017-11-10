#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include <list>

using namespace std;

class People{
private:
	vector<cv::Mat> pics, copy_pics;
	cv::Mat temp_image, interpolate_image;
	bool used = false;
	int fps = -1;
	int index = 0;
	int frame_count = 1;

public:
	void changeDesign(cv::Mat &src_image, int i){
		/*
		case 0 : Normal
		case 1 : Opposite
		*/
		switch (i){
		case 0:
			break;
		case 1: cv::flip(src_image, src_image, 1);
			break;
		}
	}

	People(cv::VideoCapture &cap){
		int i = rand() % 2;
		while (true) {
			cap >> temp_image;
			if (temp_image.empty()) {
				std::cout << "Can't read frames from your camera\n";
				break;
			}
			cv::cvtColor(temp_image, temp_image, cv::COLOR_BGR2GRAY);
			cv::cvtColor(temp_image, temp_image, cv::COLOR_GRAY2BGR);
			changeDesign(temp_image, i);
			pics.push_back(temp_image);
		}
		interpolate_image = cv::Mat(cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1, cv::Scalar(0, 0, 0));
		setFps(cap.get(CV_CAP_PROP_FPS));
	}
	~People(){}

	int getPicsLength(){
		if (pics.empty()) return 0;
		return pics.size();
	}

	int getIndex(){
		return index;
	}

	void setIndex(int i){
		index = i;
	}

	void addIndex(){
		if (getFps() > 30){
			index = index + 5;
		}
		else index++;
	}

	int getPics(cv::Mat &image){
		int index = getIndex();
		if (index > pics.size() - 1){
			setIndex(0);
			frame_count = 1;
			return -1;
		}
		image = pics.at(index);
		addIndex();
		frame_count++;
		return 1;
	}

	void setUsed(){
		used = true;
	}

	bool getUsed(){
		return used;
	}

	int getHeight(){
		return (*pics.begin()).rows;
	}

	int getWidth(){
		return (*pics.begin()).cols;
	}

	void setFps(int i){
		fps = i;
	}

	int getFps(){
		return fps;
	}
};
