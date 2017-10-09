#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

class Video{
private:
	const double T_SIZE = 0.05;
public:

	Video(){}
	~Video(){}

	void makeVideo(cv::Mat src_img){
		//出力する動画ファイルの設定をします。拡張子はWMV1で、毎秒15フレーム、画像縦横サイズは1024*1024。
		cv::VideoWriter writer("C:\\Users\\shibafu\\Documents\\OutVideo.wmv", cv::VideoWriter::fourcc('W', 'M', 'V', '1'), 15.0, cv::Size(src_img.cols, src_img.rows));
		//動画ファイルがちゃんと作れたかの判定。
		if (!writer.isOpened()){ return ; }

		//動画にする画像の名前を入れる文字配列。
		char image_name[100];
		//動画にする画像を入れるMat。
		cv::Mat image;

		//『0000000.jpg』から『0001000.jpg』までの画像を動画にしていく。
		for (int i = 0; i < 1001; i++){
			//画像の名前を更新。
			sprintf(image_name, "C:\\log\\%07d.jpg", i);
			//名前に対応した画像を取り込み。
			image = cv::imread(image_name);

			//画像がなければ処理を飛ばす。
			if (image.empty()) {
				cout << "no image : " << image_name << endl;
				continue;
			}

			//動画ファイルに画像を出力。
			writer << image;
		}
	}
};