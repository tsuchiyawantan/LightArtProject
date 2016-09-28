#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

class NeonDesign{
private:
public:
	NeonDesign(){}
	~NeonDesign(){}
	void rgb(double H, double S, double V, int &b, int &g, int &r){
		int Hi;
		double f, p, q, t;

		Hi = ((int)(H / 60)) % 6;
		f = H / 60 - (int)(H / 60);
		p = V * (1 - (S / 255));
		q = V * (1 - f * (S / 255));
		t = V * (1 - (1 - f) * (S / 255));

		switch (Hi) {
		case 0: b = p; g = t; r = V; break;
		case 1: b = p; g = V; r = q; break;
		case 2: b = t; g = V; r = p; break;
		case 3: b = V; g = q; r = p; break;
		case 4: b = V; g = p; r = t; break;
		case 5: b = q; g = p; r = V; break;
		}
	}
};