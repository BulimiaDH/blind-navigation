// Canvas.cpp
// VisualSLAM LEMS Group
//
//
// Created by Hongyi Fan on 06/04/18

#ifndef CANVAS_H_
#define CANVAS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Feature.h"
#include "View.h"

using namespace cv;
using namespace std;
class Canvas
{
public:
	Canvas(){};
	~Canvas(){};
	static void drawSingleImage(Mat img1);
	static void drawStereoPair(vector<Mat> imgs);
	static void drawFeaturePoints(Mat img1, vector<KeyPoint> points, std::string windowName);
	static void drawFeaturePoints(Mat img1, vector<blindfind::Feature> points, std::string windowName);
	static void drawFeaturePoints(blindfind::View* view, int leftORRight, std::string windowName);
};

#endif
