// Canvas.cpp
// VisualSLAM LEMS Group
//
// Author: Hongyi Fan
// Time: 06/04/18

#include "Canvas.h"

void Canvas::drawSingleImage(cv::Mat img1)
{
	cv::namedWindow("Single image", WINDOW_AUTOSIZE);
	imshow("Single image", img1);
	waitKey(0);
}

void Canvas::drawStereoPair(vector<cv::Mat> imgs)
{
	cv::Mat H;
	cv::vconcat(imgs[0],imgs[1],H);
	cv::namedWindow("Stereo Pair", cv::WINDOW_AUTOSIZE);
	imshow("Stereo Pair", H);
	waitKey(0);
}

void Canvas::drawFeaturePoints(Mat img1, vector<KeyPoint> points, std::string windowName)
{
	Mat canvas;
    drawKeypoints(img1,points,canvas);
    
    namedWindow(windowName, WINDOW_AUTOSIZE);
    imshow(windowName, canvas);
    waitKey(0);
}


void Canvas::drawFeaturePoints(Mat img1, vector<blindfind::Feature> points, std::string windowName)
{
	Mat canvas;
	vector<KeyPoint> convertedPoints;
	std::cout << points.size() << std::endl;
	for (unsigned int i = 0; i < points.size() ; i++)
	{
		convertedPoints.push_back(points[i].getPoint());
	}
    drawKeypoints(img1,convertedPoints,canvas);
    
    namedWindow(windowName, WINDOW_AUTOSIZE);
    imshow(windowName, canvas);
    waitKey(0);
}

void Canvas::drawFeaturePoints(blindfind::View* view, int leftOrRight, std::string windowName)
{
	Mat canvas;
	if(!leftOrRight)
		Canvas::drawFeaturePoints(view->getImgs()[0], view->getLeftFeatureSet().getFeaturePoints(), windowName);
	else
		Canvas::drawFeaturePoints(view->getImgs()[1], view->getRightFeatureSet().getFeaturePoints(), windowName);
}
