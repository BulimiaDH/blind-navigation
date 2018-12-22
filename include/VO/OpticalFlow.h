//
//  OpticalFlow.cpp
//  VisualSLAM
//
//  Created by Lan An on 7/24/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include "FrameTracker.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "unistd.h"
#include "opencv2/flann/flann.hpp"

using namespace blindfind;
using namespace cv;
using namespace std;

class OpticalFlow
{
public:
    OpticalFlow(){};
    ~OpticalFlow(){};
    void KLT(Mat firstimage, Mat secondimage);
    void Search_Around();
    double orb_distance(int indx, int choice, int feature_num);
    vector<Point2f> getPoints1(){return points1_copy;};
    vector<Point2f> getPoints2(){return points2_copy;};
    void Points2_update(Point2f p, int i){points1[i] = p;};
    void store_firstimage(Mat image){firstimage = image;};
    void store_secondimage(Mat image){secondimage = image;};
    void orb_initialization();
    void display();
    float getRate(){return feature_maintenance_rate;};
    vector<Point2f> KeyPoint_convert_to_Point2f(vector<KeyPoint> keypoint);
    vector<KeyPoint> Point2f_convert_to_KeyPoint(vector<Point2f> point2f);

private:
	vector<Point2f> points1;
	vector<Point2f> points2;
    vector<Point2f> points1_copy;
    vector<Point2f> points2_copy;
	vector<Point2f> points2_orb;
	Mat firstimage;
	Mat secondimage;	
	Mat points1_descriptors;
	Mat points2_descriptors;
	Mat keypoints2_descriptors;
    float feature_maintenance_rate;
};

#endif
