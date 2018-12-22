//
//  rotationEstimation.cpp
//  enclose
//
//  Created by Fangzhou Xie on 7/31/18.
//  Copyright © 2018 xxx. All rights reserved.
//
#include <fstream>

#include "rotationEstimation.hpp"

float rotationEstimation::estimate(Mat img1, Mat img2, float fx) {
	//cvtColor(img1, img1, CV_RGB2GRAY);
	//cvtColor(img2, img2, CV_RGB2GRAY);
	const int w = img1.cols;
	const int h = img1.rows;
	const float maxDist = testThreshold/fx;
	vector<KeyPoint> keypts1, keypts2;
	Mat desc1, desc2;
	Ptr<SURF> orb = SURF::create();
	//orb->setMaxFeatures(5000);
	orb->detect(img1, keypts1);
	orb->compute(img1, keypts1, desc1);
	orb->detect(img2, keypts2);
	orb->compute(img2, keypts2, desc2);

	BFMatcher bf;
	vector<vector<DMatch>> matches_pair;
	bf.knnMatch(desc1, desc2, matches_pair, 2);

	Mat match0;
	vector<DMatch> matches0;
	for(int i = 0; i < matches_pair.size(); i++) {
		matches0.push_back(matches_pair[i][0]);
	}
	// ratio test
	vector<DMatch> matches;
	vector<Point2f> pts1, pts2;
	for(size_t i = 0; i < matches_pair.size(); i ++) {
		if(matches_pair[i][0].distance < matches_pair[i][1].distance*ratioTest) {
			matches.push_back(matches_pair[i][0]);
			// unit transformation
			pts1.push_back(keypts1[matches_pair[i][0].queryIdx].pt);
			pts1.back().x = (pts1.back().x - w/2.0f)/fx;
			pts1.back().y = (pts1.back().y - h/2.0f)/fx;
			pts2.push_back(keypts2[matches_pair[i][0].trainIdx].pt);
			pts2.back().x = (pts2.back().x - w/2.0f)/fx;
			pts2.back().y = (pts2.back().y - h/2.0f)/fx;
		}
	}
	float x, xx, y, yy;
	
	// constraint test
	vector<DMatch> matches1;
	vector<Point2f> tested1, tested2;
	for(int n = 0; n < pts1.size(); n++) {
		x = pts1[n].x;
		xx = pts2[n].x;
		y = pts1[n].y;
		yy = pts2[n].y;
		float min = 100000;
		for(float i = xx - maxDist; i <= xx + maxDist; i += 1/fx) {	
			float dist = sqrt(pow(i - xx, 2) + pow(y/sqrt(1 + x*x)*sqrt(1 + i*i) - yy, 2));
			if(dist < min) {
				min = dist;
			}
		}
		if(min < maxDist) {
			matches1.push_back(matches[n]);
			tested1.push_back(pts1[n]);
			tested2.push_back(pts2[n]);
		}
	}

	// RANSAC
	vector<DMatch> matches2;
	Mat mask;
	vector<Point2f> inliers1, inliers2;
	if(tested1.size() >= 5) {
		findEssentialMat(tested1, tested2, fx, Point2d(w/2, h/2), RANSAC, 0.999, 1.0, mask);
		for(int i = 0; i < mask.rows; i++) {
			if(mask.at<unsigned char>(i, 0) == 1) {
				matches2.push_back(matches1[i]);
				inliers1.push_back(tested1[i]);
				inliers2.push_back(tested2[i]);
			}
		}
	}
	else {
		for(int i = 0; i < tested1.size(); i++) {
			inliers1.push_back(tested1[i]);
			inliers2.push_back(tested2[i]);
			matches2.push_back(matches1[i]);
		}
	}
	// using correspondece after constraint test and RANSAC
	float angle = 0;
        for(unsigned i = 0; i < inliers1.size(); i ++) {
        	x = inliers1[i].x;
		xx = inliers2[i].x;
		angle += (x - xx)/(1.0 + x*xx);
	}
    angle /= inliers1.size();
    angle = atan(angle)/pi * 180.0;
	return angle;
}
