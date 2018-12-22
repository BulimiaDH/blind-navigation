#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <iostream>
#include <vector>
#include "View.h"
#include "Feature.h"
#include "SystemParameters.h"
#include <opencv2/features2d/features2d.hpp>

using namespace blindfind;
using namespace cv;
using namespace std;
class FeatureExtractor
{
public:
	FeatureExtractor(){};
	virtual ~FeatureExtractor(){};
	virtual void extractFeature(View* view) = 0;
	virtual void extractFeatureAndDescriptor(View* view) = 0;
	virtual void extractFeature(Mat img, vector<KeyPoint> &keypoints) = 0;
	virtual void extractFeatureAndDescriptor(Mat img, vector<KeyPoint> &keyPoints, Mat &desc) = 0;
};

#endif
