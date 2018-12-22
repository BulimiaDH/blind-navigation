#ifndef ORBEXTRACTOR_H_
#define ORBEXTRACTOR_H_

#include "FeatureExtractor.h"

class ORBExtractor:public FeatureExtractor
{
public:
	ORBExtractor(){};
	~ORBExtractor(){};
	virtual void extractFeature(View* view);
	virtual void extractFeatureAndDescriptor(View* view);
	virtual void extractFeature(Mat img, vector<KeyPoint> &keypoints);
	virtual void extractFeatureAndDescriptor(Mat img, vector<KeyPoint> &keyPoints, Mat &desc);

};

#endif
