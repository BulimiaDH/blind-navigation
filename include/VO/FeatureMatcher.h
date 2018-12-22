//
//  FeatureMatcher.h
//  VisualSLAM
//
//  Created by Lan An on 7/18/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#ifndef FEATUREMATCHER_H_
#define FEATUREMATCHER_H_
#include "FrameTracker.h"


using namespace blindfind;
using namespace cv;
using namespace std;  

class FeatureMatcher{
public:
	FeatureMatcher(){};
	~FeatureMatcher(){};
	void FeatureMatch(FeatureSet& featureset1, FeatureSet& featureset2);
};


#endif
