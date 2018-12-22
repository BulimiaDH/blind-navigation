//
//  KeyFrame.h
//  VisualSLAM
//
//  Created by Lan An on 7/26/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#ifndef KeyFrame_H_
#define KeyFrame_H_

#include "OpticalFlow.h"
#include "FeatureMatcher.h"

using namespace blindfind;
using namespace cv;
using namespace std;

class KeyFrame_Selector
{
public:
	bool KeyFrame_Dicision(float new_rate, int i);
	void Initialization();
	vector<int> Output_Keyframe_nums(){ return KeyFrames_num;}; 

private:
	vector<int> KeyFrames_num;
	float feature_maintenance_rate_current = 1;
};


#endif
