//
//  KeyFrame.cpp
//  VisualSLAM
//
//  Created by Lan An on 7/26/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#include "KeyFrame_Selector.h"

using namespace blindfind;
using namespace cv;
using namespace std;

void KeyFrame_Selector::Initialization()
{
	feature_maintenance_rate_current = 1;
	KeyFrames_num.push_back(0);
}

bool KeyFrame_Selector::KeyFrame_Dicision(float new_rate, int i)
{
	feature_maintenance_rate_current = feature_maintenance_rate_current * new_rate;
	cout<< "feature_maintenance_rate_current: " << feature_maintenance_rate_current <<endl;
	if(feature_maintenance_rate_current < 0.6){
		KeyFrames_num.push_back(i);
		feature_maintenance_rate_current = 1;
		return true; //means that this is the keyframe
	}
	else{
		return false;
	}
}
