#ifndef FRAMETRACKER_H_
#define FRAMETRACKER_H_

#include <iostream>
#include <vector>
#include "View.h"
#include "Feature.h"
#include "SystemParameters.h"
#include <ctype.h>
#include "FeatureMatcher.h"


using namespace blindfind;
using namespace cv;
using namespace std;

class FrameTracker
{
public:
	FrameTracker(){};
	virtual ~FrameTracker(){};
	virtual void trackForward(View* view1, View* view2) = 0;
	virtual void trackStereo(View* view1) = 0;

	//Optional
	virtual void refineMatching(View* view1, View* view2) = 0;

	//More function to do

	//help function

};

class Derived: public FrameTracker
{
public:
	Derived(){};
	virtual ~Derived(){};
	virtual void trackForward(View* view1, View* view2);
    virtual void trackStereo(View* view1);
    virtual void refineMatching(View* view1, View* view2);

	//help function

};

#endif

