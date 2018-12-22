// Odometry.hpp 
//
// VisualSLAM
//
// Author: Hongyi Fan
// Time: 6/3/2018

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <fstream>
#include "ViewReader.h"
#include "View.h"
#include "SystemParameters.h"
#include "Feature.h"
#include "ORBExtractor.h"
#include <string>
#include "Canvas.h" 
#include <vector>
#include <iostream> 
#include "FrameTracker.h"
#include "OpticalFlow.h"
#include "KeyFrame_Selector.h"

class Odometry
{
public: 
	Odometry(){};
	~Odometry()
	{
		for (unsigned int i = 0; i < _allViews.size(); i++)
		{
			delete _allViews[i];
		}
	};
	static Odometry& getInstance()
	{
		static Odometry;
		return Odometry;
	}
	void runDissectingScale(std::string outputFile);
	void runDenseOdometry(std::string outputFile);
	//void saveView(std::string outputFile, View* view);
	//TODO: Add More
	vector<Mat> readGroundTruth(string track);
	vector<shared_ptr<blindfind::View*> getAllViews();
private:
	vector<blindfind::View*> _allViews;
};

#endif
