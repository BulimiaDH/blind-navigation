//
//
// CameraParameters.hpp
// VisualSLAMKimia
//
// Created by Hongyi Fan on 6/1/18
//


#ifndef CAMREAPARAMETERS_H_
#define CAMERAPARAMETERS_H_
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "SystemParameters.h"

using namespace std;
class CameraParameters
{
public:
	//This class needs to initialized from a calibration file;
	//CameraParameters(std::string fileName);
	CameraParameters(){};
	~CameraParameters(){};

	//Singleton Pattern TODO
	static CameraParameters* getInstance();
	//static void initialize(std::string fileName);

	double getFocalX();
	double getFocalY();
	double getPrincipalPointX();
	double getPrincipalPointY();
	double getBaseline();
	cv::Mat getIntrinsic();
	cv::Mat getStereoPose();
private:
	static CameraParameters* instance;	

	static const bool _stereo = STEREO;
	static constexpr double _focalX = FOCAL_X;
	static constexpr double _focalY = FOCAL_Y;
	static constexpr double _principalPointX = PRINCIPAL_X;
	static constexpr double _principalPointY = PRINCIPAL_Y;
	static constexpr double _baseline = BASELINE;	
};


#endif /* CameraParameters_h */
