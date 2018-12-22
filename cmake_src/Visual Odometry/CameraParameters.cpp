#include "CameraParameters.h"


using namespace std;
using namespace cv;


CameraParameters* CameraParameters::instance = 0;
//CameraParameters::_stereo = STEREO;
//CameraParameters::_focalX = FOCAL_X;
//CameraParameters::_focalY = FOCAL_Y;
//CameraParameters::_principalPointX = PRINCIPAL_X;
//CameraParameters::_principalPointY = PRINCIPAL_Y;
//CameraParameters::_baseline = BASELINE;
//CameraParameters::CameraParameters(std::string fileName)
//{
//	//Load Camera Parameters From File
//	_fileName = fileName;
//	ifstream cameraFile(_fileName);
//
//	if (cameraFile.is_open())
//	{
//		std::string stereo;
//		getline(cameraFile, stereo);
//		stringstream ss;
//		ss << stereo;
//		ss >> _stereo;
//
//		std::string focalX;
//		double nfocalX;
//		std::string focalY;
//		double nfocalY;
//		std::string principalX;
//		double nprincipalX;
//		std::string principalY;
//		double nprincipalY;
//		getline(cameraFile, focalX);
//		stringstream ssFX;
//		ssFX << focalX;
//		ssFX >> nfocalX;
//		getline(cameraFile, focalY);
//		stringstream ssFY;
//		ssFY << focalY;
//		ssFY >> nfocalY;
//		cout << focalY << endl;
//		cout << nfocalY << endl;
//		getline(cameraFile, principalX);
//		getline(cameraFile, principalY);
//		stringstream ssPX;
//		stringstream ssPY;
//		ssPX << principalX;
//		ssPX >> nprincipalX;
//		ssPY << principalY;
//		ssPY >> nprincipalY;
//
//		_focals.push_back(nfocalX);
//		_focals.push_back(nfocalY);
//		_principalPoint.push_back(nprincipalX);
//		_principalPoint.push_back(nprincipalY);
//
//		if (_stereo == 1)
//		{
//			std::string baseline;
//			getline(cameraFile, baseline);
//			stringstream ssB;
//			ssB << baseline;
//			ssB >> _baseline;
//		}
//
//		cameraFile.close();
//		//instance = new CameraParameters(_fileName);
//	}
//}

CameraParameters* CameraParameters::getInstance()
{
	if (instance == 0)
	{
		instance = new CameraParameters();
	}
	return instance;
}

double CameraParameters::getFocalX()
{
	return _focalX;
}

double CameraParameters::getFocalY()
{
	return _focalY;
}

double CameraParameters::getPrincipalPointX()
{
	return _principalPointX;
}

double CameraParameters::getPrincipalPointY()
{
	return _principalPointY;
}

double CameraParameters::getBaseline()
{
	if (_stereo == 1)
		return _baseline;
	else
		return 0;
}

cv::Mat CameraParameters::getIntrinsic()
{
	cv::Mat intrinsic = Mat::eye(3,3,CV_64F);
	intrinsic.at<double>(0,0) = getFocalX();
	intrinsic.at<double>(1,1) = getFocalY();
	intrinsic.at<double>(0,2) = getPrincipalPointX();
	intrinsic.at<double>(1,2) = getPrincipalPointY();
	
	return intrinsic.clone();
}

cv::Mat CameraParameters::getStereoPose()
{
	cv::Mat stereoPose = cv::Mat::eye(4,4,CV_64F);
	stereoPose.at<double>(0,3) = getBaseline();
	return stereoPose;
}

