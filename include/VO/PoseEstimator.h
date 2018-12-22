//Pose Estimator Class
//All the functions need to be static
// Author: Hongyi Fan
// 06/20/2018


#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <iostream>
#include "View.h"
#include "Feature.h"
#include <opencv2/opencv.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/internal/eigen.h"
#include <cmath>
#include <set>
#include <vector>
#include <assert.h>
#include "SystemParameters.h"

using namespace cv;
using namespace std;
using namespace blindfind;
class PoseEstimator
{
public:
	PoseEstimator(){};
	~PoseEstimator(){};

	//The following function are trying to use point correspondence to compute the R and T: Three Variables for Euler Angles
	//Two Variables for Euler Angles.
	//TODO: All these functions need more work on solution selection.
	//TODO: Lan: You don't need to understand this part for now.
	static void solveMotionUptoScaleUsingEpipolarGeometry(View* view1, View* view2, int pair);
	static void solveMotionUptoScaleUsingEpipolarGeometry(FeatureSet points1, FeatureSet points2, Mat R, Mat T);
	static void solveMotionUptoScaleUsingEpipolarGeometryRANSAC(FeatureSet points1, FeatureSet points2, Mat R, Mat T, int nIters, int nPoints, vector<bool> &inlierVec);
	static void solveMotionUptoScaleUsingEpipolarGeometryRANSAC(View* view1, View* view2, int pair, int nIters, int nPoints);
	static bool pickSolution(double* x1, double* y1, double* x2, double* y2, int numPoints, double* rVec, double* translation);

	//Use OpenCV implementation
	static void solveMotionUptoScaleUsingEpipolarGeometryRANSACFivePoints(View* view1, View* view2, int pair);
	static void solveMotionUptoScaleUsingEpipolarGeometryRANSACFivePoints(FeatureSet points1, FeatureSet points2, Mat R, Mat T);

	//TODO:Scale Estimator
	static void solveScaleRatio(FeatureSet points1, FeatureSet points2, FeatureSet points3, Mat R12, Mat T12, Mat R13, Mat T13, double& scaleRatio);

	//Utility Function: Transfer points in pixel to point in meters using intrinsic camera matrix
	static void fromPixelToMeter(double xP, double yP, double &xI, double &yI);
	static void fromMeterToPixel(double xI, double yI, double &xP, double &yP);

	//Utility Functions: Rotation Data Convertor, Here we need to convert between Mat and Eigen Format. 
	//As well as the conversion from Rotation Matrix to Axis-angle Rotation Paramatrization
	static Eigen::Matrix<double,3,1> convertRotationToAxisAngleEig(Mat rMat);
	static Eigen::Matrix<double,3,1> convertRotationToAxisAngleEig(Eigen::Matrix<double,3,3> rMat); 
	static Mat 						 convertRotationToAxisAngleMat(Mat rMat);
	static Mat						 convertRotationToAxisAngleMat(Eigen::Matrix<double,3,3> rMat);

	static Eigen::Matrix<double,3,3> convertAxisAngleToRotationEig(Mat rVec);
	static Eigen::Matrix<double,3,3> convertAxisAngleToRotationEig(Eigen::Matrix<double,3,1> rVec);
	static Mat						 convertAxisAngleToRotationMat(Mat rVec);
	static Mat						 convertAxisAngleToRotationMat(Eigen::Matrix<double,3,1> rVec);

	//Utility Functions: Translation Data Convertor, Here we need to convert between Mat and Eigen Format.
	//As weel as the conversion from translation parameterization to unit translation. 
	//Inversion is prohibited for now. 
	static Eigen::Matrix<double,3,1> convertTranslationParamToTranslationEig(Mat tParam);
	static Eigen::Matrix<double,3,1> convertTranslationParamToTranslationEig(Eigen::Matrix<double,2,1> tParam);
	static Mat 						 convertTranslationParamToTranslationMat(Mat tParam);
	static Mat						 convertTranslationParamToTranslationMat(Eigen::Matrix<double,2,1> tParam);		

	//TODO:Some other converters, they are not well organized will be removed later
	static void generateMatrix(Mat rVec, Mat tParam, Mat R, Mat T);
	static Eigen::Matrix<double,3,3> buildEssentialMatrix(Mat R, Mat T);	
	static Eigen::Matrix<double,3,1> translationConversion(Mat tP);
	static void rotationTranslationConversion(Mat tP, Mat rVec, Eigen::Matrix<double,3,3> &rMat, Eigen::Matrix<double,3,1> &T);

	//Triangulation
	static void triangulatePoint(Feature point1, Feature point2, Mat rVec1, Mat rVec2, Mat T1, Mat T2, double scale1, double scale2,cv::Point3d &point3d);
	static void triangulatePoint(double x1, double y1, double x2, double y2, Mat rVec1, Mat rVec2, Mat T1, Mat T2, double scale1, double scale2, cv::Point3d &point3D);
private:
	static void solveEpipolarGeometry(double* x1, double* y1, double* x2, double* y2, int numPoints, double* R, double* T);
};
#endif
