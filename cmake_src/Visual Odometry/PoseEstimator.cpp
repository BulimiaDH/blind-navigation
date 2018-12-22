#include "PoseEstimator.h"
using namespace cv;
using namespace Eigen;
using ceres::CauchyLoss;

//////////////////////////////////////////////////////////////////////////////////////////////
//Utility function:transfer pixel unit to meter unit, using K inverse matrix
void PoseEstimator::fromPixelToMeter(double xP, double yP, double &xI, double &yI)
{
	Mat pixel = Mat::zeros(3,1,CV_64F);
	pixel.at<double>(0,0) = xP;
	pixel.at<double>(1,0) = yP;
	pixel.at<double>(2,0) = 1;

	Mat Kinv = CameraParameters::getInstance()->getIntrinsic().inv();
	Mat imagePlane = Kinv * pixel;
	xI = imagePlane.at<double>(0,0);
	yI = imagePlane.at<double>(1,0);
}

//Utility function:transfer meter unit to pixel unit, using K matrix
void PoseEstimator::fromMeterToPixel(double xI, double yI, double &xP, double &yP)
{
	Mat meter = Mat::zeros(3,1,CV_64F);
	meter.at<double>(0,0) = xI;
	meter.at<double>(1,0) = yI;
	meter.at<double>(2,0) = 1;

	Mat K = CameraParameters::getInstance()->getIntrinsic();
	Mat pixel = K * meter;
	xP = pixel.at<double>(0,0);
	yP = pixel.at<double>(1,0);
}
//////////////////////////////////////Rotation Convertor/////////////////////////////////////////////////////////

//Transfer 3x3 rotation to 3x1 axis-angle paramterization
Eigen::Matrix<double,3,1> PoseEstimator::convertRotationToAxisAngleEig(Mat rMat)
{
	double rotationMat[9];
	double rotation[3];
	
	rotationMat[0] = rMat.at<double>(0,0);
	rotationMat[3] = rMat.at<double>(0,1); 
	rotationMat[6] = rMat.at<double>(0,2);
	rotationMat[1] = rMat.at<double>(1,0);
	rotationMat[4] = rMat.at<double>(1,1);
	rotationMat[7] = rMat.at<double>(1,2);
	rotationMat[2] = rMat.at<double>(2,0);
	rotationMat[5] = rMat.at<double>(2,1);
	rotationMat[8] = rMat.at<double>(2,2);

	ceres::RotationMatrixToAngleAxis(rotationMat, rotation);
	
	Eigen::Matrix<double,3,1> returnVec;	

	returnVec(0,0) = rotation[0];
	returnVec(1,0) = rotation[1];
	returnVec(2,0) = rotation[2];

	return returnVec;
}

Eigen::Matrix<double,3,1> PoseEstimator::convertRotationToAxisAngleEig(Eigen::Matrix<double,3,3> rMat)
{
	double rotationMat[9];
	double rotation[3];
	
	rotationMat[0] = rMat(0,0);
	rotationMat[3] = rMat(0,1); 
	rotationMat[6] = rMat(0,2);
	rotationMat[1] = rMat(1,0);
	rotationMat[4] = rMat(1,1);
	rotationMat[7] = rMat(1,2);
	rotationMat[2] = rMat(2,0);
	rotationMat[5] = rMat(2,1);
	rotationMat[8] = rMat(2,2);

	ceres::RotationMatrixToAngleAxis(rotationMat, rotation);
	
	Eigen::Matrix<double,3,1> returnVec;	

	returnVec(0,0) = rotation[0];
	returnVec(1,0) = rotation[1];
	returnVec(2,0) = rotation[2];

	return returnVec;
}

Mat PoseEstimator::convertRotationToAxisAngleMat(Mat rMat)
{
	double rotationMat[9];
	double rotation[3];
	
	rotationMat[0] = rMat.at<double>(0,0);
	rotationMat[3] = rMat.at<double>(0,1); 
	rotationMat[6] = rMat.at<double>(0,2);
	rotationMat[1] = rMat.at<double>(1,0);
	rotationMat[4] = rMat.at<double>(1,1);
	rotationMat[7] = rMat.at<double>(1,2);
	rotationMat[2] = rMat.at<double>(2,0);
	rotationMat[5] = rMat.at<double>(2,1);
	rotationMat[8] = rMat.at<double>(2,2);

	ceres::RotationMatrixToAngleAxis(rotationMat, rotation);
	
	Mat returnVec = Mat::zeros(3,1,CV_64F);	

	returnVec.at<double>(0,0) = rotation[0];
	returnVec.at<double>(1,0) = rotation[1];
	returnVec.at<double>(2,0) = rotation[2];

	return returnVec;

}

Mat PoseEstimator::convertRotationToAxisAngleMat(Eigen::Matrix<double,3,3> rMat)
{
	double rotationMat[9];
	double rotation[3];
	
	rotationMat[0] = rMat(0,0);
	rotationMat[3] = rMat(0,1); 
	rotationMat[6] = rMat(0,2);
	rotationMat[1] = rMat(1,0);
	rotationMat[4] = rMat(1,1);
	rotationMat[7] = rMat(1,2);
	rotationMat[2] = rMat(2,0);
	rotationMat[5] = rMat(2,1);
	rotationMat[8] = rMat(2,2);

	ceres::RotationMatrixToAngleAxis(rotationMat, rotation);
	
	Mat returnVec = Mat::zeros(3,1,CV_64F);	

	returnVec.at<double>(0,0) = rotation[0];
	returnVec.at<double>(1,0) = rotation[1];
	returnVec.at<double>(2,0) = rotation[2];

	return returnVec;
}

Eigen::Matrix<double,3,3> PoseEstimator::convertAxisAngleToRotationEig(Mat rVec)
{
	double rotationMat[9];
	double rotation[3];
	
	rotation[0] = rVec.at<double>(0,0);
	rotation[1] = rVec.at<double>(1,0);
	rotation[2] = rVec.at<double>(2,0);
	
	ceres::AngleAxisToRotationMatrix(rotation, rotationMat);

	Eigen::Matrix<double,3,3> returnMat;
	
	returnMat(0,0) = rotationMat[0];
	returnMat(0,1) = rotationMat[3]; 
	returnMat(0,2) = rotationMat[6];
	returnMat(1,0) = rotationMat[1];
	returnMat(1,1) = rotationMat[4];
	returnMat(1,2) = rotationMat[7];
	returnMat(2,0) = rotationMat[2];
	returnMat(2,1) = rotationMat[5];
	returnMat(2,2) = rotationMat[8];
	
	return returnMat;
}
Eigen::Matrix<double,3,3> PoseEstimator::convertAxisAngleToRotationEig(Eigen::Matrix<double,3,1> rVec)
{
	double rotationMat[9];
	double rotation[3];
	
	rotation[0] = rVec(0,0);
	rotation[1] = rVec(1,0);
	rotation[2] = rVec(2,0);
	
	ceres::AngleAxisToRotationMatrix(rotation, rotationMat);

	Eigen::Matrix<double,3,3> returnMat;
	
	returnMat(0,0) = rotationMat[0];
	returnMat(0,1) = rotationMat[3]; 
	returnMat(0,2) = rotationMat[6];
	returnMat(1,0) = rotationMat[1];
	returnMat(1,1) = rotationMat[4];
	returnMat(1,2) = rotationMat[7];
	returnMat(2,0) = rotationMat[2];
	returnMat(2,1) = rotationMat[5];
	returnMat(2,2) = rotationMat[8];
	
	return returnMat;
}

Mat PoseEstimator::convertAxisAngleToRotationMat(Mat rVec)
{
	double rotationMat[9];
	double rotation[3];
	
	rotation[0] = rVec.at<double>(0,0);
	rotation[1] = rVec.at<double>(1,0);
	rotation[2] = rVec.at<double>(2,0);
	
	ceres::AngleAxisToRotationMatrix(rotation, rotationMat);

	Mat returnMat = Mat::zeros(3,3,CV_64F);
	
	returnMat.at<double>(0,0) = rotationMat[0];
	returnMat.at<double>(0,1) = rotationMat[3]; 
	returnMat.at<double>(0,2) = rotationMat[6];
	returnMat.at<double>(1,0) = rotationMat[1];
	returnMat.at<double>(1,1) = rotationMat[4];
	returnMat.at<double>(1,2) = rotationMat[7];
	returnMat.at<double>(2,0) = rotationMat[2];
	returnMat.at<double>(2,1) = rotationMat[5];
	returnMat.at<double>(2,2) = rotationMat[8];
	
	return returnMat;
}

Mat PoseEstimator::convertAxisAngleToRotationMat(Eigen::Matrix<double,3,1> rVec)
{
	double rotationMat[9];
	double rotation[3];
	
	rotation[0] = rVec(0,0);
	rotation[1] = rVec(1,0);
	rotation[2] = rVec(2,0);
	
	ceres::AngleAxisToRotationMatrix(rotation, rotationMat);

	Mat returnMat = Mat::zeros(3,3,CV_64F);
	
	returnMat.at<double>(0,0) = rotationMat[0];
	returnMat.at<double>(0,1) = rotationMat[3]; 
	returnMat.at<double>(0,2) = rotationMat[6];
	returnMat.at<double>(1,0) = rotationMat[1];
	returnMat.at<double>(1,1) = rotationMat[4];
	returnMat.at<double>(1,2) = rotationMat[7];
	returnMat.at<double>(2,0) = rotationMat[2];
	returnMat.at<double>(2,1) = rotationMat[5];
	returnMat.at<double>(2,2) = rotationMat[8];
	
	return returnMat;
}

///////////////////////////////////////////////Translation Convertor/////////////////////////////////////////////////////////
//Utility Functions: Translation Data Convertor, Here we need to convert between Mat and Eigen Format.
//As weel as the conversion from translation parameterization to unit translation. 
//Inversion is prohibited for now. 
Eigen::Matrix<double,3,1> PoseEstimator::convertTranslationParamToTranslationEig(Mat tParam)
{
	Eigen::Matrix<double,3,1> returnVec;
	returnVec(0,0) = cos(tParam.at<double>(0,0)) * cos(tParam.at<double>(1,0));
	returnVec(1,0) = cos(tParam.at<double>(0,0)) * sin(tParam.at<double>(1,0));
	returnVec(2,0) = sin(tParam.at<double>(0,0));

	return returnVec;
}
Eigen::Matrix<double,3,1> PoseEstimator::convertTranslationParamToTranslationEig(Eigen::Matrix<double,2,1> tParam)
{
	Eigen::Matrix<double,3,1> returnVec;
	returnVec(0,0) = cos(tParam(0,0)) * cos(tParam(1,0));
	returnVec(1,0) = cos(tParam(0,0)) * sin(tParam(1,0));
	returnVec(2,0) = sin(tParam(0,0));

	return returnVec;
}
Mat PoseEstimator::convertTranslationParamToTranslationMat(Mat tParam)
{
	Mat returnVec = Mat::zeros(3,1,CV_64F);
	returnVec.at<double>(0,0) = cos(tParam.at<double>(0,0)) * cos(tParam.at<double>(1,0));
	returnVec.at<double>(1,0) = cos(tParam.at<double>(0,0)) * sin(tParam.at<double>(1,0));
	returnVec.at<double>(2,0) = sin(tParam.at<double>(0,0));

	return returnVec;

}
Mat PoseEstimator::convertTranslationParamToTranslationMat(Eigen::Matrix<double,2,1> tParam)
{
	Mat returnVec = Mat::zeros(3,1,CV_64F);
	returnVec.at<double>(0,0) = cos(tParam(0,0)) * cos(tParam(1,0));
	returnVec.at<double>(1,0) = cos(tParam(0,0)) * sin(tParam(1,0));
	returnVec.at<double>(2,0) = sin(tParam(0,0));

	return returnVec;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Transfer angle based translation parameters to unit translation vector 
Eigen::Matrix<double,3,1> PoseEstimator::translationConversion(Mat tP)
{
	assert(tP.rows == 2);
	assert(tP.cols == 1);
	Eigen::Matrix<double,3,1> returnMat;
	returnMat(0,0) = cos(tP.at<double>(0,0)) * cos(tP.at<double>(0,1));
	returnMat(1,0) = cos(tP.at<double>(0,0)) * sin(tP.at<double>(0,1));
	returnMat(2,0) = sin(tP.at<double>(0,0));

	return returnMat;
}

//Transfer angle-axis rotation vector to 3x3 rotation matrix; Transfer angle based translation parameters to unit translation vector
void PoseEstimator::rotationTranslationConversion(Mat tP, Mat rVec, Eigen::Matrix<double,3,3> &rMat, Eigen::Matrix<double,3,1> &T)
{
	double rotation[3];
	double rotationMat[9];
	
	rotation[0] = rVec.at<double>(0,0);
	rotation[1] = rVec.at<double>(0,1);
	rotation[2] = rVec.at<double>(0,2);

	//Convert Parametrized Translation to 3x1 vector
	Eigen::Matrix<double,3,1> Trans = PoseEstimator::translationConversion(tP);

	T(0,0) = Trans(0,0); T(1,0) = Trans(1,0); T(2,0) = Trans(2,0);
	//Convert Parametrized Rotation to 3x3 matrix
	ceres::AngleAxisToRotationMatrix(rotation,rotationMat);
	rMat(0,0) = rotationMat[0];
	rMat(0,1) = rotationMat[3]; 
	rMat(0,2) = rotationMat[6];
	rMat(1,0) = rotationMat[1];
	rMat(1,1) = rotationMat[4];
	rMat(1,2) = rotationMat[7];
	rMat(2,0) = rotationMat[2];
	rMat(2,1) = rotationMat[5];
	rMat(2,2) = rotationMat[8];

}

//Transfer angle-axis rotation vector to 3x3 rotation matrix; Transfer angle based translation parameters to unit translation vector: OpenCV Mat output version
void PoseEstimator::generateMatrix(Mat rVec, Mat tParam, Mat R, Mat T)
{
	assert(rVec.rows == 3);
	assert(rVec.cols == 1);
	assert(tParam.rows == 2);
	assert(tParam.cols == 1);
	assert(R.rows == 3);
	assert(R.cols == 3);
	assert(T.rows == 3);
	assert(T.cols == 1);

	double translation[2];
	double rotation[3];

	for (unsigned int i = 0; i < 3; i++)
	{
		if (i < 2)
		{
			translation[i] = tParam.at<double>(i,0);
		}
		rotation[i] = rVec.at<double>(i,0);	
	}

	T.at<double>(0,0) = cos(translation[0]) * cos(translation[1]);
	T.at<double>(1,0) = cos(translation[0]) * sin(translation[1]);
	T.at<double>(2,0) = sin(translation[0]);
	double rotMat[9];
	ceres::AngleAxisToRotationMatrix(rotation,rotMat);
	R.at<double>(0,0) = rotMat[0];
	R.at<double>(0,1) = rotMat[1];
	R.at<double>(0,2) = rotMat[2];
	R.at<double>(1,0) = rotMat[3];
	R.at<double>(1,1) = rotMat[4];
	R.at<double>(1,2) = rotMat[5];
	R.at<double>(2,0) = rotMat[6];
	R.at<double>(2,1) = rotMat[7];
	R.at<double>(2,2) = rotMat[8];

	
}

//convert angle-axis rotation vector and angle based translation parameters to essential matrix
Eigen::Matrix<double,3,3> PoseEstimator::buildEssentialMatrix(Mat R, Mat T)
{
	assert(rVec.rows == 3);
	assert(rVec.cols == 1);
	assert(tParam.rows == 2);
	assert(tParam.cols == 1);

	Eigen::Matrix<double,3,3> rMat = Matrix<double,3,3>::Zero();
	Eigen::Matrix<double,3,1> tVec = Matrix<double,3,1>::Zero();
	
	PoseEstimator::rotationTranslationConversion(T, R, rMat, tVec);

	cout << "R Mat:" << rMat << endl;
	cout << "T:" << tVec << endl;

	Eigen::Matrix<double,3,3> tx= Matrix<double,3,3>::Zero();	
	tx(0,1) = -tVec(2,0); //-translation[2]; 
	tx(0,2) = tVec(1,0);  //translation[1]; 
	tx(1,0) = tVec(2,0);  //translation[2]; 
	tx(1,2) = -tVec(0,0); //-translation[0]; 
	tx(2,0) = -tVec(1,0); //-translation[1]; 
	tx(2,1) = tVec(0,0); //translation[0];


	//cout << tx << endl;
	Eigen::Matrix<double,3,3> E = tx * rMat;

	return E;
}



/////////////////////////////////////////////////Triangulation/////////////////////////////////////////////////////////////////////


//Wrapper of triangulation for Feature dataset
void PoseEstimator::triangulatePoint(Feature point1, Feature point2, Mat rVec1, Mat rVec2, Mat T1, Mat T2, double scale1, double scale2, cv::Point3d &point3D)
{
	PoseEstimator::triangulatePoint(point1.getPoint().pt.x, point1.getPoint().pt.y, point2.getPoint().pt.x, point2.getPoint().pt.y,rVec1,rVec2,T1,T2,scale1,scale2,point3D);
}


void PoseEstimator::triangulatePoint(double x1, double y1, double x2, double y2, Mat rVec1, Mat rVec2, Mat T1, Mat T2, double scale1, double scale2, cv::Point3d &point3D)
{

	double x1H = 0;
	double x2H = 0;
	double y1H = 0;
	double y2H = 0;

	PoseEstimator::fromPixelToMeter(x1, y1, x1H, y1H);
	PoseEstimator::fromPixelToMeter(x2, y2, x2H, y2H);

	Eigen::Matrix<double,3,1> p1;
	Eigen::Matrix<double,3,1> p2;
	p1(0,0) = x1H;
	p1(1,0) = y1H;
	p1(2,0) = 1;
	p2(0,0) = x2H;
	p2(1,0) = y2H;
	p2(2,0) = 1;

	//Convert parameterized motion parameter to rotation matrix and translation vector
	Eigen::Matrix<double,3,3> rotMat1;
	Eigen::Matrix<double,3,3> rotMat2;
	Eigen::Matrix<double,3,1> t1;
	Eigen::Matrix<double,3,1> t2;
	
	PoseEstimator::rotationTranslationConversion(T1, rVec1, rotMat1, t1);
	PoseEstimator::rotationTranslationConversion(T2, rVec2, rotMat2, t2);
	t1 = t1 * scale1;
	t2 = t2 * scale2;

	//Least Square Triangulation(Matrix Manipulation)
	Eigen::Matrix<double,3,1> A1 = rotMat1.inverse() * p1;
	Eigen::Matrix<double,3,1> A2 = -rotMat2.inverse() * p2;
	Eigen::Matrix<double,3,1> b = rotMat1.inverse() * t1 - rotMat2.inverse() * t2;
	Eigen::Matrix<double,3,2> A;
	A.block(0,0,3,1) = A1;
	A.block(0,1,3,1) = A2;
	Eigen::Matrix<double,2,1> rhos = (A.transpose() * A).inverse() * A.transpose() * b;
	double rhoFinal = rhos(0,0);

	Eigen::Matrix<double,3,1> point3DMat = rotMat1.inverse() * ((rhoFinal * p1) - t1);
	point3D.x = point3DMat(0,0);
	point3D.y = point3DMat(1,0);
	point3D.z = point3DMat(2,0);

}
////////////////////////////////////Estimate Epipoalr Geometry//////////////////////////////////////////////////////////

//Epipolar line Error function struct for ceres solver
//The Objective function is p^T_2 * E * p1 = 0;
//The L2 problem is min |p^T_2 * E * p1|^2;
//Below is the Objective function for Ceres Solver;
struct EpipolarError{
	EpipolarError(double _x1, double _y1, double _x2, double _y2):x1(_x1),x2(_x2),y1(_y1),y2(_y2){};
	template <typename T> bool operator()(const T* const rotation, 
										  const T* const transParam, 
										  T* residuals) const{


		T rotationMatrix[9];
		Eigen::Matrix<T,3,3> tx= Matrix<T,3,3>::Zero();	
			
		//Translate Parameterization to Translation
		T transParamRadian[2];	
		transParamRadian[0] = (transParam[0]);
		transParamRadian[1] = (transParam[1]);
		T translation[3];
		translation[0] = cos(transParamRadian[0]) * cos(transParamRadian[1]);
		translation[1] = cos(transParamRadian[0]) * sin(transParamRadian[1]);
		translation[2] = sin(transParamRadian[0]);
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);
		tx(0,1) = -translation[2]; 
		tx(0,2) = translation[1]; 
		tx(1,0) = translation[2]; 
		tx(1,2) = -translation[0]; 
		tx(2,0) = -translation[1]; 
		tx(2,1) = translation[0];

		Eigen::Map<const Eigen::Matrix<T,3,3> > rotMat(rotationMatrix);
		
		Eigen::Matrix<T,3,3> E = tx * rotMat;

		Eigen::Matrix<T,3,1> p1 = Matrix<T,3,1>::Ones();
		Eigen::Matrix<T,3,1> p2 = Matrix<T,3,1>::Ones();
		p1(0,0) = (T)x1; 
		p1(1,0) = (T)y1;
		p2(0,0) = (T)x2;
		p2(1,0) = (T)y2;

		Eigen::Matrix<T,1,1> value = (p2.transpose() * E * p1);
		residuals[0] = value(0,0);
		return true;
	}
	
	static ceres::CostFunction* Create(double __x1, double __y1, double __x2, double __y2) {
		return (new ceres::AutoDiffCostFunction<EpipolarError,1,3,2>(
					new EpipolarError(__x1,__y1,__x2,__y2)));
	}
	double x1;
	double x2;
	double y1;
	double y2;
	double pi = 3.1415926 / 180;
};

bool PoseEstimator::pickSolution(double* x1, double* y1, double* x2, double* y2, int numPoints, double* rVec, double* translation)
{


	srand(time(NULL));
	int positiveCount = 0;
	//Randomly Pick Several Points
	int sampleNum = 10;
	int index = 0;
	for (int i = 0; i < sampleNum; i++)
	{
		index = rand() % numPoints;
		Mat rVec1 = cv::Mat::zeros(3,1,CV_64F);
		Mat T1 = cv::Mat::zeros(2,1,CV_64F);
		Mat rVec2 = cv::Mat::zeros(3,1,CV_64F); 
		Mat T2 = cv::Mat::zeros(2,1,CV_64F);
	
		rVec2.at<double>(0,0) = rVec[0];
		rVec2.at<double>(1,0) = rVec[1];
		rVec2.at<double>(2,0) = rVec[2];
		T2.at<double>(0,0) = translation[0];
		T2.at<double>(0,0) = translation[1];
		cv::Point3d p3D;	
		triangulatePoint(x2[index], y2[index], x1[index],y1[index], rVec1,T1,rVec2,T2,1,1, p3D);	
		if (p3D.z > 0) positiveCount++;

		cout << p3D.z << endl;
	}
	if (positiveCount > 0.7f * sampleNum) 
		return 1;
	else
		return 0;
	
}

void PoseEstimator::solveEpipolarGeometry(double* x1, double* y1, double* x2, double* y2, int numPoints, double* rVec, double* translation)
{
	//Fed points to optimization
	ceres::Problem problem; 	

	for (unsigned int i = 0; i < numPoints; i++)
	{
		ceres::CostFunction* cost_function = 
			EpipolarError::Create(x1[i],y1[i],x2[i],y2[i]);

		problem.AddResidualBlock(cost_function, 
								 new CauchyLoss(0.1),
								 rVec, translation);
	}

	//Add Bounds to parameters
	for (unsigned int i = 0; i < 3; i++)
	{
		problem.SetParameterLowerBound(rVec,i,0.0);
		problem.SetParameterUpperBound(rVec,i,2 * 3.1415926);
	}
	for (unsigned int i = 0; i < 2; i++)
	{
		problem.SetParameterLowerBound(translation,i,0.0);
		problem.SetParameterUpperBound(translation,i,2 * 3.1415926);
	}

	//Solve!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);


	//Pick the sign of solution;
	//Any Essential Matrix Estimation has 4 possible solutions: The relationship between them are: -1xT and 180 degrees rotation around T axis. 
	//Only one solution can make the triangulate depth positive. Here we need to check all these solutions to pick the correct solution. 
	//Here we implement a solution Picking function to do this.
	//PoseEstimator::pickSolution(x1,y1,x2,y2,numPoints,rVec,translation);

}


//Solve R and T up to scale using epipolar geometry 
//R and T are variables. And the objective function is the distance from points to the epipolar line
void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometry(blindfind::View* view1, blindfind::View* view2, int pairs)
{
	//If Pairs == 0: previousFrame left to right   (May not be used, just in case)
	//If Pairs == 1: previousFrame left to current left
	//If Pairs == 2: previousFrame left to current right
	//If Pairs == 3: previousFrame right to current left
	//If Pairs == 4: previousFrame right to current right
	//If Pairs == 5: currentFrame left to right    (May not be used, just in case)
	FeatureSet points1;
	FeatureSet points2;
	//Get FeatureSets
	if (pairs == 0)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view1->getRightFeatureSet();
	}
	else if (pairs == 1)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view2->getLeftFeatureSet();
	}
	else if (pairs == 2)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else if (pairs == 3)
	{
		points1 = view1->getRightFeatureSet();
		points2 = view2->getLeftFeatureSet();
	}
	else if (pairs == 4)
	{
		points1 = view1->getRightFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else if (pairs == 5)
	{
		points1 = view2->getLeftFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else
	{
		assert(pairs != 0 && pairs != 2 && pairs != 3 && pairs != 4 && pairs != 5);
	}

	cv::Mat R(3,3,CV_64F);
	cv::Mat T(3,1,CV_64F);

	//Estimate Relative Pose
	PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometry(points1, points2, R,T);

	Mat rMat = cv::Mat::eye(3,3,CV_64F);
	Mat tMat = cv::Mat::zeros(3,1,CV_64F); 
	PoseEstimator::generateMatrix(R,T,rMat,tMat);	
	//Put R and T into relativePose	
	cv::Mat relativePose = cv::Mat::eye(4,4,CV_64F);
	cv::Mat tmp = relativePose(Rect(0,0,3,3));
	tMat.copyTo(tmp);
	tmp = relativePose(Rect(3,0,1,3));
	tMat.copyTo(tmp);
	//Put relative pose into view 	
	if (pairs == 0 && pairs == 5)
	{
		cerr << "Not Supported" << endl;		
	}
	else if (pairs == 1 && pairs == 2 && pairs == 3 && pairs == 4)
	{
		view2->setRelativePose(relativePose, pairs);
		view2->setRelativeRVec(R);
		view2->setRelativeTParam(T);
		view2->setPreviousViewId(view1->getId());
	}
	else
	{
		assert(pairs != 0 && pairs != 2 && pairs != 3 && pairs != 4 && pairs != 5);
	}
}

//Solve R and T up to scale using epipolar geometry 
//R and T are variables. And the objective function is the distance from points to the epipolar line
void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometry(FeatureSet points1, FeatureSet points2, cv::Mat R, cv::Mat T)
{
	//Get corresponding points
	vector<KeyPoint> corresPoints1;
	vector<KeyPoint> corresPoints2;
	vector<long long> idsIn1 = points1.getIds();
	//std::cout << points1.size() << std::endl;
	for (int i = 0; i < points1.size() ;i++)
	{
		long long id = idsIn1[i];
		if (points1.hasId(id) && points2.hasId(id))
		{
			corresPoints1.push_back(points1.getFeatureById(id).getPoint());
			corresPoints2.push_back(points2.getFeatureById(id).getPoint());
		}
	}

	//transfer points to camera coordinate
	double* x1 = new double[corresPoints1.size()];
	double* x2 = new double[corresPoints2.size()];
	double* y1 = new double[corresPoints1.size()];
	double* y2 = new double[corresPoints2.size()];

	for (unsigned int i = 0; i < corresPoints1.size(); i++)
	{
		PoseEstimator::fromPixelToMeter(corresPoints1[i].pt.x, corresPoints1[i].pt.y, x1[i], y1[i]);
		PoseEstimator::fromPixelToMeter(corresPoints2[i].pt.x, corresPoints2[i].pt.y, x2[i], y2[i]);

	}

	double rVec[3];
	double translation[2];

	//Unit: Radius
	//Initialize Parameterset
	rVec[0] = 1;
	rVec[1] = 0;
	rVec[2] = 0;	
	translation[0] = 0;
	translation[1] = 0;

	//Fed Points to Optimization
	solveEpipolarGeometry(x1, y1, x2, y2, corresPoints1.size(), rVec, translation);
	
	T.at<double>(0,0) = translation[0];
	T.at<double>(1,0) = translation[1];

	R.at<double>(0,0) = rVec[0];
	R.at<double>(1,0) = rVec[1];
	R.at<double>(2,0) = rVec[2];
//	cout << T << endl;
//	cout << R << endl;
	//Free Spaces
	delete[] x1;
	delete[] x2;
	delete[] y1;
	delete[] y2;
}
//static void solveMotionUptoScaleUsingEpipolarGeometryRANSAC(View* view1, View* view2, int pair, int nIters, int nPoints);
void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometryRANSAC(blindfind::View* view1, blindfind::View* view2, int pairs, int nIters, int nPoints)
{
	//If Pairs == 0: previousFrame left to right   (May not be used, just in case)
	//If Pairs == 1: previousFrame left to current left
	//If Pairs == 2: previousFrame left to current right
	//If Pairs == 3: previousFrame right to current left
	//If Pairs == 4: previousFrame right to current right
	//If Pairs == 5: currentFrame left to right    (May not be used, just in case)
	FeatureSet points1;
	FeatureSet points2;
	//Get FeatureSets
	if (pairs == 0)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view1->getRightFeatureSet();
	}
	else if (pairs == 1)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view2->getLeftFeatureSet();
	}
	else if (pairs == 2)
	{
		points1 = view1->getLeftFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else if (pairs == 3)
	{
		points1 = view1->getRightFeatureSet();
		points2 = view2->getLeftFeatureSet();
	}
	else if (pairs == 4)
	{
		points1 = view1->getRightFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else if (pairs == 5)
	{
		points1 = view2->getLeftFeatureSet();
		points2 = view2->getRightFeatureSet();
	}
	else
	{
		assert(pairs != 0 && pairs != 2 && pairs != 3 && pairs != 4 && pairs != 5);
	}

	cv::Mat R(3,3,CV_64F);
	cv::Mat T(3,1,CV_64F);

	//Estimate Relative Pose
	vector<bool> inlierVec;
	PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometryRANSAC(points1, points2, R,T,nIters,nPoints,inlierVec);

	//Put R and T into relativePose	
	cv::Mat relativePose = cv::Mat::eye(4,4,CV_64F);
	cv::Mat tmp = relativePose(Rect(0,0,3,3));
	R.copyTo(tmp);
	tmp = relativePose(Rect(3,0,1,3));
	T.copyTo(tmp);
	//Put relative pose into view 	
	if (pairs == 0 && pairs == 5)
	{
		cerr << "Not Supported" << endl;		
	}
	else if (pairs == 1 && pairs == 2 && pairs == 3 && pairs == 4)
	{
		view2->setRelativePose(relativePose, pairs);
		view2->setRelativeRVec(R);
		view2->setRelativeTParam(T);
		view2->setPreviousViewId(view1->getId());
	}
	else
	{
		assert(pairs != 0 && pairs != 2 && pairs != 3 && pairs != 4 && pairs != 5);
	}

}

void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometryRANSAC(FeatureSet points1, FeatureSet points2, Mat R, Mat T, int nIters, int nPoints, vector<bool> &inlierVec)
{
	//Get corresponding points
	vector<KeyPoint> corresPoints1;
	vector<KeyPoint> corresPoints2;
	vector<long long> idsIn1 = points1.getIds();
	//std::cout << points1.size() << std::endl;
	for (int i = 0; i < points1.size() ;i++)
	{
		long long id = idsIn1[i];
		if (points1.hasId(id) && points2.hasId(id))
		{
			corresPoints1.push_back(points1.getFeatureById(id).getPoint());
			corresPoints2.push_back(points2.getFeatureById(id).getPoint());
		}
	}

	//transfer points to camera coordinate
	double* x1All = new double[corresPoints1.size()];
	double* x2All = new double[corresPoints1.size()];
	double* y1All = new double[corresPoints1.size()];
	double* y2All = new double[corresPoints1.size()];

	double* x1 = new double[nPoints];
	double* x2 = new double[nPoints];
	double* y1 = new double[nPoints];
	double* y2 = new double[nPoints];

	for (unsigned int i = 0; i < corresPoints1.size(); i++)
	{
		PoseEstimator::fromPixelToMeter(corresPoints1[i].pt.x, corresPoints1[i].pt.y, x1All[i], y1All[i]);
		PoseEstimator::fromPixelToMeter(corresPoints2[i].pt.x, corresPoints2[i].pt.y, x2All[i], y2All[i]);
	}

	double rVec[3];
	double translation[2];
	
	//RANSAC
	srand(time(NULL));
	double maxInlierRatio = 0.0;
	int maxInliers = 0;
	double bestRVec[3];
	double bestTranslation[2];
	vector<bool> inlierVector(corresPoints1.size());
	for (unsigned int i = 0; i < nIters; i++)
	{
//		cout << "Loop:" << endl;
		double rVec[3];
		double translation[2];

		//Initialize Parameterset
		rVec[0] = 1;
		rVec[1] = 0;
		rVec[2] = 0;		
		translation[0] = 0;
		translation[1] = 0;

		
		//Generate k random number between 0 and corresPoints1.size()
		std::set<int> indicesSet;
		std::vector<int> randomIndices;
		int index = rand() % corresPoints1.size();
		randomIndices.push_back(index);
		indicesSet.insert(index);
		for (int k = 1; k < nPoints; k++)
		{
			index = rand() % corresPoints1.size();
			while(indicesSet.count(index))
			{
				index = rand() % corresPoints1.size();
			}
			indicesSet.insert(index);
			randomIndices.push_back(index);	
		}
		for (int k = 0; k < nPoints; k++)
		{

			x1[k] = x1All[randomIndices[k]];
			x2[k] = x2All[randomIndices[k]];
			y1[k] = y1All[randomIndices[k]];
			y2[k] = y2All[randomIndices[k]];
		}


		solveEpipolarGeometry(x1,y1,x2,y2,nPoints,rVec,translation);	
		
		//Model Selection
		double inlierRatio = 0.0;
		int inlierNum = 0;
		vector<bool> inlierChecker(corresPoints1.size());
		for (int k = 0; k < corresPoints1.size();k++)
		{
			inlierChecker[k] = 0;
			double resValue[1];
			EpipolarError inlierSelection(x1All[k], y1All[k],x2All[k], y2All[k]);
			inlierSelection(rVec, translation, resValue);
			//cout <<"res:" <<fabs(resValue[0]) << endl;
			if (fabs(resValue[0]) < EPIPOLAR_THRESHOLD)
			{
				inlierRatio++;
				inlierNum++;
				inlierChecker[k] = 1;
			}
		}
		inlierRatio /= corresPoints1.size();
//		cout <<"inlier Ratio:" <<inlierRatio << endl;
		if (inlierRatio > maxInlierRatio)
		{
			maxInlierRatio = inlierRatio;
			maxInliers = inlierNum;
			inlierVector = inlierChecker;	
			bestRVec[0] = rVec[0]; bestRVec[1] = rVec[1]; bestRVec[2] = rVec[2];
			bestTranslation[0] = translation[0]; bestTranslation[1] = translation[1];	
		}
//		cout << "Here" << endl;
//		cout << maxInlierRatio << endl;
		if (maxInlierRatio > RANSAC_CONFIDENCE)
			break;
	}
	cout <<"Max:" << maxInliers << endl;
	double x1Inlier[maxInliers];
	double x2Inlier[maxInliers];
	double y1Inlier[maxInliers];
	double y2Inlier[maxInliers];

//	cout << corresPoints1.size() << endl;
	int inlierCounter = 0;
	for (unsigned int i = 0; i < corresPoints1.size();i++)
	{
//		cout << i << endl;
		if (inlierVector[i] == 1)
		{
			PoseEstimator::fromPixelToMeter(corresPoints1[i].pt.x, corresPoints1[i].pt.y, x1Inlier[inlierCounter], y1Inlier[inlierCounter]);
			PoseEstimator::fromPixelToMeter(corresPoints2[i].pt.x, corresPoints2[i].pt.y, x2Inlier[inlierCounter], y2Inlier[inlierCounter]);
			inlierCounter++;
		}
	}
	//solveEpipolarGeometry(x1Inlier, y1Inlier, x2Inlier, y2Inlier, maxInliers, bestRVec, bestTranslation);

	T.at<double>(0,0) = bestTranslation[0];
	T.at<double>(1,0) = bestTranslation[1];

	R.at<double>(0,0) = bestRVec[0];
	R.at<double>(1,0) = bestRVec[1];
	R.at<double>(2,0) = bestRVec[2];

	inlierVec = inlierVector;
//	cout << T << endl;
//	cout << R << endl;
//
	delete[] x1;
	delete[] x2;
	delete[] y1;
	delete[] y2;
	delete[] x1All;
	delete[] x2All;
	delete[] y1All;
	delete[] y2All;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometryRANSACFivePoints(View* view1, View* view2, int pair)
{
	//TODO
}


void PoseEstimator::solveMotionUptoScaleUsingEpipolarGeometryRANSACFivePoints(FeatureSet points1, FeatureSet points2, Mat R, Mat T)
{
	//Get corresponding points
	vector<Point2d> corresPoints1;
	vector<Point2d> corresPoints2;
	vector<long long> idsIn1 = points1.getIds();
	//std::cout << points1.size() << std::endl;
	for (int i = 0; i < points1.size() ;i++)
	{
		long long id = idsIn1[i];
		if (points1.hasId(id) && points2.hasId(id))
		{
			corresPoints1.push_back(Point2d(points1.getFeatureById(id).getPoint().pt.x, points1.getFeatureById(id).getPoint().pt.y));
			corresPoints2.push_back(Point2d(points2.getFeatureById(id).getPoint().pt.x, points2.getFeatureById(id).getPoint().pt.y));
		}
	}
	double focal = (CameraParameters::getInstance()->getFocalX() + CameraParameters::getInstance()->getFocalY()) / 2;
	Point2d pPoint(CameraParameters::getInstance()->getPrincipalPointX(), CameraParameters::getInstance()->getPrincipalPointY());

	Mat essentialMatrixStat;
	Mat E = findEssentialMat(corresPoints2,corresPoints1, focal, pPoint, RANSAC, 0.999, 1, essentialMatrixStat);

	//Recover Pose
	Mat rMat;
	Mat tVec;
	cv::recoverPose(E, corresPoints2, corresPoints1, rMat, tVec, focal, pPoint, essentialMatrixStat);
	
	
	cout << "rMat:" << rMat << endl;
	cout << "tVec" << tVec << endl;
	

	Mat relativePose = Mat::eye(4,4,CV_64F);
	Mat aux = relativePose(Rect(0,0,3,3));
	rMat.copyTo(aux);
	aux = relativePose(Rect(3,0,1,3));
	tVec.copyTo(aux);

	cout << relativePose << endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
