

#include <ORBExtractor.h>

using namespace std; using namespace cv;
using namespace blindfind;
void ORBExtractor::extractFeature(View* view)
{

	assert(view->checkImage());	
	vector<KeyPoint> keypointsL;
	extractFeature(view->getImgs()[0], keypointsL);
	//Add FeatureSet Building
	FeatureSet features(keypointsL);
	view->setLeftFeatures(features); 
	if (view->isStereo())
	{
		vector<KeyPoint> keypointsR;
		extractFeature(view->getImgs()[1],keypointsR);
		FeatureSet featuresR(keypointsR);
		view->setRightFeatures(featuresR);
	}
}

void ORBExtractor::extractFeatureAndDescriptor(View* view)
{
	assert(view->checkImage());
	vector<KeyPoint> keypointsL;
	Mat descriptor;
	extractFeatureAndDescriptor(view->getImgs()[0],keypointsL,descriptor);
	FeatureSet featuresL;
	for (unsigned int i = 0; i < keypointsL.size(); i++)
	{
		Feature feature(keypointsL[i], descriptor.row(i).clone());
		featuresL.addFeature(feature);//for featureset
	}
	view->setLeftFeatures(featuresL);
	if (view->isStereo())
	{
		vector<KeyPoint> keypointsR;
		Mat descriptor;
		extractFeatureAndDescriptor(view->getImgs()[1],keypointsR,descriptor);
		FeatureSet featuresR;
		for (unsigned int i = 0; i < keypointsR.size(); i++)
		{
			Feature feature(keypointsL[i], descriptor.row(i).clone());
			featuresR.addFeature(feature);
		}
		view->setRightFeatures(featuresR);
	}
}

void ORBExtractor::extractFeature(Mat img, vector<KeyPoint> &keypoints)
{
	Ptr<ORB> detector = ORB::create();
	detector->setScaleFactor(ORB_SCALE);
	detector->setMaxFeatures(ORB_FEATURENUM);
	detector->setNLevels(ORB_NLEVELS);
	detector->detect(img, keypoints);
}

void ORBExtractor::extractFeatureAndDescriptor(Mat img, vector<KeyPoint> &keypoints, Mat &desc)
{
	Ptr<ORB> detector = ORB::create();
	detector->setScaleFactor(ORB_SCALE);
	detector->setMaxFeatures(ORB_FEATURENUM);
	detector->setNLevels(ORB_NLEVELS);
	vector<Mat> descriptor;
	detector->detect(img,keypoints);
	detector->compute(img, keypoints, desc);
}
