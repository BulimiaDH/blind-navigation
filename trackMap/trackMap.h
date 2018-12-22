/***************************
  trackMap Class Implementation
  Created by Han Deng
  Aug. 16. 2018
  *************************/
#ifndef __TRACKMAP__
#define __TRACKMAP__
#include<cmath>
#include<View.h>
//#include<Odometry.h>
#include<memory>
#include<isam/isam.h>
#include<isam/Anchor.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<opencv/cv.h>
#include<math.h>
#include<iostream>
#include<imageMatcher.h>
#include<staticMap.h>
#include"rotationEstimation.hpp"
using namespace isam;
using namespace std;
using namespace xfeatures2d; 
using namespace cv;

typedef struct{
    std::vector<std::pair<int, int> > route;
    std::pair<int, int> layerAndId;
    float score;
    
} info;

template<typename numberType>
numberType String2Number(std::string aString)
{
	std::stringstream stream(aString);
	numberType returnValue;
	stream>>returnValue;
	return returnValue;
}

template<typename numberType>
std::string Number2String(numberType number)
{
	std::stringstream strStream;
	strStream<<number;
	std::string returnValue = strStream.str();
	return returnValue;
}

typedef struct{
    std::vector<std::pair<int, int> > relation; // first integer is level of the tree, second integer is the related node in that tree level.
    int imageId;
    double score;
    bool hasRelation;
} node;

class track{

public:
    track(vector<std::shared_ptr<blindfind::View> >, int mode );
    track(const std::vector<std::vector<double> > oneTrackRelation, int numNodesOneTrack);
    track(){;};
    ~track();
    void addRelations(std::vector<std::vector<double> > oneTrackRelation, int numNodesOneTrack);
    vector<Pose2d_Node*> getNodes()
    {
        return this->_poseNodes;
    }
    vector<Pose2d*> getPose()
    {
        return this->_pose;
    }
    vector<Pose2d_Pose2d_Factor*> getPoseFactors()
    {
        return this->_poseFactors;
    }
    Pose2d_Factor* getInitFactor()
    {
        return this->_initFactor;
    }

    Pose2d_Pose2d_Factor* getOnePoseFactor(int id)
    {
        return this->_poseFactors[id];
    }

    Pose2d* getOnePose(int id)
    {
        return this->_pose[id];
    }

    Pose2d_Node* getOneNode(int id)
    {
        return this->_poseNodes[id];
    }

    int getNodeNumber()
    {
        return this->_nodeNum;
    }
    
    std::shared_ptr<blindfind::View> getOneView(int id)
    {
        return this->_allViewsInOneTrack[id];
    }
    

private:
    void changePoseStructures(int mode);
    Noise noise = SqrtInformation(10. * eye(3));
    std::vector<std::shared_ptr<blindfind::View> > _allViewsInOneTrack;
    Pose2d_Factor* _initFactor;
    vector<Pose2d_Node* > _poseNodes;
    vector<Pose2d* > _pose;
    vector<Pose2d_Pose2d_Factor* > _poseFactors;
    int _nodeNum;
};



class trackMap{

public:
    trackMap(std::string);
    ~trackMap();
    void addNewTrack(const std::vector<std::shared_ptr<blindfind::View>>, int mode, bool newOne);
    void addSingleTrackToMap(int Id);
    void optimize(bool whetherStoreMap);
    std::pair<int, int> getTrackIdFromImageId(int imageId);
    /*Only for test*/
    void addConstraints(const std::vector<std::vector<double> >&);
    void setThreshold(double distThreshold, double angleThreshold){this->_distThreshold=distThreshold; this->_angleThreshold=angleThreshold;};
    std::pair<double, double> getThreshold(){return std::pair<double, double>(this->_distThreshold, this->_angleThreshold);};
    
private:
    std::vector<std::vector<std::pair<std::vector<node>, int> > >  spatialConstraint(std::vector<std::pair<std::vector<node>, int> >& allCandidates, int trackId, double distanceThresh, double angleThresh);
    std::vector<info> dynamicProgramming(std::vector<std::pair<std::vector<node>, int> > allCandidates);
    std::vector<std::vector<double> > findConstraints(const int trackId);
    void addFeaturesToDB(int trackId);
    Noise noise = SqrtInformation(10. * eye(3));
    void mergeTracks(const std::vector<std::vector<double> >& result);
    int intensity(Mat img1, Mat img2, int dx, int dy,float fx);
    std::string _allImagesDir;
    rotationEstimation _estimator;

    int _imageId;
    int _trackId;

    double _distThreshold;
    double _angleThreshold;
    

    int _matchedImgNumber=0;
    double _totalDistance=0;

    staticMap* _staticMap;
    Slam* _slam;
    std::vector<track> _allTracks;
    std::shared_ptr<imageMatcher> _imageMatcher;
    std::map<std::pair<int, int>, int> _imageIdToTrackId;
    std::map<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> > _measureBetweenTracks;
    std::vector<Anchor2d_Node*> _anchorNodesOfTracks;
};
#endif
