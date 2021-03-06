//
//  Frame.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "View.h"

using namespace blindfind;
ImageIdGenerator* ImageIdGenerator::instance = 0;
PointIdGenerator* PointIdGenerator::instance = 0;
View::View(const vector<Mat> _imgs, int _time)
{
    imgs = vector<Mat>();
    for(unsigned int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i].clone());
    }
    stereo = imgs.size() == 2 ? true : false;
    ImageIdGenerator *imageIdGenerator = ImageIdGenerator::createInstance();
    id = imageIdGenerator->next();
    time = _time;
    keyView = false;
	hasImage = true;
}

long View::getId()
{
    return id;
}

void View::setRelationPrev(std::vector<double> prev)
{
        relationPrev = vector<double>();
        for(size_t k=0; k<prev.size(); k++)
        {
            this->relationPrev.push_back(prev[k]);
        }
}

vector<Mat> View::getImgs()
{
    vector<Mat> _imgs;
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        _imgs.push_back(imgs[i].clone());
    }
    return _imgs;
}
void View::setImgs(const vector<Mat> _imgs)
{
    imgs = vector<Mat>();
    for(unsigned int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i].clone());
    }

}



void View::setGroundTruthInfo(double  gtXCoor, double gtYCoor, double gtAngle)
{
    this->gtInfo.gtXCoor=gtXCoor;
    this->gtInfo.gtYCoor=gtYCoor;
    this->gtInfo.gtAngle=gtAngle;
}


void View::deleteImgs(
)
{
    for(unsigned int i = 0; i < imgs.size(); i++)
        imgs[i].release();

	hasImage = false;
}
FeatureSet& View::getLeftFeatureSet()
{
   return leftFeatureSet;
}
FeatureSet& View::getRightFeatureSet()
{
   return rightFeatureSet;
}
void View::setLeftFeatures(const FeatureSet _leftFeatureSet)
{
   leftFeatureSet = _leftFeatureSet;
}
void View::setRightFeatures(const FeatureSet _rightFeatureSet)
{
    rightFeatureSet = _rightFeatureSet;
}
Mat View::getPose()
{
    return pose.clone();
}
void View::setPose(const Mat _pose)
{
    pose = _pose.clone();
}
void View::setPose(const Mat _R, const Mat _T)
{
    pose = Mat::eye(4, 4, CV_64F);
    Mat aux = pose(Rect(0, 0, 3, 3));
    _R.copyTo(aux);
    aux = pose(Rect(3, 0, 1, 3));
    _T.copyTo(aux);
}
Mat View::getR()
{
    return pose(Rect(0, 0, 3, 3)).clone();
}
Mat View::getT()
{
    return pose(Rect(3, 0, 1, 3)).clone();
}

