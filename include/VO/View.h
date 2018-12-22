//
//  Frame.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Modified by Hongyi on 06/04/18.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef View_h
#define View_h

#include "Feature.h"
#include "Factory.h"
//#include "Utility.h"
#include "CameraParameters.h"
#include "SystemParameters.h"
namespace blindfind
{

    typedef struct{
        double gtXCoor;
        double gtYCoor;
        double gtAngle;
    }GroundTruthInfo;


    class View
    {
    public:
        View(const vector<Mat> _imgs, int _time);

		//Feature Accesser
        FeatureSet& getLeftFeatureSet();
        FeatureSet& getRightFeatureSet();
        long getId();
        vector<Mat> getImgs();
        void setImgs(const vector<Mat> _imgs);
        void deleteImgs();
    	void setLeftFeatures(const FeatureSet _leftFeatureSet);
        void setRightFeatures(const FeatureSet _rightFeatureSet);

		//Pose Accesser
        Mat getPose();
        void setPose(const Mat _pose);
        void setPose(const Mat _R, const Mat _T);
		void setRelativePose(const Mat _relativePose, int pairIndex){relativePose = _relativePose.clone(); hasRelative = 1; relativePoseIndex = pairIndex;};
		void setPreviousViewId(const long id){previousViewId = id;}
        Mat getR();
        Mat getT();
		void setRelativeRVec(const Mat _rVec){relativeRVec = _rVec.clone();};
		void setRelativeTParam(const Mat _tParam){relativeTParam = _tParam.clone();};
		Mat getRelativeRVec(){return relativeRVec;};
		Mat getRelativeTParam(){return relativeTParam;};
		int getPairs(){return relativePoseIndex;};
		void setScale(double _scale){scale = _scale;};

        void setKeyView(){keyView = true;}
        void unsetKeyView(){keyView = false;}
        bool isKeyView(){return keyView;}
        bool isStereo(){return stereo;}
        int getTime(){return time;}
        void setGt(const Mat _gt){gt = _gt.clone();}
        Mat getGt(){return gt.clone();}
		bool checkImage(){return hasImage;};
		Mat getRelativePose(){return relativePose;};
		int getRelativePoseIndex(){return relativePoseIndex;};
		bool isRelativeComputed(){return hasRelative;};
		long getPreviousViewId(){return previousViewId;}; 
        void setGroundTruthInfo(double  gtXCoor, double gtYCoor, double gtAngle);
        vector<double> gerRelationPrev(){return relationPrev;}
        void setRelationPrev(std::vector<double> prev);
        GroundTruthInfo getGtInfo(){ return this->gtInfo;};

        /*This is temporary solution*/
        Mat getPoolingFeature(){return this->poolingLayerFeatures.clone();}
        Mat getFullConnectionFeature(){return this->fullConnectionLayerFeatures.clone();}
        void setPoolingFeature(Mat poolingLayerFeature){this->poolingLayerFeatures=poolingLayerFeature.clone();}
        void setFullConnectionFeature(Mat fullConnectionFeature){this->fullConnectionLayerFeatures=fullConnectionFeature;}

    private:
        GroundTruthInfo gtInfo;
        long id;
        bool keyView;
        int time;
        vector<double> relationPrev;
        vector<Mat> imgs;
        FeatureSet leftFeatureSet;
        FeatureSet rightFeatureSet;
        Mat pose;
		Mat relativePose;	
		int relativePoseIndex;
		long previousViewId;
		bool hasRelative = 0;
		bool global = 0;
        Mat gt;
        bool stereo;
		bool hasImage;

		Mat relativeRVec;
		Mat relativeTParam;
		double scale;

        Mat poolingLayerFeatures;
        Mat fullConnectionLayerFeatures;
    };
}

#endif /* View_h */
