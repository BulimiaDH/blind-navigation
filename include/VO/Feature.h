//
//  Feature.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Feature_h
#define Feature_h

#include <iostream>
#include <stdio.h>
#include <map>
#include <set>
#include <list>
#include <vector>
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
//#include "opencv2/plot.hpp"
#include "SystemParameters.h"
#include "Factory.h"
//#include "Converter.h"

using namespace std;
using namespace cv;

namespace blindfind{
    class Feature
    {
    private:
        KeyPoint point2d;
		cv::Mat descriptor;
		int descriptorDim;
        long long id;
		bool hasDescriptor = 0;
    public:
        Feature();
        Feature(KeyPoint _point2d) : point2d(_point2d)
		{	
			id = PointIdGenerator::createInstance()->next();
			hasDescriptor = 0;
			descriptorDim = 0;
		};

        Feature(long long _id) : id(_id)
		{
			point2d = KeyPoint({0.0, 0.0}, 1.f);
			hasDescriptor = 0;
			descriptorDim = 0;
		};

        Feature(KeyPoint _point2d, long long _id) : point2d(_point2d), id(_id)
		{
			hasDescriptor = 0;
			descriptorDim = 0;	
		};

		Feature(KeyPoint _point2d, cv::Mat _descriptor): point2d(_point2d)
		{
			descriptor = _descriptor.clone();
			id = PointIdGenerator::createInstance()->next();//in factory.h
			hasDescriptor = 1;
			descriptorDim = descriptor.rows;
		}

		Feature(KeyPoint _point2d, cv::Mat _descriptor, long long _id):point2d(_point2d), id(_id)
		{
			descriptor = _descriptor.clone();
			hasDescriptor = 1;	
			descriptorDim = descriptor.rows;
		}


		~Feature(){descriptor.deallocate();};

        KeyPoint getPoint(){return point2d;}
		void setPoint(KeyPoint _point2d){point2d = _point2d;};
		void setDescriptor(cv::Mat _descriptor)
		{
			descriptor = _descriptor.clone();
			hasDescriptor = 1;
			descriptorDim = descriptor.rows;
		}
		cv::Mat getDescriptor(){return descriptor;};
		bool checkDescriptor(){return hasDescriptor;};
		int getDescriptorDim(){return descriptorDim;};
        long long getId(){return id;};
        void setId(const long long new_ID){id = new_ID;};
    };
    
    class FeatureSet
    {
    private:
        vector<Feature> featurePoints;
        vector<long long> ids;
        map<long long, int> idLookup;
    public:
        FeatureSet(){;};
        FeatureSet(vector<Feature> _featurePoints, vector<long long> _ids) : featurePoints(_featurePoints), ids(_ids){};
		FeatureSet(vector<KeyPoint> _keyPoints);
        vector<Feature> getFeaturePoints() const {return featurePoints;};
        void setFeaturePoints(const vector<Feature> _featurePoints);
        vector<long long> getIds() const {return ids;};
        void setIds(const vector<long long> _ids);
        Feature getFeatureByIndex(int index) const {return featurePoints[index];};
        Feature getFeatureById(long long id) const;
        // upon feature increment, idLookup is adjusted accordingly
        void addFeature(Feature feature);
		void addFeature(KeyPoint point, const long long id);
        //void addFeature(Feature feature, const long id);
        // upon feature removal, idLookup is adjusted accordingly
        void removeFeatureById(const long long id);
		void removeFeatureByIndex(unsigned int index);
        void clear();
        int size();
        bool hasId(long long id);
    };
    
    class Landmark
    {
    public:
        Landmark(const Landmark &l){point3d = Point3d(l.point3d); id = l.id; from = l.from; descriptor = l.descriptor.clone(); inlier = l.inlier;}
        Landmark(Point3d _point3d, long _id, pair<long, long> _from) : point3d(_point3d), id(_id), from(_from){};
        Landmark(){point3d = Point3d(0, 0, 0); id = -1; from = {-1, -1};}
        Point3d point3d;
        long id;
        pair<long, long> from;
        Mat descriptor;
        bool inlier = true;
        
        Point3d getPoint(){return point3d;}
        void setPoint(const Point3d &_point3d){point3d = Point3d(_point3d);}
        Mat getDescriptor(){return descriptor.clone();}
        void setDescriptor(const Mat _descriptor){descriptor = _descriptor.clone();}
        bool isInlier(){return inlier;}
        void setInlier(bool _inlier){inlier = _inlier;}
    };
}


#endif /* Feature_h */
