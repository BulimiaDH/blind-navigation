//
//  Feature.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Feature.h"

using namespace blindfind;

FeatureSet::FeatureSet(vector<KeyPoint> _keyPoints)
{
	vector<Feature> features;
	for (unsigned int i = 0; i < _keyPoints.size(); i++)
	{
		features.push_back(Feature(_keyPoints[i]));
	}
	setFeaturePoints(features);
}

void FeatureSet::setFeaturePoints(const vector<Feature> _featurePoints)
{
	featurePoints = _featurePoints;
	idLookup.clear();	
	ids.clear();
	//Update ids vector and look up table
	for (unsigned int i = 0; i < featurePoints.size(); i++)
	{
		ids.push_back(featurePoints[i].getId());
		idLookup[featurePoints[i].getId()] = i;
	}
}

void FeatureSet::clear()
{
    // clear feature points and ids
    featurePoints.clear();
    ids.clear();
	idLookup.clear();
}

void FeatureSet::addFeature(Feature feature)
{
	featurePoints.push_back(feature);
	ids.push_back(feature.getId());
	assert(!idLookup.count(feature.getId()));
	idLookup[feature.getId()] = featurePoints.size() - 1;
    //addFeature(feature.getPoint(), feature.getId());
}

Feature FeatureSet::getFeatureById(long long id) const
{
	//cout <<"Look Up Size:" << idLookup.size() << endl;
	assert(idLookup.count(id));
    //if(!idLookup.count(id))
    //    return NULL;
    int index = idLookup.at(id);
    return featurePoints[index];    
}
void FeatureSet::addFeature(KeyPoint point2d, const long long id)
{
    featurePoints.push_back(Feature(point2d));
    ids.push_back(featurePoints[featurePoints.size() - 1].getId());
    idLookup[id] = int(ids.size() - 1);
}
void FeatureSet::removeFeatureById(const long long id)
{
    vector<Feature> newPoints;
    vector<long long> newIds;
    for(unsigned int i = 0; i < featurePoints.size(); i++)
    {
        if(ids[i] != id)
        {
            newPoints.push_back(featurePoints[i]);
            newIds.push_back(ids[i]);
        }
    }
    setFeaturePoints(newPoints);
    setIds(newIds);
}

void FeatureSet::removeFeatureByIndex(const unsigned int index)
{
    vector<Feature> newPoints;
    vector<long long> newIds;
    for(unsigned int i = 0; i < featurePoints.size(); i++)
    {
        if(i != index)
        {
            newPoints.push_back(featurePoints[i]);
            newIds.push_back(ids[i]);
        }
    }
    setFeaturePoints(newPoints);
    setIds(newIds);
}

int FeatureSet::size()
{
	assert(featurePoints.size() == ids.size());
    return int(featurePoints.size());
}
bool FeatureSet::hasId(long long id)
{
    return idLookup.count(id);
}
void FeatureSet::setIds(const vector<long long> _ids)
{
    ids = vector<long long>(_ids);
    // update id lookup
    idLookup.clear();
    for(unsigned int i = 0; i < ids.size(); i++)
    {
        idLookup[ids[i]] = i;
    }
}
