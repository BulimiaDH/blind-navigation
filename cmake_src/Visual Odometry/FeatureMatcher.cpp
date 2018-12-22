//
//  FeatureMatcher.cpp
//  VisualSLAM
//
//  Created by Lan An on 7/18/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#include "FeatureMatcher.h"
 
using namespace blindfind;
using namespace cv;
using namespace std;


//view1 has already finished, we need to update view2 now
void FeatureMatcher::FeatureMatch(FeatureSet& featureset1, FeatureSet& featureset2){//do not care about mono or stereo

	vector<Feature> feature1 = featureset1.getFeaturePoints();
	vector<Feature> feature2 = featureset2.getFeaturePoints();

//test address
  // cout<< "feature address1 after:  " << &feature1[0] <<endl;
  // cout<< "feature address2 after:  " << &feature2[0] <<endl;

  //get the descriptor and initialize it(check type, reshape)
  Mat descriptors1;
  Mat descriptors2;
  for(int i = 0; i < feature1.size(); i++){
    Mat descriptors_temp1 = feature1[i].getDescriptor();
    descriptors1.push_back(descriptors_temp1);
    //test feature1 id
    // cout << "feature1_id"<< i <<" :" << feature1[i].getId()<<endl;
  }
   // Mat descriptors1_reshape = Mat(descriptors1).reshape(1,1);

  for(int j = 0; j < feature2.size(); j++){
    Mat descriptors_temp2 = feature2[j].getDescriptor();
    descriptors2.push_back(descriptors_temp2);
    //test feature2 id
    // cout << "feature2_id"<< j <<" :" << feature2[j].getId()<<endl;
  }
  // Mat descriptors2_reshape = Mat(descriptors2).reshape(1,1);

//check empty
  if ( descriptors1.empty() ){
   cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__);
  }
  if ( descriptors2.empty() ){
   cvError(0,"MatchFinder","2nd descriptor empty",__FILE__,__LINE__);
  }

//change type
//while SURF or SIFT is used, the type of the descriptor should be changed
  // if(descriptors1.type()!=CV_32F) {
  //   descriptors1.convertTo(descriptors1, CV_32F);
  // }

  // if(descriptors2.type()!=CV_32F) {
  //   descriptors2.convertTo(descriptors2, CV_32F);
  // }
  vector<long long> ids2;

//match
  vector<DMatch> allMatches;
  BFMatcher matcher (NORM_HAMMING);
  matcher.match(descriptors1, descriptors2, allMatches);
  //cout<<"number of matches before filtering: "<<allMatches.size()<<endl;

  // cout << "feature1: address after2:" << &feature1[0] <<endl;
//id update
  for(int i = 0; i < allMatches.size(); i++)
  {
  	long long id1 = allMatches[i].queryIdx;
  	long long id2 = allMatches[i].trainIdx;
  	long long id_temp = feature1[id1].getId();

  	//for feature1, allMatches[i] = feature[i]
  	feature2[id2].setId(id_temp);
  	ids2.push_back(id_temp);
    // cout << "allMatches size: " << allMatches.size() <<endl;
    // cout << "id1: " << id1 << endl;
    // cout << "id2: " << id2 << endl;
    // cout << "id_temp: " << id_temp <<endl;
    // cout << "feature1 id after update: "<< feature1[id1].getId()<<endl;
    // cout << "feature2 id after update: "<< feature2[id2].getId()<<endl;
  }

  // for(int j = 0; j < feature2.size(); j++){
  //   feature2[j].setId(j);
  //   cout << "feature2 " << j << "id" << feature2[j].getId() << endl;
  // }

  // for(int i = 0; i < feature1.size(); i++){
  //   //test feature1 id
  //   cout << "feature1_id after "<< i <<" :" << feature1[i].getId()<<endl;
  // }

  // for(int j = 0; j < feature2.size(); j++){
  //   //test feature2 id
  //   cout << "feature2_id after "<< j <<" :" << feature2[j].getId()<<endl;
  // }

  featureset2.setFeaturePoints(feature2);
 
  //-- calculate the max and in distance
  /**
  double maxDist = 0;
  double minDist = 100;
  for( int i = 0; i < descriptors1.rows; i++ )
  {
    double dist = allMatches[i].distance;
    if( dist < minDist )
      minDist = dist;
    if( dist > maxDist )
      maxDist = dist;
  }
  printf("  max dist : %f \n", maxDist );
  printf("  min dist : %f \n", minDist );
 
  //-- keep the good matches（norm here：distance<3*minDist）
  vector< DMatch > goodMatches;
  for( int i = 0; i < descriptors1.rows; i++ )
  {
    if( allMatches[i].distance < 3*minDist )
      goodMatches.push_back( allMatches[i]); 
  }
*/
}
