#include "FrameTracker.h"
#include <ctime>
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"  //FlannBasedMatcher
#include "opencv2/calib3d/calib3d.hpp"  //findHomography
#include "unistd.h"

using namespace blindfind;
using namespace cv;
using namespace std;  

void Derived::trackForward(View* view1, View* view2){
  // FeatureSet featureset1 = view1->getLeftFeatureSet();
  // FeatureSet featureset2 = view2->getLeftFeatureSet();

  //cout << "AAAAAAA:" <<&featureset1 << &(view1->getLeftFeatureSet()) << endl;

// //test address  
//   vector<Feature> feature1 = featureset1.getFeaturePoints();
//   cout<<"feature address1 before: " << &feature1[0] <<endl;
//   vector<Feature> feature2 = featureset2.getFeaturePoints();
//   cout<<"feature address2 before: " << &feature2[0] <<endl;
  FeatureMatcher* FM12 = new FeatureMatcher();


  FM12->FeatureMatch(view1->getLeftFeatureSet(), view2->getLeftFeatureSet());

  //cout << view2->getLeftFeatureSet().getFeaturePoints().size() <<"   " <<view2->getLeftFeatureSet().getIds().size() <<endl;
  // for (int i = 0; i < view2->getLeftFeatureSet().size(); i++)
  // // {
  //   cout << (view2->getLeftFeatureSet().getIds())[i] << endl;
  // }

  if(view1->isStereo()){
      FeatureMatcher* FM34 = new FeatureMatcher();
  //     FeatureSet featureset3 = view1->getRightFeatureSet();
  //     FeatureSet featureset4 = view2->getRightFeatureSet();
      FM34->FeatureMatch(view1->getRightFeatureSet(), view2->getRightFeatureSet());

//test address 
      // vector<Feature> feature3 = featureset3.getFeaturePoints();
      // cout<<"feature address3 before: " <<  &feature3[0] <<endl;
      // vector<Feature> feature4 = featureset4.getFeaturePoints();
      // cout<<"feature address4 before: " <<  &feature4[0] <<endl;
  }

}

void Derived::trackStereo(View* view1){
  FeatureMatcher* FM56 = new FeatureMatcher();
  // FeatureSet featureset5 = view1->getLeftFeatureSet();
  // FeatureSet featureset6 = view1->getRightFeatureSet();
  FM56->FeatureMatch(view1->getLeftFeatureSet(), view1->getRightFeatureSet());
}


//Optional
void Derived::refineMatching(View* view1, View* view2){

}

//More function to do


//helpfunction
