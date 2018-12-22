//
//  rotationEstimation.hpp
//  enclose
//
//  Created by Fangzhou Xie on 7/27/18.
//  Copyright © 2018 Fangzhou Xie. All rights reserved.
//

#ifndef rotationEstimation_hpp
#define rotationEstimation_hpp

#include <iostream>
#include <algorithm>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace std;
using namespace xfeatures2d;

class rotationEstimation {
private:
    constexpr static float ratioTest = 0.7;
    //constexpr static float testThreshold = 0.02;
    constexpr static float testThreshold = 2;
    //constexpr static float inlierThreshold = 0.01;
    //constexpr static float ransacPortion = 0.5;
    constexpr static float pi = 3.1415926;
    bool overlap = 0;
public:
    float estimate(Mat img1, Mat img2, float fx);
    bool do_overlap() {
	    return overlap;
    }
};

#endif /* rotationEstimation_hpp */
