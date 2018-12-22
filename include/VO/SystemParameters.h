//
// SystemParameters.h
// VisualSLAMKimia
//
// Author: Hongyi Fan 
// Time: 06/02/2018

#ifndef SYSTEMPARAMETERS_H_
#define SYSTEMPARAMETERS_H_

#define FEATURE_NAME "ORB"
#define DESCRIPTOR_NAME "ORB"
#define FEATURE_REDETECTION_TRIGGER 200

//For ORB Feature
#define ORB_SCALE 2
#define ORB_FEATURENUM 2000
#define ORB_NLEVELS 6

//For SURF Feature


//For KLT Feature
#define KLT_FEATURENUM 2000

#define NUM_POSES 500


#define EPIPOLAR_THRESHOLD 0.0001

#define BUNDLE_ADJUSTMENT_LENGTH 5

#define REPROJECTION_THRESHOLD 3.0

#define RANSAC_CONFIDENCE 0.95

#define DATA_NAME "KITTI"
#define TRACK_NUM "00"
#define START_FRAME 0

#define IMG_HEIGHT 376
#define IMG_WIDTH 1241

#define IMG_DIR "/home/lems/intern_2018/Andy/"
#define GROUNDTRUTH_DIR "/home/hongyi/Documents/SLAMData/GT/"
#define OUTPUT_DIR "/home/lems/intern_2018/Andy/results/"

//Camera Parameters
#define STEREO 1
#define FOCAL_X 718.856
#define FOCAL_Y 718.856
#define PRINCIPAL_X 607.1928
#define PRINCIPAL_Y 185.2157
#define BASELINE 386.1448


#endif
