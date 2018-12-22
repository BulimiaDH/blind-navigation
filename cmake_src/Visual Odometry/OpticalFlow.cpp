//
//  OpticalFlow.h
//  VisualSLAM
//
//  Created by Lan An on 7/24/18.
//  Copyright © 2018 Lan An. All rights reserved.
//

#include "OpticalFlow.h"
#include "FeatureMatcher.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

using namespace blindfind;
using namespace cv;
using namespace std;


void OpticalFlow::KLT(Mat firstimage, Mat secondimage){
    store_firstimage(firstimage);
    store_secondimage(secondimage);

    vector<uchar> status;
    vector<float> err;

    Mat image1Gray, image2Gray, image1, image2;
    image1Gray = firstimage;
    image2Gray = secondimage;
    // cout << "image size: "<< firstimage.size() << secondimage.size() << endl;
    //cvtColor(image1, image1Gray, CV_RGB2GRAY);//convert from RGB to gray
    goodFeaturesToTrack(image1Gray, points1, KLT_FEATURENUM, 0.01, 10, Mat());


    if (true)
    {
        goodFeaturesToTrack(image2Gray, points2, KLT_FEATURENUM, 0.01, 10, Mat());

        //Calculates an optical flow for a sparse feature set 
        //using the iterative Lucas-Kanade method with pyramids.
        calcOpticalFlowPyrLK(image1Gray, image2Gray, points1, points2, status, err, Size(60, 60), 3);  
        
        // some output to check
        // cout << "points1 size: "<< points1.size()<<endl;
        // cout << "points2 size: "<< points2.size()<<endl;

        // for(int j = 0; j < status.size(); j++){
        //     cout << "status"<< j <<": " << status[j] << " "<<endl;
        // }

        // for(int i = 0; i < err.size(); i++ ){
        //     cout <<"error"<< i <<": " << err[i] << " ";
        // }
    }
}


void OpticalFlow::Search_Around()//use ORB
{
    // distance = orb_distance(points1, points2);
    vector<double> distance_search;

    //Use KDTree to search the nearest orb point
    flann::KDTreeIndexParams indexParams(2);
    flann::Index kdtree(cv::Mat(points2_orb).reshape(1), indexParams);
    int num = 0;

    //Get the smaller size to match
    int size = min(points1.size(), points2.size());
    
    for(int i = 0; i < size; i++)
    {
        vector<float> query;
        query.push_back(points1[i].x);
        query.push_back(points1[i].y);
        // cout << "keypoints1 size: " << keypoints1_after.size()<<endl;

        int k = 2; //number of nearest neighbors
        vector<int> indices(k);//找到点的索引
        vector<float> dists(k);

        flann::SearchParams params(128);
        kdtree.knnSearch(query, indices, dists, k,params);
        int index1 = indices[0];
        int index2 = indices[1];

        double nearest_distance1 = sqrt((points2_orb[index1].x - points1[i].x)*(points2_orb[index1].x - points1[i].x) + (points2_orb[index1].y - points1[i].y) * (points2_orb[index1].y - points1[i].y));
        double nearest_distance2 = sqrt((points2_orb[index2].x - points1[i].x)*(points2_orb[index2].x - points1[i].x) + (points2_orb[index2].y - points1[i].y) * (points2_orb[index2].y - points1[i].y));

        if((nearest_distance1 * 2) < nearest_distance2){

        // cout << "search coordinate: x: " << points2_orb[index].x <<" y: " << points2_orb[index].y <<endl;
        // cout << "original coordinate: x:" << points1[i].x <<" y: "<< points1[i].y <<endl;
        // cout << "points1 size: "<< points1.size() <<endl;
        // cout << "points2 size: "<< points2.size() <<endl;

        //Abandon the point if it is too far for the original one
        if((abs(points2_orb[index1].x - points1[i].x) < 10) && (abs(points2_orb[index1].y - points1[i].y )<10))
        {
            // double distance = orb_distance( 0, 0, i);
            
            // Point2f p_original(points1_copy[i].x, points1_copy[i].y);

            // cout << "x_diff" << abs(points2_orb[index1].x - points1[i].x )<<endl;
            // cout << "y_diff" << abs(points2_orb[index1].y - points1[i].y )<<endl;

            Points2_update(points2_orb[index1], i);

            // double distance_search = orb_distance(index, 1, i);
            // cout <<"distance size: "<<distance_search.size()<<endl;
            // if(distance_search < distance)
            // {
            //     // distance_update(distance_search);
                // cout<<"update! " << num <<"\n";//show the update number
                num++;
            // }
            // else
            // {
            //     Points2_update(p_original, i);
            // }
        // cout<<"points2_orb size: "<< points2_orb.size()<<endl;
        }
    }



        if(points1[i].x > 0 && points1[i].x < 1241 && points1[i].y > 0 && points1[i].y < 1241 &&points2[i].x > 0 && points2[i].x < 1241 && points2[i].y > 0 && points2[i].y < 1241)
        {
              points1_copy.push_back(points1[i]);
              points2_copy.push_back(points2[i]);
        }
    }
    int points1_size = points1.size();
    int points1_copy_size = points1_copy.size();
    feature_maintenance_rate = (float)points1_copy_size/ (float)points1_size;
    cout << "points1_copy size: " << points1_copy.size() << endl;
    // cout<<"points2_copy size: "<< points2_copy.size() << endl;
    //     cout<<"points1 size: "<< points1.size() << endl;
    cout << "points2 size: " << points2.size() << endl;
    cout << "feature_maintenance_rate: " << feature_maintenance_rate << endl;
}


double OpticalFlow::orb_distance(int indx, int choice, int feature_num)
{
    
    // double dist;
    // cout << "feature_num"<< feature_num<<endl;

    Mat descriptors1 = points1_descriptors.row(feature_num);
    Mat descriptors2;
    // cout << "descriptors1 size: " << points1_descriptors.size() <<endl;
    // cout << "points1 size: " << points1.size() << endl;
    // cout << "descriptors2 size: " << points2_descriptors.size() <<endl;
    // cout << "points2 size: " << points2.size() << endl;
    if(choice == 0){
        descriptors2 = points2_descriptors.row(feature_num);
    }
    if(choice == 1)
    {
        descriptors2 = keypoints2_descriptors.row(indx);
    }
    
    double dist = norm(descriptors1, descriptors2, NORM_HAMMING);

    // match BRIEF descriptor，use Hamming distance
    // vector<DMatch> matches;
    // BFMatcher matcher ( NORM_HAMMING );
    // matcher.match(descriptors1, descriptors2, matches);
    return dist;
}

void OpticalFlow::orb_initialization()
{
    // vector<cv::KeyPoint> keypoints1;
    vector<cv::KeyPoint> keypoints2;
    vector<cv::KeyPoint> keypoints2_orb;

    //convert class Point2f to KeyPoint
    // for(int i = 0; i < points1.size(); i++){
    //     keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
    // } 
    for(int j = 0; j < points2.size(); j++){
        keypoints2.push_back(cv::KeyPoint(points2[j], 1.f));
    }

    //detect secondimage orb features
    Ptr<ORB> orb = ORB::create();
    orb->detect(secondimage, keypoints2_orb);
    //convert back
    points2_orb = KeyPoint_convert_to_Point2f(keypoints2_orb);

    //convert points1,points2
    // vector<KeyPoint> points1_key = Point2f_convert_to_KeyPoint(points1);
    // vector<KeyPoint> points2_key = Point2f_convert_to_KeyPoint(points2);



    //Get the descriptor and update the points to points_copy
    //There will be fewer points after orb
    // for(int i = 0; i < KLT_FEATURENUM; i++)
    // {
    //     Mat descriptors1_temp;
    //     Mat descriptors2_temp;
    //     vector<KeyPoint> points1_each;
    //     vector<KeyPoint> points2_each;
    //     points1_each.push_back(points1_key[i]);
    //     points2_each.push_back(points2_key[i]);
    //     // cout << "points1_each size: "<< points1_each.size()<<endl;
    //     // cout<< "points2_each size: "<< points2_each.size()<<endl;
    //     orb->compute(firstimage, points1_each, descriptors1_temp);
    //     orb->compute(secondimage, points2_each, descriptors2_temp);
    //     // cout << "descriptors1_temp size: "<< descriptors1_temp.size()<<endl;

    //     if((descriptors1_temp.empty() == false)&&(descriptors2_temp.empty() == false))
    //     {
    //         points1_descriptors.push_back(descriptors1_temp);
    //         points2_descriptors.push_back(descriptors2_temp);
    //         points1_copy.push_back(points1[i]);
    //         points2_copy.push_back(points2[i]);
    //     }
    //     // cout << "points1_each size: "<< points1_each.empty() << " des1 size "<< descriptors1_temp.empty()<< endl;
    //     // cout << "points2_each size: "<< points2_each.empty()<< " des2 size "<< descriptors2_temp.empty()<< endl;
    //     points1_each.clear();
    //     points2_each.clear();
    // }

    // compute descriptors in all, in this way some points will vanish
    // orb->compute(firstimage, points1_key, points1_descriptors);
    // orb->compute(secondimage, points2_key, points2_descriptors);

    // cout << "points1_copy size: " << points1_copy.size() << " points1_descriptors size: "<< points1_descriptors.size()<< endl;
    // cout << "points2_copy size: " << points2_copy.size() << " points2_descriptors size: "<< points2_descriptors.size()<< endl;
    //keypoints2
    // orb->compute(secondimage, keypoints2_orb, keypoints2_descriptors);
    
    //convert the type of keypoints
}



void OpticalFlow::display(){
    Mat image1Gray, image2Gray;
    image1Gray = firstimage;
    image2Gray = secondimage;
    Mat image2;
    std::vector<Point2f> points1Copy = getPoints1();
    std::vector<Point2f> points2Copy = getPoints2();
    // for(int j = 0; j< points1Copy.size(); j++ ){
    //     cout << "x1: " << points1Copy[j].x << " y1: "<< points1Copy[j].y<<endl;
    //     cout << "x2: " << points2Copy[j].x << " y2: "<< points2Copy[j].y<<endl;
    // }

    // for (int i = 0; i < points1Copy.size(); i++)    //绘制特征点位  
    // {
    //     circle(image1Gray, points1Copy[i], 1, Scalar(0, 0, 255), 2);
    // }
    // namedWindow("Optical Flow Feature Points");


    cvtColor(image2Gray, image2, CV_GRAY2RGB);
    for (int i = 0; i < points2Copy.size(); i++)
    {
        circle(image2, points2Copy[i], 1, Scalar(255, 0, 0), 3);
        // if(points1Copy[i].x > 0 && points1Copy[i].x < 1241 && points1Copy[i].y > 0 && points1Copy[i].y < 1241 &&points2Copy[i].x > 0 && points2Copy[i].x < 1241 && points2Copy[i].y > 0 && points2Copy[i].y < 1241){
        line(image2, points1Copy[i], points2Copy[i], Scalar(0, 0, 255), 2);  
        // }
    }
    imshow("Optical Flow Feature Points", image2);
        // sleep(5*1000);

    if(waitKey(0) == ' '){
        cv::destroyWindow("Optical Flow Feature Points");
    }
    //imshow("first image", image1Gray);
    // swap(points1, points2);
    // image1Gray = image2Gray.clone();
}

vector<Point2f> OpticalFlow::KeyPoint_convert_to_Point2f(vector<KeyPoint> keypoint){
    vector<Point2f> output;
    for(int i = 0; i< keypoint.size(); i++){
        Point2f temp(keypoint[i].pt.x, keypoint[i].pt.y);
        output.push_back(temp);
    }
    return output;
}

vector<KeyPoint> OpticalFlow::Point2f_convert_to_KeyPoint(vector<Point2f> point2f){
    vector<KeyPoint> output;
    for(int i = 0; i< point2f.size(); i++){
        output.push_back(cv::KeyPoint(point2f[i], 1.f));
    }
    return output;
}
