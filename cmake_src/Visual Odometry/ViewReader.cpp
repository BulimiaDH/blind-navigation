//
//  ImageReader.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/22/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "ViewReader.h"

using namespace blindfind;
using namespace std;
using namespace cv;
ViewReader::ViewReader(string _dataset, string _track, bool _stream, bool _stereo)
{
    dataset = _dataset;
    track = _track;
    stereo = _stereo;
	stream = _stream;
    index = START_FRAME;
    left_right = "0";
}

vector<Mat> ViewReader::next()
{
	if (stream == false)
	{
    	vector<Mat> images;
    	Mat leftImage;
		if (dataset == "KITTI")
		{
			string filePath("KITTI/");
			string leftRight("/image_0/");
			string fileDir = IMG_DIR + filePath + track + leftRight;
			fileDir = fileDir + string(6 - to_string(index).size(), '0') + to_string(index) + ".png";
			leftImage = cv::imread(fileDir, 0);   // Read the file
			images.push_back(leftImage);

			if(stereo)
			{
				Mat rightImage;
				string filePath("KITTI/");
				string leftRight("/image_1/");
       		 	string fileDir = IMG_DIR + filePath + track + leftRight;
       		 	fileDir = fileDir + string(6 - to_string(index).size(), '0') + to_string(index) + ".png";
       		 	rightImage = cv::imread(fileDir, 0);   // Read the file
       		 	images.push_back(rightImage);

			}

		}
		else if (dataset == "MALL")
		{
		}
    	++index;
  		return images;
	}
	else //TODO:for streaming
	{
		vector<Mat> images;
		return images;
	}
}


vector<Mat> ViewReader::current()
{
	if (stream == false)
	{
    	vector<Mat> images;
    	Mat leftImage;
		if (dataset == "KITTI")
		{
			string filePath("KITTI/");
			string leftRight("/image_0/");
			string fileDir = IMG_DIR + filePath + track + leftRight;
			fileDir = fileDir + string(6 - to_string(index).size(), '0') + to_string(index - 1) + ".png";
			leftImage = cv::imread(fileDir, 0);   // Read the file
			images.push_back(leftImage);

			if(stereo)
			{
				Mat rightImage;
				string filePath("KITTI/");
				string leftRight("/image_1/");
       		 	string fileDir = IMG_DIR + filePath + track + leftRight;
       		 	fileDir = fileDir + string(6 - to_string(index).size(), '0') + to_string(index - 1) + ".png";
       		 	rightImage = cv::imread(fileDir, 0);   // Read the file
       		 	images.push_back(rightImage);

			}

		}
		else if (dataset == "MALL")
		{
		}
    	++index;
  		return images;
	}
	else //TODO:for streaming
	{
		vector<Mat> images;
		return images;
	}
}
//vector<Mat> ViewReader::next()
//{
//    vector<Mat> images;
//    Mat leftImage;
//    string fileDir = IMAGE_DIR + track + "/";
//    fileDir = fileDir + "image_" + left_right + "/" + string(6 - to_string(index).size(), '0') + to_string(index);
//    fileDir = fileDir + ".png";
//    //    left_right = left_right == "0" ? "1" : "0";
//    
//    leftImage = imread(fileDir, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
//    images.push_back(leftImage);
//    if(stereo)
//    {
//        Mat rightImage;
//        string fileDir = IMAGE_DIR + track + "/";
//        fileDir = fileDir + "image_1/" + string(6 - to_string(index).size(), '0') + to_string(index);
//        fileDir = fileDir + ".png";
//        rightImage = imread(fileDir, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
//        images.push_back(rightImage);
//    }
//    ++index;
//    return images;
//}
