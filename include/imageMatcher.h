#ifndef __ImageMatcher_H_
#define __ImageMatcher_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
using namespace std;
#include <algorithm>
#include <utility>
#include <dirent.h>
#include <stdio.h>
#include <sys/time.h>
//Add DBoW library
#include "DBoW3/DBoW3.h"
#include<CNNExtractor.h>
//Add DLib library
//##include "DUtils/DUtils.h"
//##include "DUtilsCV/DUtilsCV.h" // defines macros CVXX
//##include "DVision/DVision.h"
//Add Boost
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
//Add OpenCV
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include<opencv2/core/core.hpp>

#include<View.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include<opencv2/imgproc/imgproc.hpp>

#if CV24
#include <opencv2/features2d/features2d.hpp>
#endif


using namespace DBoW3;
using namespace cv;
using namespace cv::xfeatures2d;
//using namespace DUtils;

class imageMatcher
{
public:
	//Default Constructor
	imageMatcher();

	//Constructor for static test. load query image from folder
	imageMatcher(string query_image_path, string indexed_image_list_file, 
			     string database_file_path, string feature_name,
				 string matching_method);

	//Constructor for practicl use. query image as Mat file
	imageMatcher(string indexed_image_list_file, string database_file_path, string vocabulary_file_path, string feature_name, string matching_method, bool whetherNew);

	~imageMatcher();

	//Name Loader
	vector<string> list_names(const char* target_path);
	vector<string> load_image_list(string path);
	
	//Initial Matcher
	void changeStructure(const vector<float> &plain, vector<vector<float> > &out, int L);

	void addFeatures();
	void addFeatures(std::shared_ptr<blindfind::View>  imgSet);
	void addFeatures(cv::Mat image);

	void saveDatabase(std::string databasdDir);

	float compareTwoImages(cv::Mat image1, cv::Mat image2);

	vector<pair<int, float> > matching_one_image(string query_image);
	vector<pair<int, float> > matching_one_image(cv::Mat image, double threshold);

	vector<pair<int, float> > matching_one_image(std::shared_ptr<blindfind::View> view, double threshold);
	vector<pair<int,int> > matching_images(std::shared_ptr<blindfind::View> view1, std::shared_ptr<blindfind::View> view2, double threshold);
	
	vector<pair<int, int> > matching_images(cv::Mat image1, cv::Mat image2, double threshold);
	vector<pair<int, int> > matching_images(string query1, string query2);

	vector<pair<int, float> > thresholding(vector<pair<int,float> > full_list, float threshold);
	std::string getFeatureLayer(){return this->_featureLayer;};
	void setFeatureLayer(std::string featureLayer){this->_featureLayer = featureLayer;};

private:
	string query_Images_Path_;
	string feature_name_;
	string matching_method_;
	string indexed_image_information_;
	std::vector<cv::Mat> feature_in_one_track_;
	vector<string> indexed_image_list_;
	vector<string> query_list_;
	int cross_num_;
	std::string _featureLayer;
	Database db_;
	std::vector<cv::Mat> full_connection_feature_set_;
	DBoW3::Vocabulary vocab;
	Ptr<Feature2D> detector_;
	CNNExtractor* CNNDetector_;
};

#endif
