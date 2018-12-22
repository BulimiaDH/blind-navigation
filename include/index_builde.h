#include <iostream>
#include <vector>
#include <dirent.h>
#include <stdio.h>
#include <sys/time.h>
#include<CNNExtractor.h>
//Add DBoW library
#include "DBoW3/DBoW3.h"
/*
//Add DLib library
#include "DUtils/DUtils.h"
#include "DUtilsCV/DUtilsCV.h" // defines macros CVXX
#include "DVision/DVision.h"
*/
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
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif

using namespace std;
using namespace DBoW3;
using namespace cv;
using namespace cv::xfeatures2d;

class index_builder
{
public:
	//Constructors
	index_builder();
	index_builder(string path_of_dataset_image, string feature_name, string matching_method, string feature_output, bool);
	~index_builder();
	
	//Set parameter for vocabulary tree method
	//K: cluster in each level
	//L: The depth of one level
	void setParam(int K, int L);

	//List file names of one folder. This function used for load name of images
	//target_path: the path constains images
	vector<string> list_names(const char* target_path);
	vector<string> load_image_list(string path);
	void build_index();
	void extract_features();
	void changeStructure(const vector<float> &plain, vector<vector<float> > &out, int L);
	void setCrossNum(int cross_num);

private:
	 void extract_SURF();
	 void extract_ORB();
	 void extract_CNN();
	 string path_of_dataset_;
	 string feature_name_;
	 string matching_method_;
	 string feature_output_;
	 vector<string> image_list_;
	 vector<string> indexed_image_list_;
	 int cross_num_;
	 bool cross_validation_;
	 Ptr<Feature2D> detector_;
	 vector<cv::Mat> descriptors_;
	 CNNExtractor* CNNDetector_;
	 int K_; //cluster in each level.
	 int L_;  // depth of one level.
};
