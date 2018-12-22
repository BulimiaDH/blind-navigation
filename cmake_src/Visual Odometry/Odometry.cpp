// Odometry.cpp
//
// VisualSLAM
//
// Author: Hongyi Fan
// Time: 06/03/2018
//

#include "Odometry.h"
#include <assert.h>
#include <stdio.h>
#include <time.h>

using namespace std;
using namespace cv;
using namespace blindfind;

void Odometry::runDissectingScale(std::string outputFile)
{
	//Initialize All the values
	ViewReader *imageReader = new ViewReader(DATA_NAME,TRACK_NUM,false,true);	
	ORBExtractor *extractor = new ORBExtractor();
	FrameTracker* FT = new Derived();
	KeyFrame_Selector* KF = new KeyFrame_Selector();
	KF-> Initialization();
	FeatureExtractor *generalExtractor;
	generalExtractor = dynamic_cast<FeatureExtractor*>(extractor);
		
	//initialization
	vector<Mat> images = imageReader->next();
	View* firstView = new View(images,0);			
	_allViews.push_back(firstView);
	generalExtractor->extractFeatureAndDescriptor(_allViews[0]);

	//Canvas::drawFeaturePoints(_allViews[0],0,"Features");
	//Loop among all the frames
	for (int i = 1; i <= NUM_POSES; i++)
	{
		//Read next image
		vector<Mat> formerImage = imageReader->current();
		vector<Mat> currentImage = imageReader- >next();
		//initialize next view;
		View* currentView = new View(currentImage,i);
		//cout <<"ID:"<< currentView->getId() << endl;
		//cout <<"isStereo():"<< currentView->isStereo() << endl;
		//cout << currentView->getId() << endl;
		_allViews.push_back(currentView);

		//test the id
		// FeatureSet featureset1 = _allViews[i-1]->getLeftFeatureSet();
		// vector<Feature> feature1 = featureset1.getFeaturePoints();
		// for(int j = 0; j < feature1.size(); j++)
		// {
		//     cout << "feature1_id "<< j-1 <<" :" << feature1[j].getId()<<endl;
	 //    }


		//ORB Extraction Test
		generalExtractor->extractFeatureAndDescriptor(_allViews[i]);

// }
		clock_t start,end;
		float cost;
		start = clock();
		OpticalFlow* OF = new OpticalFlow();
		OF->KLT(formerImage[0], currentImage[0]);//left
		// FT->OpticalFlow(formerImage[1], currentImage[1]);//right
		OF->orb_initialization();
		OF->Search_Around();
		OF->display();
		float new_rate = OF-> getRate();
		cout << "new_rate: "<< new_rate<<endl;
		end = clock();
		float t1 = start;
		float t2 = end;
		// printf ("time cost[%d]: %0.3fms\n", i, (t2-t1)/1000);

		delete OF;

		//For visualization:
		//Canvas::drawFeaturePoints(_allViews[i],0,"Features");

		//Output view ID

		//Check keyframe or not
		bool key_frame_decision = KF->KeyFrame_Dicision(new_rate, i); 
		cout << "key_frame_decision: " << key_frame_decision <<endl;
		cout << "keyframe_nums" << endl;
		vector<int> keyframe_nums = KF->Output_Keyframe_nums();
		for(int m = 0; m < keyframe_nums.size(); m++){
			cout << keyframe_nums[m] <<endl;
		}

		//TODO:Tracking & Matching
		//To Lan: Several Things needs to be done here.
		//(1) Match current view and previous view
		//(2) Update FeatureSet for Each view. The same correspondence needs to have the same 

		int former_keyframe = keyframe_nums[keyframe_nums.size()-2];
		if(key_frame_decision == true){
			FT->trackForward(_allViews[former_keyframe], _allViews[i]);
			if(_allViews[i]->isStereo()){
				FT->trackStereo(_allViews[i]);
			}
		}
		// cout<< "former_keyframe: "<<former_keyframe<<" i: "<< i<<endl;
		// cout << "first ID: " << _allViews[i-1]->getId()<<endl;
		// cout << "second ID: " << _allViews[i]->getId()<<endl;
		//(3) Then Go on		

		//If it is a keyframe, compute the pose and scale


		//For every N frames fo bundle adjustment.

		//Save Files
	
}
	//Delete All the variables
	delete imageReader;
	delete extractor;
	//delete FT;
}


void Odometry::runDenseOdometry(std::string outputFile)
{
	//Initialize All the values

	//initialization

	//Loop among all the frames
	for (int i = 1; i <= NUM_POSES; i++)
	{
		//Read next image
		//initialize next view
		//Check keyframe or not
		//If it is a keyframe, compute the pose

		//For every N frames fo bundle adjustment.
	}
}

//void Odometry::saveView(std::string filename, View* view)
//{
//	ofstream output;
//	output.open(OUTPUT_DIR + TRACK_NUM + filename + ".output", std::ofstream::out | std::ofstream::app);
//	for (int row = 0; row < 4; row++)
//	{
//		for (int col = 0; col < 4; col++)
//		{
//			if (col != 0; col != 0)
//				output << " ";
//			output << double(pose.at<double>(row, col));
//			if(col == 3 && row == 3)
//				output << "\n";
//		}
//	}
//
//}

vector<Mat> Odometry::readGroundTruth(string track)
{
	vector<Mat> poses;
	string gtFileDir = GROUNDTRUTH_DIR + track + ".txt";
	ifstream in;
	in.open(gtFileDir,std::ifstream::in);

	assert(in.good());

	double num = 0.0;
	vector<double> nums;
	
	while(in >> num)
	{
		nums.push_back(num);
	}
	unsigned int i = 0;
	while (i < nums.size())
	{
		Mat pose = Mat::eye(4,4,CV_64F);
		for(int j = 0; j < 12; j++, i++)
		{
			pose.at<double>(j / 4, j % 4) = nums[i];
		}
		poses.push_back(pose.clone());
	}
	vector<Mat>::iterator start = poses.begin() + START_FRAME, end = start + NUM_POSES;
	poses = vector<Mat>(start, end);
	in.close();
	return poses;
}
