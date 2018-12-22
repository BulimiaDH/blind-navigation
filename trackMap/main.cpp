#include"trackMap.h"
#include<iostream>
int main()
{
	trackMap tMap;
	std::vector<double> data={1,0,0,0,1,0,0,0,1};
	cv::Matx44d aMat(1,0,0,0,0,1,0,0,0,0,1,1,0,0,0,1);
	
	//cv::Matx44d cMat(1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1);
	cv::Mat bMat(aMat);

	cv::Mat fakeImg;
	std::vector<cv::Mat> fakeImgSet;
	fakeImgSet.push_back(fakeImg.clone());

	std::vector<std::shared_ptr<blindfind::View> > allViewsInOneTrack;

	for(size_t i = 0; i<40; i++)
	{
		std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(fakeImgSet, 0); 
		newView->setPose(bMat);
		allViewsInOneTrack.push_back(newView);
	}
	
	std::vector<std::shared_ptr<blindfind::View> > allViewsInOneTrackTwo;
	for(size_t i = 0; i<18; i++)
	{
		std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(fakeImgSet, 0); 
		newView->setPose(bMat);
		allViewsInOneTrackTwo.push_back(newView);
	}

	
	tMap.addNewTrackTest(allViewsInOneTrack);
	tMap.addNewTrackTest(allViewsInOneTrackTwo);
	std::vector<double> result = {0.,1.,39.,0.,0.,0.,1.5707963};
	std::vector<std::vector<double> > results;
	results.push_back(result);
	tMap.addConstraints(results);
	tMap.addConstraints(results);
	tMap.optimize();
	return 0;
}
