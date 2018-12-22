/***************************
  track complementor Implementation
  Created by Han Deng
  Aug. 16. 2018
  *************************/
#include"trackMap.h"
#include <stdlib.h>   
#include<iostream>
#include<string>
#include<fstream>
#include<memory>
#include<thread>
#include<chrono>
#include <dirent.h>
#include<algorithm>
//#include<conio.h>
#include<ps4/PS4CameraManager.h>

#define RAD2DEG(x) ((x)*180./M_PI)  
#define DEG2RAD(x) ((x)*M_PI/180.)

static bool globalConstant=0;
static bool useCamera=false;

const double step_size=0.033; //According to 20fps 0.067m/frame my step size
template<typename numberType>
numberType String2Number(std::string aString)
{
	std::stringstream stream(aString);
	numberType returnValue;
	stream>>returnValue;
	return returnValue;
}

template<typename numberType>
std::string Number2String(numberType number)
{
	std::stringstream strStream;
	strStream<<number;
	std::string returnValue = strStream.str();
	return returnValue;
}
typedef struct trackInfo{

public:
    std::vector<std::vector<std::vector<double> > > relationInOneTrack;
    std::vector<std::vector<double> > relationBetweenTracks;
    double stepSize;

}trackInfo;
bool comp(const std::string& name1, const std::string& name2);
std::vector<std::string> listNames(const std::string& folderName);
std::vector<std::string> imageListRead(const std::string& fileDst);
std::vector<std::string> split(const std::string &s, const std::string &seperator);
std::shared_ptr<trackInfo> mapProcess(const std::string& mapFilePath);

int main()
{
    std::string trackAllPath = "/home/blindfind/Documents/Blindfind3/tools/imageCollector/build/all/trackAll";
    std::string imageListPath = "/home/blindfind/Documents/Blindfind3/tools/imageCollector/build/all/result.txt";
    std::string mapFilePath = "/home/blindfind/Documents/Blindfind3/data/map_new.txt";
    std::string newTrackPath = "/home/blindfind/Documents/Blindfind3/tools/imageCollector/build/track2/";///home/blindfind/Documents/Blindfind3/tools/imageCollector/build/track1/";///home/blindfind/Documents/Blindfind3/data/Ready/track2/";
   // std::vector<std::string> imageList = imageListRead(imageListPath);
    std::shared_ptr<trackInfo> infoTracks = mapProcess(mapFilePath);
    std::cout<<"Number of Tracks: "<<infoTracks->relationInOneTrack.size()<<std::endl;
    //First, add existing nodes to the iSAM.
    trackMap tMap(trackAllPath+"/");
    for(size_t j = 0; j<infoTracks->relationInOneTrack.size(); j++)
    {
        
        std::vector<std::shared_ptr<blindfind::View> > allViewsInOneTrack;   
        
        for(size_t k  = 0; k<infoTracks->relationInOneTrack[j].size();k++)
        {
            
            if(k==0)
            {
                std::vector<cv::Mat> fakeImgSet;
                cv::Matx44d aMat(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,0,1);
                cv::Mat bMat(aMat);
                cv::Mat fakeImg;
                fakeImgSet.push_back(fakeImg.clone());
                fakeImgSet.push_back(fakeImg.clone());
                std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(fakeImgSet, 0); 
                newView->setPose(bMat);
                allViewsInOneTrack.push_back(newView); //This is the initial view.
            }
            std::vector<cv::Mat> fakeImgSet;
            cv::Matx44d aMat(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,0,1);
            aMat(0,3) = infoTracks->relationInOneTrack[j][k][1];
            aMat(2,3) = infoTracks->relationInOneTrack[j][k][0];
            cv::Mat bMat(aMat);
            cv::Mat fakeImg;
            fakeImgSet.push_back(fakeImg.clone()); 
            fakeImgSet.push_back(fakeImg.clone());
            std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(fakeImgSet, 0); 
            newView->setPose(bMat);
		    allViewsInOneTrack.push_back(newView);
        }
        tMap.addNewTrackTest(allViewsInOneTrack, false);
        
    }
    tMap.addConstraints(infoTracks->relationBetweenTracks);
  //  tMap.optimize();
  //  
  

    //Second, the audio synthesizer tells the person to begin, the track begins

    /*Initialzie the PS4CameraManager*/
    std::vector<std::shared_ptr<blindfind::View> > allViewsInOneTrack;
    if(useCamera)
    {
        std::thread cameraThread(PS4CameraManager::threadMain, std::string("test"), false);
        while(!globalConstant)
        {
            PS4CameraFrame Frame = PS4CameraManager::Instance().getLastestFramesPrimary();
            cv::Mat image_left(Frame.height, Frame.width, CV_8UC3, Frame.leftFrame.get());
            cv::Mat image_right(Frame.height, Frame.width, CV_8UC3, Frame.rightFrame.get()); //Dosen't implement the 

            cv::flip(image_left, image_left, 1);  //Do the flip but not grey scale transformation, which is delegated to imageMatcher
            cv::flip(image_right, image_right,1);

            std::vector<cv::Mat> imgSet;
            imgSet.push_back(image_left.clone());
            imgSet.push_back(image_right.clone());
            std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(imgSet, 0);
            cv::Matx44d aMat(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,0,1);
            aMat(2,3) = (infoTracks->stepSize)*2; //Needs to be re-considered. The stepSize;
            cv::Mat bMat(aMat);
            newView->setPose(bMat);
            allViewsInOneTrack.push_back(newView);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    else
    {
        std::vector<std::string> leftList = listNames(newTrackPath+"left");
        std::vector<std::string> rightList = listNames(newTrackPath+"right");
        std::sort(leftList.begin(), leftList.end(), comp);
        std::sort(rightList.begin(), rightList.end(), comp);
        for(size_t k =0; k<leftList.size(); k++)
        {
            std::vector<cv::Mat> imgSet;
            cv::Mat leftImage = cv::imread(newTrackPath+"left/"+leftList[k], CV_LOAD_IMAGE_GRAYSCALE);//, CV_LOAD_IMAGE_GRAYSCALE);
            cv::Mat rightImage = cv::imread(newTrackPath+"right/"+rightList[k], CV_LOAD_IMAGE_GRAYSCALE);//,CV_LOAD_IMAGE_GRAYSCALE);
            //cv::imshow("image", leftImage);
            //cv::waitKey(0);
            imgSet.push_back(leftImage.clone());
            imgSet.push_back(rightImage.clone());
            std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(imgSet, 0);
            cv::Matx44d aMat(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,0,1);
            aMat(2,3) = (infoTracks->stepSize); //Needs to be re-considered. The stepSize;
            cv::Mat bMat(aMat);
            newView->setPose(bMat);
            allViewsInOneTrack.push_back(newView);
        }
         tMap.addNewTrackTest(allViewsInOneTrack, true);
    }
    tMap.optimize();
    return 0;
}
bool comp(const std::string& name1, const std::string& name2)
{
   // std::cout<<"666"<<std::endl;
   // std::cout<<name1<<std::endl;
   // int number1 = String2Number<int>(split(name1, ".")[0]);
   // int number2 = String2Number<int>(split(name2, ".")[0]); 
    int number1 = String2Number<int>(split(split(name1, "_")[2], ".")[0]);
    int number2 = String2Number<int>(split(split(name2, "_")[2], ".")[0]); 
   // std::cout<<number1<<" "<<number2<<std::endl;
    return number1<number2;
}

std::vector<std::string> listNames(const std::string& folderName)
{
    
    std::vector<std::string> allNames;
    DIR * dir;
    struct dirent *ent;
    std::vector<std::string> fileList;
    dir = opendir(folderName.c_str());
    if(dir!=NULL)
    {
        while((ent=readdir(dir))!=NULL)
        {
           // std::cout<<"We are going to list our names"<<std::endl;
            std::string s;
            s=ent->d_name;
            unsigned posOfPeriod = s.find_last_of(".");
            std::string substr  = s.substr(posOfPeriod+1);

            if (substr.compare("bmp") == 0 || substr.compare("BMP") == 0 || substr.compare("jpg") == 0 || substr.compare("JPG") == 0 ||substr.compare("jpeg") == 0 || substr.compare("JPEG") == 0 ) 
            {
                allNames.push_back(s);
            }
            else
            {
                continue;
            }
        }
    }
    closedir (dir);
    return allNames;
}

std::vector<std::string> imageListRead(const std::string& fileDst)
{
    fstream exFile;
    std::vector<std::string> imageList;
    exFile.open(fileDst, ios::in);
    char line[256];
    if(!exFile.is_open())
    {
        std::cerr<<"There is sth wrong about the file(path)"<<std::endl;
        exit(-1);
    }
    while(!exFile.eof())
    {
        exFile.getline(line, 100);
     //   std::cout<<line<<std::endl;
        imageList.push_back(line);
    }
    exFile.close();
    return imageList;
}

std::vector<std::string> split(const std::string& s,  const std::string& c){
    std::vector<string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));

    return v;
}

std::shared_ptr<trackInfo> mapProcess(const std::string& mapFilePath)
{
    //不属于同一条track的条件（满足任意一个）： 1. 方向发生了变化 2. 坐标同时发生了变化 3. 单个坐标发生大的变化.
    //每当发现了一个新的track，就添加上个track和这个新track(用ID表示)的关系。角度可以计算，坐标差也可以计算（坐标差一定要根据现在的朝向和新的朝向之间的关系来计算）
    //如果现在朝向是0，就是正常的坐标差， 否则需要重新计算。
    //可以先尝试一下基本的constraint能否建立合适的地图。

    std::shared_ptr<trackInfo> info = std::make_shared<trackInfo>();
    fstream exFile;
    exFile.open(mapFilePath, ios::in);
    char line[256];
    if(!exFile.is_open())
    {
        std::cerr<<"There is sth wrong about the file(path)"<<std::endl;
        exit(-1);
    }
    int iterNum=0;
    int trackId = 0;
    double oldXCoor=0;
    double oldYCoor=0;
    double oldAngle=0;
    int allSteps=0;
    while(!exFile.eof())
    {
        //std::cout<<"666"<<std::endl;
        exFile.getline(line, 100);
      //  std::cout<<line<<std::endl;
        std::vector<std::string> result = split(line, " ");
        double xCoor = String2Number<double>(result[0]);
        double yCoor = String2Number<double>(result[1]);
        double angle = String2Number<double>(result[2]);
        //Only for test
        //std::cout<<xCoor<<" "<<yCoor<<" "<<angle<<" "<<std::endl;
        if(iterNum==0)
        {
            oldXCoor = xCoor;
            oldYCoor = yCoor;
            oldAngle = angle;
            std::vector<std::vector<double > > oneRelation;
            info->relationInOneTrack.push_back(oneRelation);
            ++iterNum;
            continue;
        }
        else
        {
            if(iterNum%2!=0)
            {
                ++iterNum;
                continue;
            }
            ++iterNum;
            bool flag1=0;
            bool flag2=0;
            bool flag3=0;
            bool finalFlag=0;
            if((oldXCoor-xCoor==0&&oldYCoor-yCoor!=0)||(oldXCoor-xCoor!=0&&oldYCoor-yCoor==0)||(oldXCoor-xCoor==0&&oldYCoor-yCoor==0))
                flag1=1;
            if(oldAngle==angle)
                flag2=1;
            if(fabs(oldXCoor-xCoor)<1&&fabs(oldYCoor-yCoor)<1)
                flag3=1;
            if(flag1&&flag2&&flag3)
                finalFlag=1;
            if(finalFlag)
            {
                std::vector<double> relation;
                if(xCoor-oldXCoor==0)
                {
                    relation={fabs(yCoor-oldYCoor), 0, 0};
                    info->stepSize+=fabs(yCoor-oldYCoor);
                    ++allSteps;
                }
                else
                {
                    relation={fabs(xCoor-oldXCoor), 0, 0};
                    info->stepSize+=fabs(xCoor-oldXCoor);
                    ++allSteps;
                }
                info->relationInOneTrack[trackId].push_back(relation);
            }  
            else
            {
                ++trackId;
                std::vector<std::vector<double > > oneRelation;
                info->relationInOneTrack.push_back(oneRelation);
                std::vector<double> relation;
              //  if(DEG2RAD(oldAngle-angle)==0)
              //      std::cout<<"XCoor-oldXCoor: "<<xCoor-oldXCoor<<"yCoor-oldYCoor: "<<yCoor-oldYCoor<<std::endl;
                if(oldAngle==0)
                    relation = {trackId-1, trackId, info->relationInOneTrack[trackId-1].size()-1, 0., xCoor-oldXCoor, yCoor-oldYCoor, DEG2RAD(oldAngle-angle)};
                else if(oldAngle==270)
                    relation = {trackId-1, trackId, info->relationInOneTrack[trackId-1].size()-1, 0., yCoor-oldYCoor, -(xCoor-oldXCoor), DEG2RAD(oldAngle-angle)};
                else if(oldAngle==180)
                    relation = {trackId-1, trackId, info->relationInOneTrack[trackId-1].size()-1, 0.,-(xCoor-oldXCoor), -(yCoor-oldYCoor), DEG2RAD(oldAngle-angle)};
                else
                    relation = {trackId-1, trackId, info->relationInOneTrack[trackId-1].size()-1, 0., -(yCoor-oldYCoor), xCoor-oldXCoor, DEG2RAD(oldAngle-angle)};
                
                info->relationBetweenTracks.push_back(relation);
            }
            oldXCoor = xCoor;
            oldYCoor = yCoor;
            oldAngle = angle;
        }
    }
    info->stepSize/=allSteps;
    exFile.close();
    return info;
}
