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
int main()
{
    /*
    Point2f c(0, 0);
	Point2f pt1(0, -1);
	Point2f pt2(0.01, 1);
 
	float theta = getAngelOfTwoVector(pt1, pt2, c);
 
	cout << "theta: " << theta << endl;
    */
    
    std::string testDir = "/home/blindfind/Documents/Blindfind3/testDataset/Test/test2/";
    std::string trackAllPath = "/home/blindfind/Documents/Blindfind3/tools/imageCollector/build/all/trackAll";
    std::vector<std::string> allTracksId = {"track1", "track2", "track3", "track4"};//, "track2"};
    std::vector<std::vector<std::string> > eachImagesInTracks;
    trackMap tMap(trackAllPath+"/");
    tMap.setThreshold(0.02,1.5);
    for(size_t k=0; k<allTracksId.size(); k++)
    {
        std::string imageList = testDir+allTracksId[k]+".txt";
        eachImagesInTracks.push_back(imageListRead(imageList));
    }
    for(size_t i=0; i<allTracksId.size(); i++)//eachImagesInTracks.size(); i++)
    {
        double xCoorPrev=0;
        double yCoorPrev=0;
        double anglePrev=0;
        std::vector<std::shared_ptr<blindfind::View> > allViewsInOneTrack;
        for(size_t j=0; j<eachImagesInTracks[i].size(); j++)
        {
            std::vector<std::string> splittedResult = split(eachImagesInTracks[i][j], "_");
            double xCoordinate = String2Number<double>(splittedResult[2].substr(1));
            double yCoordinate = String2Number<double>(splittedResult[3].substr(1));
            double angle = String2Number<double>(splittedResult[4].substr(1, splittedResult[4].size()-5));

            if(j==0)
            {
                xCoorPrev = xCoordinate;
                yCoorPrev = yCoordinate;
                anglePrev = angle;
                std::string totalImagePath=testDir+allTracksId[i]+"/"+eachImagesInTracks[i][j];
                cv::Mat monocularImage = cv::imread(totalImagePath);//, CV_LOAD_IMAGE_GRAYSCALE);
                if(monocularImage.empty())
                {
                    std::cout<<"Image empty"<<std::endl;
                    exit(0);
                }
                std::vector<cv::Mat> imgSet;
                imgSet.push_back(monocularImage.clone());
                std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(imgSet, 0);
                if(i==0)
                {
			//	    std::vector<double> relation={0,0,0};
					std::vector<double> relation = {xCoordinate, yCoordinate, angle};
                    newView->setRelationPrev(relation);
                    newView->setGroundTruthInfo(xCoordinate, yCoordinate, angle);
                }
                else
                {
                 //   std::vector<double> relation = {xCoordinate, yCoordinate, angle};
                    std::vector<double> relation = {0,0,0};
                    newView->setRelationPrev(relation);
                    newView->setGroundTruthInfo(xCoordinate, yCoordinate, angle);
                }    
                allViewsInOneTrack.push_back(newView);
            }
            else
            {
                Eigen::Vector2d worldCoor;
                worldCoor<<xCoordinate, yCoordinate;
                Eigen::Vector2d prevWorldCoor;
                prevWorldCoor<<xCoorPrev,yCoorPrev;
                Eigen::Matrix2d transMat;
                transMat<<cos(anglePrev), sin(anglePrev), -sin(anglePrev), cos(anglePrev);
                Eigen::Vector2d newCoor = transMat*worldCoor;
                Eigen::Vector2d  prevNewCoor = transMat*prevWorldCoor;
                double xDiff = newCoor[0]-prevNewCoor[0];
                double yDiff = newCoor[1]-prevNewCoor[1];
                double angleDiff = angle-anglePrev;
                std::string totalImagePath;
                totalImagePath=testDir+allTracksId[i]+"/"+eachImagesInTracks[i][j];
                cv::Mat monocularImage = cv::imread(totalImagePath);
                if(monocularImage.empty())
                {
                    std::cout<<"Image empty"<<std::endl;
                    exit(0);
                }
                std::vector<cv::Mat> imgSet;
                imgSet.push_back(monocularImage.clone());
                std::vector<double> relation = {xDiff, yDiff, angleDiff};
                std::shared_ptr<blindfind::View> newView = std::make_shared<blindfind::View>(imgSet, 0);
                newView->setRelationPrev(relation);
                newView->setGroundTruthInfo(xCoordinate, yCoordinate, angle);
                allViewsInOneTrack.push_back(newView);
                xCoorPrev = xCoordinate;
                yCoorPrev = yCoordinate;
                anglePrev = angle;
            }
        }
        std::cout<<allViewsInOneTrack.size()<<std::endl;
        tMap.addNewTrack(allViewsInOneTrack, 1, false);
        if(i!=0)
        {
            if(i==allTracksId.size()-1) tMap.optimize(true);
            else tMap.optimize(false);
        }
    }
}

bool comp(const std::string& name1, const std::string& name2)
{
    int number1 = String2Number<int>(split(split(name1, "_")[2], ".")[0]);
    int number2 = String2Number<int>(split(split(name2, "_")[2], ".")[0]); 
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
   // std::cout<<imageList[1][52]<<std::endl;
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

