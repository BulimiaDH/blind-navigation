/***************************
  trackMap Class Implementation
  Created by Han Deng
  Aug. 16. 2018
  *************************/
#include "trackMap.h"
#include<iostream>
float getAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c)
{
	float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
	if (theta > CV_PI)
		theta -= 2 * CV_PI;
	if (theta < -CV_PI)
		theta += 2 * CV_PI;
 
	theta = theta * 180.0 / CV_PI;
	return theta;
}

std::vector<float> getRelation(double x1, double y1, double x2, double y2, double angle1, double angle2)
{
    std::vector<float> relation;
    Eigen::Vector2d worldCoor;
    worldCoor<<x2, y2;
    Eigen::Vector2d prevWorldCoor;
    prevWorldCoor<<x1, y1;
    Eigen::Matrix2d transMat;
    transMat<<cos(angle1), sin(angle1), -sin(angle1), cos(angle1);
    Eigen::Vector2d newCoor = transMat*worldCoor;
    Eigen::Vector2d prevNewCoor = transMat*prevWorldCoor;
    relation.push_back(newCoor[0]-prevNewCoor[0]);
    relation.push_back(newCoor[1]-prevNewCoor[1]);
    relation.push_back(180*(angle2-angle1)/M_PI);
    return relation;
}

std::vector<int> findLargest(std::vector<float> scores)
{
	float largest = 0;
	std::vector<int> allIds;
	for (size_t k = 0; k < scores.size(); k++)
	{
		if (scores[k] > largest)
			largest = scores[k];
	}

	for (size_t k = 0; k < scores.size(); k++)
	{
		if (scores[k] == largest)
        {
			allIds.push_back(k);
            break;
        }
	}
	return allIds;
}
std::string format(int number, int length)
{
	std::string temp = Number2String<int>(number);
	int addedLength = length - temp.length();
	temp.insert(0, addedLength, '0');
	return temp;
}

track::track(vector<std::shared_ptr< blindfind::View> > allViewsInOneTrack, int mode)
{
    
    this->_allViewsInOneTrack = allViewsInOneTrack;
    this->_nodeNum = allViewsInOneTrack.size();
    this->changePoseStructures(mode);
     
}

/*For test only*/
track::track(const std::vector<std::vector<double> > oneTrackRelation, int numNodesOneTrack)
{
    this->_nodeNum = numNodesOneTrack;
    this->addRelations(oneTrackRelation, numNodesOneTrack);
}

track::~track()
{
    //std::cout<<"Destructor is used"<<std::endl;
    /*
    for(size_t i = 0; i<_poseNodes.size(); i++)
    {
        delete _poseNodes[i];
    }
    for(size_t i = 0; i<_pose.size(); i++)
    {
        delete _pose[i];
    }
    for(size_t i = 0; i<_poseFactors.size(); i++)
    {
        delete _poseFactors[i];
    }
    */
}

void track::addRelations(const std::vector<std::vector<double> > oneTrackRelation, int numNodesOneTrack)
{
    //Add relations between one track inside to the attributes of the class.
    for(int k =0; k<numNodesOneTrack; k++)
    {
        if(k==0)
        {
            this->_poseNodes.push_back(new Pose2d_Node());
            Pose2d* initialPose = new Pose2d(0.,0.,0.);
            this->_pose.push_back(initialPose);
            this->_initFactor = new Pose2d_Factor(_poseNodes[k], *(_pose[k]), noise);
        }
        else
        {
            this->_poseNodes.push_back(new Pose2d_Node());
        }
    }

    for(int j=0; j<oneTrackRelation.size(); j++)
    {
        int id1 = oneTrackRelation[j][0];
        int id2 = oneTrackRelation[j][1];
        double xDiff = oneTrackRelation[j][2];
        double yDiff = oneTrackRelation[j][3];
        double angleDiff = oneTrackRelation[j][4];
        this->_pose.push_back(new Pose2d(xDiff, yDiff, angleDiff));
        this->_poseFactors.push_back(new Pose2d_Pose2d_Factor(_poseNodes[id1], _poseNodes[id2], *(_pose[j+1]), noise));
    }
}

void track::changePoseStructures(int mode)
{
    if(mode==0)
    {
        Eigen::Matrix3d coordinate;
        coordinate<<1.,0.,0.,0.,1.,0.,0.,0.,1.;
        for(size_t i = 0; i<_allViewsInOneTrack.size(); i++)
        {
            if(i==0)
            {
                Pose2d* initialPose = new Pose2d(0.,0.,0.);
                this->_pose.push_back(initialPose);
            }
            else
            {
                cv::Mat RotMat = (*_allViewsInOneTrack[i]).getR();
                cv::Mat TransMat = (*_allViewsInOneTrack[i]).getT();
                cv::Mat copiedRotMat;
                cv::Mat copiedTransMat;
                RotMat.convertTo(copiedRotMat, CV_64F);
                TransMat.convertTo(copiedTransMat, CV_64F);
                Eigen::Matrix3d eigenRotMat;
                Eigen::Vector3d eigenTransVec;
                for(size_t row=0; row<RotMat.rows; row++)
                {
                    for(size_t col=0; col<RotMat.cols; col++)
                    {
                        eigenRotMat(row, col) = copiedRotMat.at<double>(row,col);
                    }
                }
                for(size_t row=0; row<RotMat.rows; row++)
                {
                    eigenTransVec(row) = copiedTransMat.at<double>(row,0);
                }

                Eigen::Matrix3d invRotMat = eigenRotMat.inverse();
                Eigen::Matrix3d newCoordinate = invRotMat*coordinate;
                Eigen::Vector3d priorToCurrentTransVec = invRotMat*eigenTransVec;
                Eigen::Vector3d worldTransVec = newCoordinate*priorToCurrentTransVec;
                double newXInPrev_X = newCoordinate(2,2);
                double newXInPrev_Y = -1*newCoordinate(0,2);
                double xPrev_X = coordinate(2,2);
                double xPrev_Y = -1*coordinate(0,2);
                double angle = atan2(xPrev_Y, xPrev_X)-atan2(newXInPrev_Y, newXInPrev_X);
                Pose2d* newPose = new Pose2d(worldTransVec(2), -1*worldTransVec(0),angle);
                this->_pose.push_back(newPose);
                coordinate = newCoordinate;
            }
        }
    }
    else if(mode==1)
    {
        for(size_t i = 0; i<_allViewsInOneTrack.size(); i++)
        {
            if(i==0)
            {
                std::vector<double> relationPrev = this->_allViewsInOneTrack[i]->gerRelationPrev();
                Pose2d* initialPose = new Pose2d(0,0,0);
                //Pose2d* initialPose = new Pose2d(relationPrev[0], relationPrev[1], relationPrev[2]);
                 this->_pose.push_back(initialPose);
            }
            else
            {
                std::vector<double> relationPrev = this->_allViewsInOneTrack[i]->gerRelationPrev();
                Pose2d* newPose = new Pose2d(relationPrev[0], relationPrev[1], relationPrev[2]);
                this->_pose.push_back(newPose);
            }
        }
    }

    for(int j=0; j<this->_pose.size(); j++)
    {
        this->_poseNodes.push_back(new Pose2d_Node());
        if(j==0)
        {
            this->_initFactor = new Pose2d_Factor(_poseNodes[j], *(_pose[j]), noise);
        } 
        else
        {
            this->_poseFactors.push_back(new Pose2d_Pose2d_Factor(_poseNodes[j-1], _poseNodes[j], *(_pose[j]), noise));
        }
    }
}

trackMap::trackMap(std::string allImagesDir)
{
    
    std::string mapPath = "/home/blindfind/Documents/Blindfind3/data/map_new.txt";
    std::string destinationPath = "/home/blindfind/Documents/Blindfind3/data/destination.txt";
    std::string imageList = "/home/blindfind/Documents/Blindfind3/data/data_list_new.txt"      ;
    std::string matchingDatabase = "/home/blindfind/Documents/Blindfind3/data/ORBdatabase.yml.gz";
    std::string vocabularyPath = "/home/blindfind/Documents/Blindfind3/testDataset/CNN_Vocabulary.yml.gz";
    this->_allImagesDir = allImagesDir;
    
    this->_slam = new Slam();
    this->_trackId=0;
    this->_imageId=0;
    this->_distThreshold=-1;
    this->_angleThreshold=-1;

    this->_imageMatcher = std::make_shared<imageMatcher>(imageList, matchingDatabase, vocabularyPath, "CNN", "vocabularyTree", true); 
    this->_imageMatcher->setFeatureLayer("pool5"); //Set which layer's feature the CNN will extract from the image. It requires correct database.

    this->_estimator = rotationEstimation();
}

trackMap::~trackMap()
{
    delete _slam;
}

int trackMap::intensity(Mat img1, Mat img2, int dx, int dy, float fx)
{
    assert(dx > 0 && dy > 0 && fx > 0);

	const int w = img1.cols;
	const int h = img1.rows;

	vector<pair<float, float>> intensities;
	float xx, yy;
	float x0, y0, x1, y1;
	int flag = 0;
	for(float angle = -40; angle <= 40; angle++) {
		float intensity = 0;
		int cnt = 0;
		Mat trans(h, w, CV_8UC1, Scalar::all(0));
		Mat mask(h, w, CV_8UC1, Scalar::all(0));
		for(int x = 0; x < w; x += dx) {
			for(int y = 0; y < h; y += dy) {
				xx = (x - w/2.0f)/fx;
				yy = (y - h/2.0f)/fx;
				yy = yy/(-sin(angle / 180 * M_PI) * xx + cos(angle / 180 * M_PI));
				xx = (cos(angle / 180 * M_PI) * xx + sin(angle / 180 * M_PI)) / (-sin(angle / 180 * M_PI) * xx + cos(angle / 180 * M_PI));
				xx = xx * fx + w / 2.0f;
				yy = yy * fx + h / 2.0f;
				
				x0 = floor(xx);
				x1 = ceil(xx);
				y0 = floor(yy);
				y1 = ceil(yy);

				if(xx >= 0 && xx < w && yy >= 0 && yy < h) {
					float p00 = img1.at<unsigned char>(y0, x0);
				       	float p01 = img1.at<unsigned char>(y1, x0);
					float p10 = img1.at<unsigned char>(y0, x1);
					float p11 = img1.at<unsigned char>(y1, x1);
					float p0 = p00 + (p01 - p00) * (yy - y0);
					float p1 = p10 + (p11 - p10) * (yy - y0);
					unsigned char p = p0 + (p1 - p0) * (xx - x0) + 0.5;
				
					trans.at<unsigned char>(y, x) = p;
					mask.at<unsigned char>(y, x) = 1;
				}
			}
		}
		for(int x = 0; x < w; x++) {
			for(int y = 0; y < h; y++) {
				if(mask.at<unsigned char>(y, x) == 1) {
					intensity += abs(trans.at<unsigned char>(y, x) - img2.at<unsigned char>(y, x));
					cnt++;
				}
			}
		}
		intensities.push_back(make_pair(intensity / cnt, angle));
	}
	sort(intensities.begin(), intensities.end());
	return intensities[0].second;
}


std::vector<info> trackMap::dynamicProgramming(std::vector<std::pair<std::vector<node>, int> > allCandidates)
{
    std::vector<info> currentStateSet;
    for(size_t k=0; k<allCandidates.size(); k++)
    {
        if(k==0)
        {
            std::pair<std::vector<node>, int> firstCandidate = allCandidates[allCandidates.size()-1];
            for(size_t j=0; j<firstCandidate.first.size(); j++)
            {

                if(!firstCandidate.first[j].hasRelation) continue;
                info oneState;
                oneState.route.push_back(std::pair<int, int>(firstCandidate.first[j].imageId, firstCandidate.second));
                oneState.score = firstCandidate.first[j].score;
                oneState.layerAndId = std::pair<int, int>(allCandidates.size()-1-k, j);
                currentStateSet.push_back(oneState);
            }
        }
        else
        {
            std::vector<info> newStateSet;
            std::pair<std::vector<node>, int> nextCandidate = allCandidates[allCandidates.size()-1-k];
            std::vector<float> transferRateSet;
            bool allNodeNoRelation=true;
            for(size_t j=0; j<nextCandidate.first.size(); j++)
            {
                transferRateSet.clear();
                if(allCandidates.size()-1-k!=0)
                {
                    if(!nextCandidate.first[j].hasRelation)
                    {
                        continue;
                    }
                }
                allNodeNoRelation=false;
                for(size_t i=0; i<currentStateSet.size(); i++)
                {
                    std::pair<int, int> layerAndId = currentStateSet[i].layerAndId;
                    bool found=false;
                    for(size_t h=0; h<nextCandidate.first[j].relation.size(); h++)
                    {
                        if(nextCandidate.first[j].relation[h].first == layerAndId.first && nextCandidate.first[j].relation[h].second == layerAndId.second)
                        {
                            found=true;
                            break;
                        }
                    }
                    if(!found)
                    {
                        transferRateSet.push_back(0);
                        continue;
                    }
                    double newScore = nextCandidate.first[j].score+currentStateSet[i].score;
                    transferRateSet.push_back(newScore);
                }
                std::vector<int> allIds = findLargest(transferRateSet);
                if(transferRateSet[allIds[0]]!=0)
                {
                    newStateSet.push_back(currentStateSet[allIds[0]]);
                    newStateSet[newStateSet.size()-1].route.push_back(std::pair<int, int>(nextCandidate.first[j].imageId, nextCandidate.second));
                    newStateSet[newStateSet.size()-1].score = transferRateSet[allIds[0]];
                    newStateSet[newStateSet.size()-1].layerAndId = std::pair<int, int>(allCandidates.size()-1-k, j);
                }
            }
            if(allNodeNoRelation)
            {
                std::cout<<"There is something wrong, no node is found having a relation"<<std::endl;
                break;
            }
            currentStateSet = newStateSet;
        }
    }
    return currentStateSet;
}

std::vector<std::vector<std::pair<std::vector<node>, int> > > trackMap::spatialConstraint(std::vector<std::pair<std::vector<node>, int> >& allCandidates, int trackId, double distanceThresh, double angleThresh)
{
    std::vector<std::vector<std::pair<std::vector<node>, int> > >  allCandidatesSet;
    int continuousLayerNum=0;
    std::cout<<"All candidates size: "<<allCandidates.size()<<std::endl;
    for(size_t i=0; i<allCandidates.size()-1; i++)
    {   
        std::cout<<"i=: "<<i<<std::endl;
        int prevNodeIdQueryTrack = allCandidates[i].second;
        int nextNodeIdQueryTrack = allCandidates[i+1].second;
        Pose2d_Node* prevNode = this->_allTracks[trackId].getOneNode(prevNodeIdQueryTrack);
        Pose2d_Node* nextNode = this->_allTracks[trackId].getOneNode(nextNodeIdQueryTrack);
        std::cout<<"Previous Node id: "<<prevNodeIdQueryTrack<<" Next Node id: "<<nextNodeIdQueryTrack<<std::endl;
        double xPrev = prevNode->value().x();
        double yPrev = prevNode->value().y();
        double anglePrev = prevNode->value().t();
        double xNext = nextNode->value().x();
        double yNext = nextNode->value().y();
        double angleNext = nextNode->value().t();

        std::vector<float> relationTruth = getRelation(xPrev, yPrev, xNext, yNext, anglePrev, angleNext);
        float angleTruth = relationTruth[2];
        double distance = pow(pow(relationTruth[0], 2)+pow(relationTruth[1], 2), 0.5);
        
        angleTruth = angleTruth>0?angleTruth:360+angleTruth;
        std:cout<<"XPrev: "<<xPrev<<" YPrev: "<<yPrev<<" XNext: "<<xNext<<" YNext: "<<yNext<<std::endl;
        std::cout<<"Distance:: "<<distance<<" Angle:: "<<angleTruth<<" Node id: "<<allCandidates[i].second<<std::endl;
        
        bool hasAtLeastOneRelation=false;
        for(size_t j=0; j<allCandidates[i].first.size(); j++)
        {
            if(continuousLayerNum>0)
            {
                if(!(allCandidates[i].first[j].hasRelation)) continue;
            }
            int prevImgId = allCandidates[i].first[j].imageId;
            std::pair<int, int> prevTrackAndImage = this->getTrackIdFromImageId(prevImgId);
            Pose2d_Node* prevCandidateNode = this->_allTracks[prevTrackAndImage.first].getOneNode(prevTrackAndImage.second);
            double xPrevCandidate = prevCandidateNode->value().x();
            double yPrevCandidate = prevCandidateNode->value().y();
            double prevAngle = prevCandidateNode->value().t();

            for(size_t k=0; k<allCandidates[i+1].first.size(); k++)
            {
                int nextImgId = allCandidates[i+1].first[k].imageId;
                std::pair<int, int> nextTrackAndImage = this->getTrackIdFromImageId(nextImgId);
                Pose2d_Node* nextCandidate = this->_allTracks[nextTrackAndImage.first].getOneNode(nextTrackAndImage.second);
                double xNextCandidate = nextCandidate->value().x();
                double yNextCandidate = nextCandidate->value().y();
                double nextAngle = nextCandidate->value().t();
                std::vector<float> relationCandidate = getRelation(xPrevCandidate, yPrevCandidate, xNextCandidate, yNextCandidate, prevAngle, nextAngle);
                float angleCandidate = relationCandidate[2];
                angleCandidate = angleCandidate>0?angleCandidate:360+angleCandidate;

                double upperBound=0;
                double lowerBound=0;

                bool flag=false;
                if(angleTruth<=angleThresh)
                { 
                    lowerBound = 360+(angleTruth-angleThresh);
                    upperBound = angleTruth+angleThresh;
                    if(relationCandidate[0]>relationTruth[0]-distanceThresh&&relationCandidate[0]<relationTruth[0]
                    +distanceThresh&&relationCandidate[1]>relationTruth[1]-distanceThresh&&relationCandidate[1]<relationTruth[1]
                    +distanceThresh&& angleCandidate>angleTruth-angleThresh && angleCandidate<angleTruth+angleThresh)
                        flag=true;
                }
                else if(angleTruth>=(360-angleThresh))
                {
                    lowerBound = angleTruth-angleThresh;
                    upperBound = angleTruth+angleThresh-360;
                       if(relationCandidate[0]>relationTruth[0]-distanceThresh&&relationCandidate[0]<relationTruth[0]
                       +distanceThresh&&relationCandidate[1]>relationTruth[1]-distanceThresh&&relationCandidate[1]<relationTruth[1]
                       +distanceThresh&& angleCandidate>angleTruth-angleThresh && angleCandidate<angleTruth+angleThresh)
                            flag=true;
                }
                else
                {
                       if(relationCandidate[0]>relationTruth[0]-distanceThresh&&relationCandidate[0]<relationTruth[0]
                       +distanceThresh&&relationCandidate[1]>relationTruth[1]-distanceThresh&&relationCandidate[1]<relationTruth[1]
                       +distanceThresh && angleCandidate>angleTruth-angleThresh && angleCandidate<angleTruth+angleThresh)
                            flag=true;
                }
                if(flag)
                {
                        hasAtLeastOneRelation=true;
                        allCandidates[i+1].first[k].hasRelation=true;
                        allCandidates[i].first[j].relation.push_back(std::pair<int, int>(continuousLayerNum+1, k));
                        flag=false;
                }
            }
        }        
        if(!hasAtLeastOneRelation)
        {
                std::cout<<"Continues Layer Number: "<<continuousLayerNum<<std::endl;
                /*This threshold is very important, influence the accuracy**/
                if(continuousLayerNum>=7)
                {   
                    std::vector<std::pair<std::vector<node>, int> > oneCandidateSet;
                    for(size_t m=i-continuousLayerNum; m<=i; m++)
                    {
                        oneCandidateSet.push_back(allCandidates[m]);
                    }
                    allCandidatesSet.push_back(oneCandidateSet);
                    continuousLayerNum=0;
                    continue;
                }
                else
                {
                    continuousLayerNum=0;
                    continue;
                }
        }

        /**This exists because when the iteration comes to the end, it may not be stopped. 
         * we need this to judge whetehr the continuous layer is larger than a threshold 
         * to add this last relation into the map.
         * */
        if(i==allCandidates.size()-2)
        {
            if(continuousLayerNum>=7)
            {   
                std::vector<std::pair<std::vector<node>, int> > oneCandidateSet;
                for(size_t m=i-continuousLayerNum; m<=i; m++)
                {
                    oneCandidateSet.push_back(allCandidates[m]);
                }
                allCandidatesSet.push_back(oneCandidateSet);
                continuousLayerNum=0;
                continue;
            }
        }
        continuousLayerNum++;
    }
    return allCandidatesSet;
}

/*Only for test*/
void trackMap::addNewTrack(const std::vector<std::shared_ptr< blindfind::View> > allViewsInOneTrack,int mode, bool newOne=false )
{

    track aTrack(allViewsInOneTrack, mode);
    this->_allTracks.push_back(aTrack);
    int viewNum = allViewsInOneTrack.size()*allViewsInOneTrack[0]->getImgs().size();
    this->_imageIdToTrackId.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(this->_imageId, this->_imageId+viewNum), this->_trackId)); //2 here means there are 2 images in one View
    this->addSingleTrackToMap(_trackId);
    std::cout<<"New track, ID: "<<this->_trackId<<" range of images: "<<this->_imageId<<" to "<<this->_imageId+viewNum<<std::endl;
    
    if(this->_trackId==0)
    {
        std::cout<<"Now we add features to the database"<<std::endl;
        this->addFeaturesToDB(_trackId);
    }
    else
    {
        std::cout<<"a new track, we need to relate it to previous tracks"<<std::endl;
        std::vector<std::vector<double> > result = this->findConstraints(_trackId);
        if(result.size()!=0)
        {
            std::cout<<"Find constraints, now we add constraints to the SLAM"<<std::endl;
            this->addConstraints(result);
            std::cout<<"Constraitns adding is finished, now we add features to the database"<<std::endl;
            
        }
        this->addFeaturesToDB(this->_trackId);
    }
    
    this->_imageId+=viewNum;
    this->_trackId+=1;
}

void trackMap::addFeaturesToDB(int trackId) //send a Message to a database class? May need mutex
{
    //Currently, we use image Matcher to add new features to the existing database, feature is extracted from View class, a encapsulated class of other informations.
    if(trackId!=0)
    {
        this->_imageMatcher->addFeatures();
    }
    else   
    {
        for(size_t k =0; k<this->_allTracks[trackId].getNodeNumber(); k++)
        {
            std::cout<<"Adding images: "<<k<<"/"<<this->_allTracks[trackId].getNodeNumber()<<std::endl;
            this->_imageMatcher->addFeatures(this->_allTracks[trackId].getOneView(k)->getImgs()[0]);
        }
    }

    /**Currently, no need to save the feature to the database**/
   // this->_imageMatcher->saveDatabase();
}


std::pair<int, int> trackMap::getTrackIdFromImageId(int imgId)
{
    int trackPrevId=0;
    std::map<std::pair<int, int>, int>::iterator iter;
    for(iter=this->_imageIdToTrackId.begin(); iter!=this->_imageIdToTrackId.end(); iter++)
    {
        if(imgId>=iter->first.first&&imgId<iter->first.second)
        {
            int imagePrevTrack = imgId-(iter->first.first);
            return std::pair<int, int>(trackPrevId,imagePrevTrack);
        }
        trackPrevId+=1;
    }
    return std::pair<int, int>(-1,-1);
}

/*Currently the constraints only provides same location and angle. needs later on to be modified**/
std::vector<std::vector<double> > trackMap::findConstraints(const int trackId)
{
    std::vector<std::vector<double> > myResult;
    std::vector<std::pair<std::vector<node>, int> > allCandidates;
    for(size_t k = 0; k<this->_allTracks[trackId].getNodeNumber(); k++)
    {
        std::vector<cv::Mat> Imgs = this->_allTracks[trackId].getOneView(k)->getImgs();
        cv::Mat leftImageGrey = Imgs[0].clone();
        if(this->_allTracks[trackId].getOneView(k)->isStereo())
        {
            cv::Mat rightImageGrey = Imgs[1].clone();
            std::vector<std::pair<int, int> > result = this->_imageMatcher->matching_images(leftImageGrey, rightImageGrey, 0.25f);
            if(result.size()==0)
            {
                continue;
            }
        }
        else
        {
            std::cout<<"Matching "<<k<<"/"<<this->_allTracks[trackId].getNodeNumber()<<std::endl;
            std::vector<pair<int, float> > result = this->_imageMatcher->matching_one_image(leftImageGrey, 0.01); 
            if(result.size()==0)
            {
                continue;
            }
            else
            {
                /*Now we try to convert the score from result set to the possibility*/
                double totalScore=0;
                for(size_t i=0; i<result.size(); i++)
                    totalScore+=result[i].second;
                for(size_t i=0; i<result.size(); i++)
                    result[i].second = result[i].second/totalScore;
                std::vector<node> oneCandidate;
                for(size_t m=0; m<(result.size()<20?result.size():20); m++)
                {
                    node oneNode;
                    oneNode.imageId = result[m].first;
                    oneNode.score = result[m].second;
                    oneNode.hasRelation = false;
                    //oneNode.hasRelationWithLayerBehind = false;
                    oneCandidate.push_back(oneNode);
                }
                allCandidates.push_back(std::pair<std::vector<node>, int>(oneCandidate,k));
            }
        }  
    } 
    std::vector<std::vector<std::pair<std::vector<node>, int> > > allCandidatesSet = this->spatialConstraint(allCandidates, this->_trackId, this->_distThreshold, this->_angleThreshold);
    for(size_t l=0; l<allCandidatesSet.size(); l++)
    {    
        std::vector<info> optimizedResults = this->dynamicProgramming(allCandidatesSet[l]);
        std::sort(optimizedResults.begin(), optimizedResults.end(), [](const info& a, const info& b)->bool{return a.score>b.score;});
        std::vector<std::pair<int, int> > optimizedRoute = optimizedResults[0].route;
        for(size_t s=0; s<optimizedRoute.size(); s++)
        {
            std::pair<int, int> prevTrackAndImage = this->getTrackIdFromImageId(optimizedRoute[s].first);


            /**
             * You can choose to use either ground truth angle or estimated angle;
             * Now I am trying to use groundtruth location and angle to give to the iSAM, it is just a test
             **/
            std::shared_ptr<blindfind::View> queryView = this->_allTracks[trackId].getOneView(optimizedRoute[s].second);
            std::shared_ptr<blindfind::View> matchedView = this->_allTracks[prevTrackAndImage.first].getOneView(prevTrackAndImage.second);

            double xCoorQuery = queryView->getGtInfo().gtXCoor;
            double yCoorQuery = queryView->getGtInfo().gtYCoor;
            double angleQuery = queryView->getGtInfo().gtAngle;

            double xCoorMatched = matchedView->getGtInfo().gtXCoor;
            double yCoorMatched = matchedView->getGtInfo().gtYCoor;
            double angleMatched = matchedView->getGtInfo().gtAngle;

            std::vector<float> allRelation=getRelation(xCoorMatched, yCoorMatched, xCoorQuery, yCoorQuery, angleMatched, angleQuery);
            std::cout<<"Relation: Matched track and Image: "<<prevTrackAndImage.first<<" "<<prevTrackAndImage.second<<" query track and image: "<<trackId<<" "<<optimizedRoute[s].second<<" Relation from prev to current "
            <<allRelation[0]<<" "<<allRelation[1]<<" "<<allRelation[2]<<std::endl;


            //This is the rotation estimation method
            int angle = this->_estimator.estimate(this->_allTracks[prevTrackAndImage.first].getOneView(prevTrackAndImage.second)->getImgs()[0], 
                this->_allTracks[trackId].getOneView(optimizedRoute[s].second)->getImgs()[0], 386.14);
            
            //This is the intensity based method to estimate the angle
            
            //int angle = this->intensity(this->_allTracks[prevTrackAndImage.first].getOneView(prevTrackAndImage.second)->getImgs()[0],
            //    this->_allTracks[trackId].getOneView(optimizedRoute[s].second)->getImgs()[0], 1, 1, 386.14);
            
            if(angle>100||angle<-100){   
                angle=0;          
            }
            std::vector<double>  relation={prevTrackAndImage.first, trackId, prevTrackAndImage.second, optimizedRoute[s].second, 0., 0., -((double)angle/180*M_PI)};
            myResult.push_back(relation);
        }
    }
    std::cout<<"My result size "<<myResult.size()<<std::endl;
    return myResult;   
}  

void trackMap::addSingleTrackToMap(int id)
{
    vector<Pose2d_Node* > trackNodes = this->_allTracks[id].getNodes();
    Pose2d_Factor* initFactor = this->_allTracks[id].getInitFactor();
    vector<Pose2d_Pose2d_Factor* > poseFactors = this->_allTracks[id].getPoseFactors();
    int num = this->_allTracks[id].getNodeNumber();
    for(int j = 0; j<num; j++)
    {
        this->_slam->add_node(trackNodes[j]);
        if(j==0) 
        {
            this->_slam->add_factor(initFactor);   
        }
        else
        {
            this->_slam->add_factor(poseFactors[j-1]);
        }
    }
    this->_anchorNodesOfTracks.push_back(new Anchor2d_Node(this->_slam));
    this->_slam->add_node(this->_anchorNodesOfTracks[_trackId]);

    /**This is only a test for adding a pose between the anchor node and the origin**/
    if(id==0)
    {
        std::vector<double> relationPrev = this->_allTracks[id].getOneView(0)->gerRelationPrev();
        Pose2d* anchorPose = new Pose2d(relationPrev[0], relationPrev[1], relationPrev[2]);
        Pose2d_Factor* anchorFactor = new Pose2d_Factor(this->_anchorNodesOfTracks[_trackId], *anchorPose, this->noise);
        this->_slam->add_factor(anchorFactor);   
    }
    
}

void trackMap::mergeTracks(const std::vector<std::vector<double> >& result)
{
    for(size_t i = 0; i<result.size(); i++)
    {
        int trackId1 = (int)result[i][0];
        int trackId2 = (int)result[i][1];
        int imageId1 = (int)result[i][2];
        int imageId2 = (int)result[i][3];
        double x_diff = result[i][4];
        double y_diff = result[i][5];
        double angle_diff = result[i][6];
        std::pair<int, int> trackPair(trackId1, trackId2);
        std::map<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> >::iterator iter = this->_measureBetweenTracks.find(trackPair);
        Anchor2d_Node* anchor0 = this->_anchorNodesOfTracks[trackId1];
        Anchor2d_Node* anchor1 = this->_anchorNodesOfTracks[trackId2];
        Pose2d_Node* track1Node = this->_allTracks[trackId1].getOneNode(imageId1);
        Pose2d_Node* track2Node = this->_allTracks[trackId2].getOneNode(imageId2);
        Pose2d* measure = new Pose2d(x_diff, y_diff, angle_diff);
        if(iter==this->_measureBetweenTracks.cend())
        {
            std::vector<Pose2d_Pose2d_Factor*> measures;
            measures.push_back(new Pose2d_Pose2d_Factor(track1Node, track2Node, *measure, noise, anchor0, anchor1));
            this->_measureBetweenTracks.insert(std::pair<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> >(std::pair<int, int>(trackId1,  trackId2), measures));
        }
        else
        {
            iter->second.push_back(new Pose2d_Pose2d_Factor(track1Node, track2Node, *measure, noise, anchor0, anchor1));
        }
    }
}

void trackMap::optimize(bool whetherStoreMap=false)
{
    std::cout<<"Now we begin to optimize"<<std::endl;
    std::map<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> >::iterator iter;
    for(iter = this->_measureBetweenTracks.begin(); iter!=this->_measureBetweenTracks.end(); iter++)
    {
        for(size_t k = 0; k<iter->second.size(); k++)
        {
            this->_slam->add_factor(iter->second[k]);
        }
    }
    if(this->_measureBetweenTracks.size()!=0)
    {
        this->_slam->batch_optimization();
        for(int id = 0 ; id<_trackId; id++)
        {
            int&& nodeNum = this->_allTracks[id].getNodeNumber();
            for(int k = 0; k<nodeNum; k++)
            {
                Pose2d_Node* currentNodeInTrack =  this->_allTracks[id].getOneNode(k);
                Anchor2d_Node* anchorNodeThisTrack = this->_anchorNodesOfTracks[id];
                currentNodeInTrack->init(anchorNodeThisTrack->value().oplus(currentNodeInTrack->value()));
            }
        }
    }
    if(whetherStoreMap)
    {
        this->_slam->save("./result.graph"); //Later on needs some other places to implement this code.
    }
    /* Refresh the _measureBetweenTracks*/
    this->_measureBetweenTracks.clear();
}

/*Only for test*/
void trackMap::addConstraints(const std::vector<std::vector<double> >& result)
{
    std::cout<<"Result size "<<result.size()<<std::endl;
    for(size_t i = 0; i<result.size(); i++)
    {
        std::cout<<i<<std::endl;
        int trackId1 = (int)result[i][0];
        int trackId2 = (int)result[i][1];
        int imageId1 = (int)result[i][2];
        int imageId2 = (int)result[i][3];
        double x_diff = result[i][4];
        double y_diff = result[i][5];
        double angle_diff = result[i][6];
        std::cout<<trackId1<<" "<<trackId2<<" "<<imageId1<<" "<<imageId2<<" "<<x_diff<<" "<<y_diff<<" "<<angle_diff<<std::endl;
        std::pair<int, int> trackPair(trackId1, trackId2);
        std::map<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> >::iterator iter = this->_measureBetweenTracks.find(trackPair);

        Anchor2d_Node* anchor0 = this->_anchorNodesOfTracks[trackId1];
        Anchor2d_Node* anchor1 = this->_anchorNodesOfTracks[trackId2];
        Pose2d_Node* track1Node = this->_allTracks[trackId1].getOneNode(imageId1);
        Pose2d_Node* track2Node = this->_allTracks[trackId2].getOneNode(imageId2);

        /**This code is used to test whether matched images are at the same location(Within a threshold)**/
        double x1 = this->_allTracks[trackId1].getOneView(imageId1)->getGtInfo().gtXCoor;
        double x2 = this->_allTracks[trackId2].getOneView(imageId2)->getGtInfo().gtXCoor;
        double y1 = this->_allTracks[trackId1].getOneView(imageId1)->getGtInfo().gtYCoor;
        double y2 = this->_allTracks[trackId2].getOneView(imageId2)->getGtInfo().gtYCoor;
        double distance = pow(pow(x1-x2, 2)+pow(y1-y2, 2), 0.5);
        std::cout<<"Distance between matched images and query images: "<<distance<<std::endl;
        
        /**This code is only used to test the average distance**/
        /**Should be deleted later**/
        this->_matchedImgNumber+=1;
        this->_totalDistance+=distance;

        /**Also, we store matched images and query image to check**/
        //cv::Mat image1 = this->_allTracks[trackId1].getOneView(imageId1)->getImgs()[0];
        //cv::Mat image2 = this->_allTracks[trackId2].getOneView(imageId2)->getImgs()[0];
        //cv::imshow("Query Image", image1);
        //cv::imshow("Matched Image", image2);
        //cv::waitKey(0); 
        //cv::imwrite("./matchedImages/"+Number2String<int>(trackId1)+"_"+Number2String<int>(imageId1)+".jpg", image1);
        //cv::imwrite("./matchedImages/"+Number2String<int>(trackId2)+"_"+Number2String<int>(imageId2)+".jpg", image2);
        	
        Pose2d* measure = new Pose2d(x_diff, y_diff, angle_diff);
        if(iter==this->_measureBetweenTracks.end())
        {
           
            std::vector<Pose2d_Pose2d_Factor*> measures;
            measures.push_back(new Pose2d_Pose2d_Factor(track1Node, track2Node, *measure, noise, anchor0, anchor1));
            this->_measureBetweenTracks.insert(std::pair<std::pair<int, int>, std::vector<Pose2d_Pose2d_Factor*> >(std::pair<int, int>(trackId1,  trackId2), measures));
        }
        else
        {
            iter->second.push_back(new Pose2d_Pose2d_Factor(track1Node, track2Node, *measure, noise, anchor0, anchor1));
        }
    }
    
    /**
     * This code is only used to test the average distance
     * Should be deleted later
    **/
    std::cout<<"Average Distance: "<<this->_totalDistance/this->_matchedImgNumber<<std::endl;
}

