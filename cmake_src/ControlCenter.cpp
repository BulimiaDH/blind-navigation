#include<ControlCenter.h>
#include<thread>
#include<chrono>
bool returnFlag=0;
void ControlCenter::threadMain(std::string mapPath, std::string destinationPath, 
std::string destinationIndexPath, std::string matchingDatabase, std::string vocabularyDatabase, std::string imageList, 
std::string destinationChannel, std::string featureName, bool whetherNew=true)
{
    ControlCenter* center = &ControlCenter::getInstance();
    lcm::LCM lcm;
    if(!lcm.good()) 
    {
        std::cout<<"The lcm data for control center is not generated well"<<std::endl;
        return;
    }

    center->_mapPath = mapPath;  //"/home/blindfind/Documents/Blindfind3/data/map_new.txt";
    center->_destinationPath = destinationPath; 
    center->_destinationIndexPath = destinationIndexPath;
    center->_matchingDatabase = matchingDatabase;
    center->_imageList = imageList;
    center->_featureName = featureName;
    center->_whetherNew = whetherNew; 

    center->_imageMatcher = new imageMatcher(imageList, matchingDatabase, vocabularyDatabase, featureName, "vocabularyTree", false);
    center->_imageMatcher->setFeatureLayer("pool3");
    
    /*Define three kinds of data channel*/
    std::string locationBundleChannel = "LOCATION_BUNDLE_CHANNEL";
    std::string voiceCommandChannel = "VOICE_COMMAND_CHANNEL";
    std::string hapticBeltChannel = "HAPTIC_BELT_CHANNEL";


    /*Open Four threads*/
    std::cout<<"Begin the AudioSynthesizer thread"<<std::endl;
    std::thread voiceSynThread(AudioSynthesizer::threadMain, voiceCommandChannel);
    std::cout<<"Begin the haptic belt thread"<<std::endl;
    //std::thread hapticBeltThread(HapticBelt::threadMain, hapticBeltChannel);
    std::cout<<"Begin the navigator thread"<<std::endl;
    std::thread navigatorThread(navigator::threadMain, 
        mapPath, destinationPath, locationBundleChannel, voiceCommandChannel, hapticBeltChannel, 0);
    std::cout<<"Begin the camera thread"<<std::endl;
    std::thread cameraThread(PS4CameraManager::threadMain, std::string("test"), false);
    lcm.subscribe(destinationChannel, &ControlCenter::handleLCMRequest, center);
   // std::cout<<"The Control Center is initialized"<<std::endl;
    while(0 == lcm.handle());
}


void ControlCenter::handleLCMRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::destinationMsg *msg)
{
    std::string locationBundleChannel = "LOCATION_BUNDLE_CHANNEL";
    this->_destinationID = msg->destinationIndex;   
    while(true)
    {
        if(returnFlag==1)
        {
            std::cout<<"Detect returnFlag set to zero, will return to initial state"<<std::endl;
            lcm::LCM lcm;
	        if(!lcm.good()) return;
            LCMDataType::locationBundle bundleMsg;
            bundleMsg.msgType=1; //reset the navigator
            lcm.publish(locationBundleChannel, &bundleMsg);
            returnFlag=0;
            break;
        }
        this->iterateControl();
      
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}


void ControlCenter::iterateControl()
{

    std::string locationBundleChannel = "LOCATION_BUNDLE_CHANNEL";
    PS4CameraFrame Frame = PS4CameraManager::Instance().getLastestFramesPrimary();
    cv::Mat image_left(Frame.height, Frame.width, CV_8UC3, Frame.leftFrame.get());
	cv::Mat image_right(Frame.height, Frame.width, CV_8UC3, Frame.rightFrame.get());
    std::vector<std::pair<int, int> > result;
    if(this->_featureName=="CNN")
    {
        cv::flip(image_left, image_left, 1);
        cv::flip(image_right, image_right, 1);
        result = this->_imageMatcher->matching_images(image_left, image_right, 0.05);
    }
    else if(this->_featureName=="ORB")
    {
        cv::Mat image_left_gray;
        cv::Mat image_right_gray;
        cv::cvtColor(image_left, image_left_gray, CV_BGR2GRAY);
        cv::cvtColor(image_right, image_right_gray, CV_BGR2GRAY);
        cv::flip(image_left_gray, image_left_gray, 1); 
        cv::flip(image_right_gray, image_right_gray,1);
        result = this->_imageMatcher->matching_images(image_left_gray, image_right_gray, 0.1);
    }
    
    if(result.size()==0)
    {
        return;
    }

    lcm::LCM lcm;
	if(!lcm.good()) return;

    LCMDataType::locationBundle bundleMsg;

    bundleMsg.msgType=0;
    bundleMsg.imageIndex1 = result[0].first;
    bundleMsg.imageIndex2 = result[0].second;
    bundleMsg.destination = this->_destinationID;
    lcm.publish(locationBundleChannel, &bundleMsg);

}

