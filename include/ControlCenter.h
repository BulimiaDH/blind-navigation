#ifndef __CONTROL_CENTER__
#define __CONTROL_CENTER__

#include<AudioSynthesizer.h>
#include<navigator.h>
#include<staticMap.h>
#include<imageMatcher.h>
#include<ps4/PS4CameraManager.h>
#include<destinationMsg.hpp>
#include<HapticBelt.h>

class ControlCenter
{
    public:
       // static bool returnFlag;
        static void threadMain(std::string mapPath, std::string destinationListPath, std::string destinationIndexPath, 
        std::string matchingDataBase, string vocabularyDatabase, std::string imageList, std::string destinationChannel, std::string featureName, bool whetherNew); //在这个里面启动那些其他的thread, 比如navigator, PS4CameraManager这些线程。都是单例，没关系 
        static ControlCenter& getInstance()
        {
            static ControlCenter _center;
            return _center;
        }

        void handleLCMRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::destinationMsg *msg);
        /*This handler will call the iterateControl function every 2 seconds**/

        /**考虑，在handleLCMRequest里面也waiting这个信息。 ThreadMain 里面也waiting这个信息，每次wait到以后都会重新触发这个handleLCMRequest()**/

    private:

        void iterateControl(); //每两秒调用一次iterateControl, 调用函数里面会调用PS4CameraManager获得图片，然后ImageMatcher匹配图片获得图片Index，然后进行封装，成locationBundle,
        
        imageMatcher* _imageMatcher;
        std::string _mapPath; /*Path to the map*/
        std::string _destinationPath;  /*Path to the destination list file, including coordinates and orientation and destinationID*/
        std::string _destinationIndexPath;  /*Path to the destination index*/
        std::string _matchingDatabase;  /*Database path*/
        std::string _imageList;  /* */
        std::string _featureName;
        bool _whetherNew;
        int _destinationID;
      //  static ControlCenter _center;



};

#endif
