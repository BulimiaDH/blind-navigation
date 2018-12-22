/*********************************
 *       PS4 Camera Manager      *
 *   Created by Joshua Girard    *
 *         June 26, 2015         *
 *********************************/


// This is a singleton class (for obvious reasons) 


#ifndef _PS4CameraManager_h_
#define _PS4CameraManager_h_

#include <vector>
using namespace std;
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>

#include<opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include<opencv2/core/core.hpp>

#include "PS4Client.h"
#include "ps4eye.h"

using namespace ps4eye;
using namespace cv;
using namespace xfeatures2d;

struct PS4CameraFrame{

	public:

		//PS4CameraFrame(): 

		std::shared_ptr<uint8_t> leftFrame, rightFrame;
		unsigned frame_len;
		unsigned width, height;
		
		// The image format type, default is BMP
		enum TYPE { YUV=0, RGB, BGR, BMP, JPEG };
		TYPE type;

		// The physical position of the camera, CENTER_PRIMARY is the left front camera if there is more than one front camera
		enum POSITION { FRONT_PRIMARY=0, FRONT_SECONDARY, RIGHT, LEFT, UNKNOWN };
		POSITION position;


};


struct PS4CameraImageFeatures{

	public: 
		std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
		std::vector<cv::Mat> descriptors_left, descriptors_right;

		// The physical position of the camera, CENTER_PRIMARY is the left front camera if there is more than one front camera
		enum POSITION { FRONT_PRIMARY=0, FRONT_SECONDARY, RIGHT, LEFT, UNKNOWN };
		POSITION position;

};

class PS4CameraManager{

public:

	// Call this to get a handle for the object instead of the constructor
	static PS4CameraManager& Instance(){
		static PS4CameraManager instance;
		return instance;
	}
	
    // invoked by the System Manager to start the module, use false for the last parameter to use this module without networking
    static void threadMain(std::string datachannel, bool shouldUseNetworkedAcquisition=true);
	static void mythread(void*);

	bool initAll();

	void initNetworking();

//	bool isInitialized(unsigned camera_number);
//	bool allInitialized();


	bool startAll();


	void pauseAll();
	void resumeAll();

	void stopAll();

	void handleData();


//-------------------------------------------------------------------
//---------FUNCTIONS TO BE CALLED FROM OTHER MODULES-----------------

	// check if the acquisition system is ready
	bool ready() { return _ready; }
	
	// Will block until the acquisiton system is ready or until failure, returns true if ready or false if failure
	bool blockUntilReady();

	// returns the latest frames from the primary camera (front)
	PS4CameraFrame getLastestFramesPrimary();

	// returns the latest frames from the primary camera (front) with header embedded
	PS4CameraFrame getLastestFramesPrimaryWithHeader();

	// returns a vector of all the frames from all cameras
	std::vector<PS4CameraFrame> getLatestFramesWithHeaderAll();

	// returns a vector of all the frames features from all cameras
	std::vector<PS4CameraImageFeatures> getLatestFrameFeaturesAll();


	PS4CameraImageFeatures getLatestFramesFeatures(unsigned device_num);

//-------------------------------------------------------------------
//-------------------------------------------------------------------

private:

	// Private Con/De structors b/c singleton 
	PS4CameraManager();
	~PS4CameraManager();

	// We don't want these constructors
	PS4CameraManager(PS4CameraManager const&) = delete; 
	void operator=(PS4CameraManager const&) = delete;

	Ptr<SurfFeatureDetector> _detector;
	//cv::SURF* _surf;

	PS4Client* _client;  //This is defined in PS4Client.h header file.

	static PS4CameraManager* objectInstance;

	std::vector<PS4EYECam::PS4EYERef> _devices;

	std::vector<PS4EYECam::PS4EYERef> getAvailableCameras();
	
	PS4CameraFrame getLatestFramesWithHeader(unsigned device_num);

	volatile bool _shouldexit = false;
	//std::thread devicedataupdatethread;

	bool _ready;  // Whether the camera is ready
	bool _fail;
	std::mutex _ready_mutex;  // Mutex for the _ready boolean.
	std::condition_variable _ready_cv;  //Condition Variable for CV

	bool init(PS4EYECam::PS4EYERef camera);
	bool start(PS4EYECam::PS4EYERef camera);
	void pause(PS4EYECam::PS4EYERef camera);
	void resume(PS4EYECam::PS4EYERef camera);
	void stop(PS4EYECam::PS4EYERef camera);


	//Used for converting YUV to BGR
	void yuv2rgb(int y, int u, int v, char *r, char *g, char *b);
	void yuyvToRgb(uint8_t *in,uint8_t *out, int size_x,int size_y);

	void writeBMPHeader(uint8_t *buffer, int width, int height);


};
#endif // _PS4CameraManager_h_
