/*********************************
 *       PS4 Camera Manager      *
 *   Created by Joshua Girard    *
 *         June 26, 2015         *
 *********************************/

#include <thread>
#include <chrono>
#include <iostream>

#include <assert.h>

#include <ps4/bmp.h>
#include <ps4/PS4CameraManager.h>

using namespace ImageAcquisitionBMP;


PS4CameraManager::PS4CameraManager(){
	_ready = false;
	_fail = false;
	_client = nullptr; 
	_detector = SurfFeatureDetector::create();
	//_surf = new cv::SURF(400,4,2,false); //在这里就初始化了.
}

PS4CameraManager::~PS4CameraManager(){
	_shouldexit = true;
	delete _detector;
	if(_client != nullptr) delete _client;
	//devicedataupdatethread.join();
	stopAll();
}

void PS4CameraManager::threadMain(std::string datachannel, bool shouldUseNetworkedAcquisition){

	PS4CameraManager * cm = &PS4CameraManager::Instance();

	std::thread t([&cm, shouldUseNetworkedAcquisition](){
		if(shouldUseNetworkedAcquisition) cm->initNetworking();
	});

	if(!cm->initAll()){ 
		std::cerr << "PS4 Camera Initialization Failed" << std::endl;
		return;
	}

	if(!cm->startAll()){
		std::cerr << "PS4 Camera Start Failed" << std::endl;
		return;
	}

	t.join();

	//blocks
	cm->handleData();

}

void PS4CameraManager::initNetworking(){

	_client = new PS4Client;
	_client->requestNewFrames();
	
}

void PS4CameraManager::mythread(void* myInstance){
	//PS4CameraManager* myinstance = &PS4CameraManager::Instance();
	static_cast<PS4CameraManager*>(myInstance)->handleData();//.handleData();//object->handleData();
}

void PS4CameraManager::handleData(){

//TODO: multicamera support
// Ideas for identifying cameras: use eeprom to store which position camera

	

	{
		std::lock_guard<std::mutex> lock(_ready_mutex);
		_ready = true;
	}
	_ready_cv.notify_one();
	std::cout << "Starting PS4 Camera Acquisition!!!" << std::endl;

	if(_client != nullptr){
		std::thread t([this](){
			while(!_shouldexit){
				std::cout << "Requesting new frames.." << std::endl;
				_client->requestNewFrames();
			}
		});
		t.detach();
	}

	//std::thread t([this](){
		while(!_shouldexit) 
		{
			PS4EYECam::updateDevices();
			//std::cout<<666<<std::endl;
		}
			//if(!PS4EYECam::updateDevices())
				//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//});
	//std::swap(t,devicedataupdatethread);


	std::cout << "Done" << std::endl;

}



bool PS4CameraManager::blockUntilReady(){
	
	if(_ready) return true;
	if(_fail)  return false;

	std::unique_lock<std::mutex> lock(_ready_mutex);
	_ready_cv.wait(lock, [this]{ return _ready; });
	lock.unlock();

	return !_fail;	

}

float deserialize_float(unsigned char *buffer) { 
    unsigned int f;
    f = ((unsigned int)buffer[0]<<24) | ((unsigned int)buffer[1]<<16) | ((unsigned int)buffer[2]<<8) | buffer[3];
    return *((float *)&f);
}


PS4CameraImageFeatures PS4CameraManager::getLatestFramesFeatures(unsigned device_num){

	PS4CameraImageFeatures frame;

	PS4EYECam::PS4EYERef camera = _devices[device_num];
	
	while(!camera->isNewFrame()){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

	// don't ask why
	camera->check_ff71();


	unsigned frame_len = camera->getWidth()*camera->getHeight()*3; //没有bmp文件头

	uint8_t *frame_left = new uint8_t[frame_len];
	uint8_t *frame_right = new uint8_t[frame_len];

	eyeframe* yuvframe = camera->getLastVideoFramePointer();


	yuyvToRgb(yuvframe->videoLeftFrame, frame_left, camera->getWidth(), camera->getHeight());
    	yuyvToRgb(yuvframe->videoRightFrame, frame_right, camera->getWidth(), camera->getHeight());


	// extract features
	cv::Mat image_left(camera->getHeight(),camera->getWidth(), CV_8UC3, frame_left);
	cv::Mat image_right(camera->getHeight(),camera->getWidth(), CV_8UC3, frame_right);
	cv::Mat mask_left, mask_right;
	_detector->detectAndCompute(image_left,mask_left,frame.keypoints_left,frame.descriptors_left);
	_detector->detectAndCompute(image_right,mask_right,frame.keypoints_right,frame.descriptors_right);
	//(*_surf)(image_left,mask_left,frame.keypoints_left,frame.descriptors_left);
	//(*_surf)(image_right,mask_right,frame.keypoints_right,frame.descriptors_right);


	frame.position = PS4CameraImageFeatures::UNKNOWN;

	return frame;

}
/*
std::vector<PS4CameraImageFeatures> PS4CameraManager::getLatestFrameFeaturesAll(){
	std::vector<PS4CameraImageFeatures> frames;
	frames.reserve(3);
	
	std::thread t([this, &frames]{
		// get frames from local machine
		for(int i=0; i<_devices.size(); i++)
			frames.push_back(this->getLatestFramesFeatures(i));
	});

	if(_client == nullptr){
		t.join();
		return frames;
	}

	// get frames from the remote machine
	while(!_shouldexit && !_client->newFramesReady()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

	// make sure the server is sending features and not frames
	assert(_client->isUsingFeatures());

	PS4CameraImageFeatures frame1, frame2;
*/
	/*
		uint32_t frame1left_len, frame1right_len, frame2left_len, frame2right_len;
		uint8_t *framefeatures1_left, *framefeatures1_right, *framefeatures2_left, *framefeatures2_right;
//--------------------------------------------------------------------------------------------------------------------
		std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
		std::vector<float> descriptors_left, descriptors_right;

		// The physical position of the camera, CENTER_PRIMARY is the left front camera if there is more than one front camera
		enum POSITION { FRONT_PRIMARY=0, FRONT_SECONDARY, RIGHT, LEFT, UNKNOWN };
		POSITION position;

		KeyPoint (float x, float y, float _size, float _angle=-1, float _response=0, int _octave=0, int _class_id=-1)
*/
/*
	_client->frameaccess_mtx.lock();


	assert(_client->frame1left_len%((128+4)*4) == 0);
	assert(_client->frame1right_len%((128+4)*4) == 0);
	assert(_client->frame2left_len%((128+4)*4) == 0);
	assert(_client->frame2right_len%((128+4)*4) == 0);

	uint8_t * data = _client->framefeatures1_left;

	frame1.keypoints_left.reserve(_client->frame1left_len/((128+4)*4));
	frame1.descriptors_left.reserve(128*_client->frame1left_len/((128+4)*4));

	for(int i=0; i<_client->frame1left_len/((128+4)*4); i++){
		frame1.keypoints_left.push_back(cv::KeyPoint(deserialize_float(data), deserialize_float(data+4), deserialize_float(data+8), deserialize_float(data+12)));
		data+=16;
		for(int j=0; i<128; j++){
			frame1.descriptors_left.push_back(deserialize_float(data));
			data+=4;
		}
	}

	data = _client->framefeatures1_right;

	frame1.keypoints_right.reserve(_client->frame1right_len/((128+4)*4));
	frame1.descriptors_right.reserve(128*_client->frame1right_len/((128+4)*4));

	for(int i=0; i<_client->frame1right_len/((128+4)*4); i++){
		frame1.keypoints_right.push_back(cv::KeyPoint(deserialize_float(data), deserialize_float(data+4), deserialize_float(data+8), deserialize_float(data+12)));
		data+=16;
		for(int j=0; i<128; j++){
			frame1.descriptors_right.push_back(deserialize_float(data));
			data+=4;
		}
	}
	

	if(_client->isUsing2Frames()){
		data = _client->framefeatures2_left;

		frame2.keypoints_left.reserve(_client->frame2left_len/((128+4)*4));
		frame2.descriptors_left.reserve(128*_client->frame2left_len/((128+4)*4));

		for(int i=0; i<_client->frame2left_len/((128+4)*4); i++){
			frame2.keypoints_left.push_back(cv::KeyPoint(deserialize_float(data), deserialize_float(data+4), deserialize_float(data+8), deserialize_float(data+12)));
			data+=16;
			for(int j=0; i<128; j++){
				frame2.descriptors_left.push_back(deserialize_float(data));
				data+=4;
			}
		}

		data = _client->framefeatures2_right;

		frame2.keypoints_right.reserve(_client->frame2right_len/((128+4)*4));
		frame2.descriptors_right.reserve(128*_client->frame2right_len/((128+4)*4));

		for(int i=0; i<_client->frame2right_len/((128+4)*4); i++){
			frame2.keypoints_right.push_back(cv::KeyPoint(deserialize_float(data), deserialize_float(data+4), deserialize_float(data+8), deserialize_float(data+12)));
			data+=16;
			for(int j=0; i<128; j++){
				frame2.descriptors_right.push_back(deserialize_float(data));
				data+=4;
			}
		}
	}

	_client->frameaccess_mtx.unlock();

	t.join();

	frame1.position = PS4CameraImageFeatures::UNKNOWN;
	frames.push_back(frame1);

	if(_client->isUsing2Frames()){
		frame2.position = PS4CameraImageFeatures::UNKNOWN;
		frames.push_back(frame2);
	}

	return frames;
}
*/

// returns the latest frames from the primary camera (front)
PS4CameraFrame PS4CameraManager::getLastestFramesPrimary(){

	PS4EYECam::PS4EYERef camera = _devices[0];

	
	while(!camera->isNewFrame()){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

	// don't ask why
	camera->check_ff71();


	unsigned buffer_len = camera->getWidth()*camera->getHeight()*3;

	uint8_t *frame_left = new uint8_t[buffer_len];
	uint8_t *frame_right = new uint8_t[buffer_len];

	PS4CameraFrame frame;
	frame.frame_len = buffer_len;
	frame.leftFrame = std::shared_ptr<uint8_t>(frame_left, std::default_delete<uint8_t[]>());
	frame.rightFrame = std::shared_ptr<uint8_t>(frame_right, std::default_delete<uint8_t[]>());
	frame.width = camera->getWidth();
	frame.height = camera->getHeight();
	frame.type = PS4CameraFrame::BGR;
	frame.position = PS4CameraFrame::FRONT_PRIMARY;

	eyeframe* yuvframe = camera->getLastVideoFramePointer();

	uint8_t *frame_bgr_left = frame_left;
	uint8_t *frame_bgr_right = frame_right;

	yuyvToRgb(yuvframe->videoLeftFrame, frame_bgr_left, camera->getWidth(), camera->getHeight());
    yuyvToRgb(yuvframe->videoRightFrame, frame_bgr_right, camera->getWidth(), camera->getHeight());

	//camera->getLastLeftVideoFrame(frame.leftFrame.get());
	//camera->getLastRightVideoFrame(frame.rightFrame.get());

	return frame;
}

 //returns the latest frames from the primary camera (front) with header embedded
PS4CameraFrame PS4CameraManager::getLastestFramesPrimaryWithHeader(){
	return getLatestFramesWithHeader(0);
}


// returns a vector of all the frames from all cameras
std::vector<PS4CameraFrame> PS4CameraManager::getLatestFramesWithHeaderAll(){
std::cout << "--" << std::endl;

	std::vector<PS4CameraFrame> frames;
	frames.reserve(3);

	// get frames from local machine
	std::cout<<"The device size is: "<<_devices.size()<<std::endl;
	for(int i=0; i<_devices.size(); i++)
	{
		std::cout<<"Start to get frames"<<std::endl;
		frames.push_back(getLatestFramesWithHeader(i));
	//	std::cout<<i+1<<" is finished"<<std::endl;
	}


	if(_client == nullptr) return frames;

	// get frames from the remote machine
	while(!_shouldexit && !_client->newFramesReady()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

	// make sure the server is sending frames and not features
	assert(!_client->isUsingFeatures());

	PS4CameraFrame frame1, frame2;
	
	_client->frameaccess_mtx.lock();

	uint8_t *frame_left1 = new uint8_t[_client->frame_len];
	uint8_t *frame_right1 = new uint8_t[_client->frame_len];
	uint8_t *frame_left2, *frame_right2;

	memcpy(frame_left1, _client->frame1_left, _client->frame_len);
	memcpy(frame_right1, _client->frame1_right, _client->frame_len);

	if(_client->isUsing2Frames()){
		frame_left2 = new uint8_t[_client->frame_len];
		frame_right2 = new uint8_t[_client->frame_len];

		memcpy(frame_left2, _client->frame2_left, _client->frame_len);
		memcpy(frame_right2, _client->frame2_right, _client->frame_len);
	}

	_client->frameaccess_mtx.unlock();

//TODO: fix hardcoded width and height 
	frame1.frame_len = _client->frame_len;
	frame1.leftFrame = std::shared_ptr<uint8_t>(frame_left1, std::default_delete<uint8_t[]>());
	frame1.rightFrame = std::shared_ptr<uint8_t>(frame_right1, std::default_delete<uint8_t[]>());
	frame1.width = 1280;
	frame1.height = 800;
	frame1.type = PS4CameraFrame::BMP;
	frame1.position = PS4CameraFrame::UNKNOWN;

	frames.push_back(frame1);

	if(_client->isUsing2Frames()){

		frame2.frame_len = _client->frame_len;
		frame2.leftFrame = std::shared_ptr<uint8_t>(frame_left2, std::default_delete<uint8_t[]>());
		frame2.rightFrame = std::shared_ptr<uint8_t>(frame_right2, std::default_delete<uint8_t[]>());
		frame2.width = 1280;
		frame2.height = 800;
		frame2.type = PS4CameraFrame::BMP;
		frame2.position = PS4CameraFrame::UNKNOWN;

		frames.push_back(frame2);
	}


	return frames;
}


PS4CameraFrame PS4CameraManager::getLatestFramesWithHeader(unsigned device_num){


	PS4EYECam::PS4EYERef camera = _devices[device_num];

	//std::cout<<"Wait before"<<std::endl;
	while(!camera->isNewFrame()){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
	//std::cout<<"Wait after"<<std::endl;
	// don't ask why
	camera->check_ff71();


	unsigned buffer_len = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + camera->getWidth()*camera->getHeight()*3;

	uint8_t *frame_left = new uint8_t[buffer_len];
	uint8_t *frame_right = new uint8_t[buffer_len];

	writeBMPHeader(frame_left, camera->getWidth(), camera->getHeight()); //写入bmp文件必要的信息，比如通道数，大小等等。
	writeBMPHeader(frame_right, camera->getWidth(), camera->getHeight());

	PS4CameraFrame frame;
	frame.frame_len = buffer_len;
	frame.leftFrame = std::shared_ptr<uint8_t>(frame_left, std::default_delete<uint8_t[]>());
	frame.rightFrame = std::shared_ptr<uint8_t>(frame_right, std::default_delete<uint8_t[]>());
	frame.width = camera->getWidth();
	frame.height = camera->getHeight();
	frame.type = PS4CameraFrame::BMP;

//TODO: fix camera position
	frame.position = PS4CameraFrame::UNKNOWN;

	eyeframe* yuvframe = camera->getLastVideoFramePointer();

	uint8_t *frame_bgr_left = frame_left + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	uint8_t *frame_bgr_right = frame_right + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	yuyvToRgb(yuvframe->videoLeftFrame, frame_bgr_left, camera->getWidth(), camera->getHeight());
    yuyvToRgb(yuvframe->videoRightFrame, frame_bgr_right, camera->getWidth(), camera->getHeight());

	//camera->getLastLeftVideoFrame(frame.leftFrame.get());
	//camera->getLastRightVideoFrame(frame.rightFrame.get());

	return frame;

}

std::vector<PS4EYECam::PS4EYERef> inline PS4CameraManager::getAvailableCameras(){
	return PS4EYECam::getDevices();
}

bool PS4CameraManager::init(PS4EYECam::PS4EYERef camera){

	if(!camera) { std::cerr << "bad camera reference!" << std::endl; return false; }

	camera->firmware_path="firmware.bin";
	camera->firmware_upload();

	//Mode: 0 1280x800 1 640x400 2 320x192
	bool res = camera->init(0, 30); // Mode 0 (1280x800) , 30FPS
	
	if(res == false) { std::cout << "INIT FAILED, WHO CARES?" << std::endl; } // If the camera has already been run once, then init will fail

	_devices.push_back(camera);
	
	return true;

}


bool PS4CameraManager::initAll(){
	_devices = getAvailableCameras();

	if(_devices.size() < 1){ 
		std::cerr << "No cameras detected!" << std::endl; 
		stopAll(); 

		{
			std::lock_guard<std::mutex> lock(_ready_mutex);
			_ready = _fail = true;
		}
		_ready_cv.notify_one(); 
		
		return false; 
	}

	int i=0;
	std::vector<PS4EYECam::PS4EYERef> cameras = getAvailableCameras();
	for(std::vector<PS4EYECam::PS4EYERef>::iterator it = cameras.begin(); it != cameras.end(); ++it, ++i)
		if(!init(*it)) { 
			{
				std::lock_guard<std::mutex> lock(_ready_mutex);
				_ready = _fail = true;
			}
			_ready_cv.notify_one();
			return false; 
		}

	std::cout << i << " cameras initialized!" << std::endl;

	return true;
}

bool PS4CameraManager::start(PS4EYECam::PS4EYERef camera){



	camera->start();
	//camera->set_mirror_sensors(1);
/*
	pause(camera);
	std::cout << "camera paused" << std::endl;
	camera->set_flip_and_mirror_sensors(1);
	std::cout << "fliped" << std::endl;
	resume(camera);*/
	return true;
}

bool PS4CameraManager::startAll(){
	for(std::vector<PS4EYECam::PS4EYERef>::iterator it = _devices.begin(); it != _devices.end(); ++it)
		start(*it);
	return true;
}

void PS4CameraManager::pause(PS4EYECam::PS4EYERef camera){
	camera->stop_sensors_streaming();
}

void PS4CameraManager::pauseAll(){
	for(std::vector<PS4EYECam::PS4EYERef>::iterator it = _devices.begin(); it != _devices.end(); ++it)
		pause(*it);
}

void PS4CameraManager::resume(PS4EYECam::PS4EYERef camera){
	camera->start_sensors_streaming();
}

void PS4CameraManager::resumeAll(){
	for(std::vector<PS4EYECam::PS4EYERef>::iterator it = _devices.begin(); it != _devices.end(); ++it)
		resume(*it);
}

void PS4CameraManager::stop(PS4EYECam::PS4EYERef camera){
	camera->shutdown();
}

void PS4CameraManager::stopAll(){
	for(std::vector<PS4EYECam::PS4EYERef>::iterator it = _devices.begin(); it != _devices.end(); ++it)
		stop(*it);
}



//---------------------------------------------------------------------------------------
//----------------------BEGIN UGLY YUV TO BGR CODE---------------------------------------


//from https://social.msdn.microsoft.com/forums/windowsdesktop/en-us/1071301e-74a2-4de4-be72-81c34604cde9/program-to-translate-yuyv-to-rgbrgb modified yuyv order
/*--------------------------------------------------------*\
 |    yuv2rgb                                               |
 \*--------------------------------------------------------*/
void PS4CameraManager::yuv2rgb(int y, int u, int v, char *r, char *g, char *b)
{
    int r1, g1, b1;
    int c = y-16, d = u - 128, e = v - 128;

    r1 = (298 * c           + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d           + 128) >> 8;

    // Even with proper conversion, some values still need clipping.

    if (r1 > 255) r1 = 255;
    if (g1 > 255) g1 = 255;
    if (b1 > 255) b1 = 255;
    if (r1 < 0) r1 = 0;
    if (g1 < 0) g1 = 0;
    if (b1 < 0) b1 = 0;

    *r = r1 ;
    *g = g1 ;
    *b = b1 ;
}

/*--------------------------------------------------------*\
 |    yuyvToRgb                                             |
 \*--------------------------------------------------------*/
void PS4CameraManager::yuyvToRgb(uint8_t *in,uint8_t *out, int size_x,int size_y)
{
    int i;
    unsigned int *pixel_16=(unsigned int*)in;;     // for YUYV
    unsigned char *pixel_24=out;    // for RGB
    int y, u, v, y2;
    char r, g, b;


    for (i=0; i< (size_x*size_y/2) ; i++)
    {
        // read YuYv from newBuffer (2 pixels) and build RGBRGB in pBuffer

     //   v  = ((*pixel_16 & 0x000000ff));
       // y  = ((*pixel_16 & 0x0000ff00)>>8);
       // u  = ((*pixel_16 & 0x00ff0000)>>16);
       // y2 = ((*pixel_16 & 0xff000000)>>24);

        y2  = ((*pixel_16 & 0x000000ff));
        u  = ((*pixel_16 & 0x0000ff00)>>8);
        y  = ((*pixel_16 & 0x00ff0000)>>16);
        v = ((*pixel_16 & 0xff000000)>>24);

    yuv2rgb(y2, u, v, &r, &g, &b);            // 2nd pixel




        *pixel_24++ = b;
        *pixel_24++ = g;
        *pixel_24++ = r;

     yuv2rgb(y, u, v, &r, &g, &b);            // 1st pixel



        *pixel_24++ = b;
        *pixel_24++ = g;
        *pixel_24++ = r;



        pixel_16++;
    }
}


//-----------------------BMP Header---------------------------


void PS4CameraManager::writeBMPHeader(uint8_t *buffer, int width, int height){

	if(buffer==NULL)return; //TODO: exception

	size_t bmp_data_len = width * height * 3;	


	BITMAPFILEHEADER bf;
  BITMAPINFOHEADER bi;    
    
  bf.bfType = 0x4d42;
  bf.bfSize = bmp_data_len + sizeof(bf) + sizeof(bi);
  bf.bfReserved1 = 0;
  bf.bfReserved2 = 0;
  bf.bfOffBits = 54;
    
  bi.biSize = 40;
  bi.biWidth = width;
  bi.biHeight = -height;
  bi.biPlanes = 1;
  bi.biBitCount = 24;
  bi.biCompression = 0;
  bi.biSizeImage = 0;
  bi.biXPelsPerMeter = 0xb12;
  bi.biYPelsPerMeter = 0xb12;
  bi.biClrUsed = 0;
  bi.biClrImportant = 0;

	memcpy(buffer, &bf, sizeof(bf));
	memcpy(buffer + sizeof(bf), &bi, sizeof(bi));
}
