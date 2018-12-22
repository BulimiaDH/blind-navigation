/*
PS4 Client
Joshua Girard
Aug 21, 2015
*/

#include <unistd.h>
#include <iostream>
#include <assert.h>

#include <ps4/PS4Client.h>


// Max size of PS4 camera frame is (1280*800*3 + BMPHeader) < 2^22
#define MAX_FRAME_SIZE 1<<22


PS4Client::PS4Client(){

	newFrames = false;
	using2frames = false;

	connector = new TCPConnector();
	stream = NULL;

	frame1_left = new uint8_t[MAX_FRAME_SIZE];
	frame1_right = new uint8_t[MAX_FRAME_SIZE];
	frame2_left = new uint8_t[MAX_FRAME_SIZE];
	frame2_right = new uint8_t[MAX_FRAME_SIZE];

	framefeatures1_left = framefeatures1_right = framefeatures2_left = framefeatures2_right = nullptr;

	while(!connect(PS4_IMAGEACQUISITION_SERVER_IP_STRING, PS4_IMAGEACQUISITION_NETWORK_PORT)){
		std::cerr << "\nIMAGE ACQUISITION ERROR: Couldn't connect to the other computer! Retrying...\n" << std::endl;
		sleep(1);
	}
}

PS4Client::~PS4Client(){
	disconnect();

	if(framefeatures1_left != nullptr){
		delete[] framefeatures1_left;
		delete[] framefeatures1_right;
	}
	if(framefeatures2_left != nullptr){
		delete[] framefeatures2_left;
		delete[] framefeatures2_right;
	}

	delete[] frame1_left;
	delete[] frame1_right;
	delete[] frame2_left;
	delete[] frame2_right;

	delete connector;
}

bool PS4Client::connect(std::string address, unsigned port){

	stream = connector->connect(address.c_str(), port);
	if(stream == NULL)
		return false;

	return true;

}

void PS4Client::disconnect(){
	if(stream!=NULL)
		delete stream;
}

//TODO: add support for frames not of max resolution size and 15fps
void PS4Client::requestNewFrames(){

	uint8_t modebuffer = 0;

	const char request_byte = '0';
	
	// send an ASCII 0 char to signal we are ready for data
	stream->send((const unsigned char *)&request_byte, 1);

	// get the mode	
	getNetworkData((unsigned char *)&modebuffer, 1);
	if(modebuffer == 0b00101100 || modebuffer == 0b01101100){
		twoframe_mtx.lock();
		using2frames = true;
		twoframe_mtx.unlock();
	}else if(modebuffer == 0b00001100 || modebuffer == 0b01001100){
		twoframe_mtx.lock();
		using2frames = false;
		twoframe_mtx.unlock();
	}else{
		std::cerr << "\n\n\n IMAGE ACQUISITION ERROR: UNSUPPORTED MODE REPORTED BY SERVER! \n\n\n" << std::endl;
		exit(-1);
	}

	// if the client is sending image features
	if(modebuffer & 0b01000000){
		features_mtx.lock();
		usingfeatures = true;
		features_mtx.unlock();

		frameaccess_mtx.lock();


		getNetworkData((unsigned char *)&frame1left_len, sizeof(uint32_t));
		getNetworkData((unsigned char *)&frame1right_len, sizeof(uint32_t));

		if(using2frames){
			getNetworkData((unsigned char *)&frame2left_len, sizeof(uint32_t));
			getNetworkData((unsigned char *)&frame2right_len, sizeof(uint32_t));
		}

		getNetworkData((unsigned char *)&timestamp_frames1and2, sizeof(uint64_t));
		if(using2frames){
			getNetworkData((unsigned char *)&timestamp_frames3and4, sizeof(uint64_t));
		}

		// remove old data
		if(framefeatures1_left != nullptr){
			delete[] framefeatures1_left;
			delete[] framefeatures1_right;
		}
		if(framefeatures2_left != nullptr){
			delete[] framefeatures2_left;
			delete[] framefeatures2_right;
		}

		// allocate new data
		framefeatures1_left = new uint8_t[frame1left_len];
		framefeatures1_right = new uint8_t[frame1right_len];
		if(using2frames){
			framefeatures2_left = new uint8_t[frame2left_len];
			framefeatures2_right = new uint8_t[frame2right_len];
		}

		getNetworkData((unsigned char *)framefeatures1_left, frame1left_len);
		getNetworkData((unsigned char *)framefeatures1_right, frame1right_len);
		if(using2frames){
			getNetworkData((unsigned char *)framefeatures2_left, frame2left_len);
			getNetworkData((unsigned char *)framefeatures2_right, frame2right_len);
		}

		frameaccess_mtx.unlock();

		frameready_mtx.lock();
		newFrames = true;
		frameready_mtx.unlock();

	}else{ // if the client is sending image frames

		features_mtx.lock();
		usingfeatures = false;
		features_mtx.unlock();
	

		//uint32_t frame_len;
		//uint64_t timestamp_frames1and2;
		//uint64_t timestamp_frames3and4; // only used if using 2 cameras

		frameaccess_mtx.lock();

		getNetworkData((unsigned char *)&frame_len, sizeof(uint32_t));

		assert(frame_len <= MAX_FRAME_SIZE);

		getNetworkData((unsigned char *)&timestamp_frames1and2, sizeof(uint64_t));
		if(using2frames){
			getNetworkData((unsigned char *)&timestamp_frames3and4, sizeof(uint64_t));
		}

		//frameaccess_mtx.lock();
		getNetworkData((unsigned char *)frame1_left, frame_len);
		getNetworkData((unsigned char *)frame1_right, frame_len);
		if(using2frames){
			getNetworkData((unsigned char *)frame2_left, frame_len);
			getNetworkData((unsigned char *)frame2_right, frame_len);
		}
		frameaccess_mtx.unlock();

		frameready_mtx.lock();
		newFrames = true;
		frameready_mtx.unlock();
	}
}






void PS4Client::getNetworkData(unsigned char * databuffer, unsigned data_len){
	// get the data
	int len = stream->receive(databuffer, data_len);
	while(len < data_len){
		len += stream->receive(databuffer+len, data_len - len);
	}
}

bool PS4Client::newFramesReady(){
	bool ready;
	frameready_mtx.lock();
	ready = newFrames;
	newFrames = false;
	frameready_mtx.unlock();
	return ready;
}

bool PS4Client::isUsing2Frames(){
	bool twoframes;
	twoframe_mtx.lock();
	twoframes = using2frames;
	twoframe_mtx.unlock();
	return twoframes;
}

bool PS4Client::isUsingFeatures(){
	bool features;
	features_mtx.lock();
	features = usingfeatures;
	features_mtx.unlock();
	return features;
}
