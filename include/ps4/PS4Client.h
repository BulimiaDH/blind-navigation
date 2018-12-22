/*
PS4 Client Header
Joshua Girard
Aug 21, 2015
*/

// Note: this class is a helper for PS4ClientManager


#ifndef _PS4Client_h_
#define _PS4Client_h_

#include <thread>
#include <mutex>

#include <stdint.h>

#include "tcp/ps4_packet_def.h"
#include "tcp/ps4_port_def.h" //PS4_IMAGEACQUISITION_NETWORK_PORT

#include "tcp/tcpconnector.h"

#define PS4_IMAGEACQUISITION_SERVER_IP_STRING "10.9.92.13"

class PS4Client{
	friend class PS4CameraManager;
	
	// for testing
	friend class PS4ClientTest;

	private:
		PS4Client();
		~PS4Client();

		bool connect(std::string address, unsigned port);
		void disconnect();

		// check if the server is sending 2 frames or 1, by default it is false until the first frame is received
		bool isUsing2Frames();
		void requestNewFrames();
		// check if new frames are ready and resets ready state
		bool newFramesReady();
		bool isUsingFeatures();


//--------------------------------------PRIVATE-------------------------------------

		void getNetworkData(unsigned char * databuffer, unsigned data_len);

		std::mutex frameready_mtx, twoframe_mtx, features_mtx;
		bool newFrames, using2frames, usingfeatures;
		
		std::mutex frameaccess_mtx;
		uint32_t frame_len;
		uint64_t timestamp_frames1and2, timestamp_frames3and4;
		uint8_t *frame1_left, *frame1_right, *frame2_left, *frame2_right;
		
		uint32_t frame1left_len, frame1right_len, frame2left_len, frame2right_len;
		uint8_t *framefeatures1_left, *framefeatures1_right, *framefeatures2_left, *framefeatures2_right;

		TCPConnector* connector;
		TCPStream* stream;

		// shorthand of frame1_left, etc
		//std::mutex f1l_mtx, f1r_mtx, f2l_mtx, f2r_mtx;
};

#endif
