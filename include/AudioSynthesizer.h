/*****************************
*   Audio Synthesizer Class
*   Created by Hongyi Fan    
*            Nov. 27 2017 
*****************************/
#ifndef __AudioSynthesizer_h_
#define __AudioSynthesizer_h_


#include <vector>
#include <string>
#include <festival/festival.h>
#include <iostream>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <voiceCommand.hpp>

class AudioSynthesizer {
public:
	static AudioSynthesizer& GetInstance()
	{
		static AudioSynthesizer instance;
		return instance;
	}

	static void threadMain(std::string lcmchannel);
	void handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::voiceCommand* msg);
	//Commands:
	void goStraight();
	void turnLeftFor90Degrees();
	void turnRightFor90Degrees();
	void turnAroundFor180Degrees();
	void reachedDestination(std::string orientation);
	void holdOn();	
	void relocating();
	void respond();
	void takeYouToThe(std::string str);


private:
	AudioSynthesizer()
	{
		festival_initialize(1,210000);	
	};
	~AudioSynthesizer(){};
	AudioSynthesizer(const AudioSynthesizer &p){};
	AudioSynthesizer& operator=(AudioSynthesizer &p){};
	
};

#endif
