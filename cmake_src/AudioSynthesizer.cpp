/***********************************
* Audio Synthesizer Implementation
* Created by Hongyi Fan
* Nov. 27 2017
*
***********************************/
#include "AudioSynthesizer.h"
#include <unistd.h>
void AudioSynthesizer::goStraight()
{
	festival_say_text("Go straight");
}

void AudioSynthesizer::turnLeftFor90Degrees()
{
	festival_say_text("Turn Left For 90 Degrees");
}


void AudioSynthesizer::turnRightFor90Degrees()
{
	festival_say_text("Turn Right For 90 Degrees");
}

void AudioSynthesizer::reachedDestination(std::string orientation)
{
	festival_say_text("You have reached the destination");
	std::string front_back[] = {"front", "behind"};
	std::string ori[] = {"left", "right"};
	if(orientation.compare(ori[0]) == 0 )
	{
		festival_say_text("Destination is at the left");
	}
	else if(orientation .compare(ori[1])==0)
	{
		festival_say_text("Destination is at the right");
	}
	else if(orientation.compare(front_back[0])==0)
	{
		festival_say_text("Destination is in front of you");
	}
	else if(orientation.compare(front_back[1])==0)
	{
		festival_say_text("Destination is behind you");
	}
	else if(orientation.compare(ori[0]+" "+front_back[0])==0)
	{
		festival_say_text("Destination is on the left front of you");
	}
	else if(orientation.compare(ori[0]+" "+front_back[1])==0)
	{
		festival_say_text("Destination is on the left behind of you");
	}
	else if(orientation.compare(ori[1]+" "+front_back[0])==0)
	{
		festival_say_text("Destination is on the right front of you");
	}
	else if(orientation.compare(ori[1]+" "+front_back[1])==0)
	{
		festival_say_text("Destination is on the right behind of you");
	}
	else
	{
		festival_say_text("Command is not correct");
	}
}

void AudioSynthesizer::holdOn()
{
	festival_say_text("Please hold on for a second");
}

void AudioSynthesizer::relocating()
{
	festival_say_text("I am relocating your place, please hold on");
}

void AudioSynthesizer::turnAroundFor180Degrees()
{
	festival_say_text("Turn around for 180 Degrees");
}

void AudioSynthesizer::respond()
{
	usleep(200000);
	festival_say_text("Yes?");
}

void AudioSynthesizer::takeYouToThe(std::string str)
{
	usleep(200000);
	std::string fullStr = "Take you to the " + str;
	std::cout<<"Reach this place"<<std::endl;
	festival_say_text(fullStr.c_str());
	
}

void AudioSynthesizer::threadMain(std::string lcmchannel)
{
	lcm::LCM lcm;
	if(!lcm.good()) return;
	//std::cout << lcmchannel << std::endl;	
	AudioSynthesizer synthesizer = AudioSynthesizer::GetInstance();
	lcm.subscribe(lcmchannel, &AudioSynthesizer::handleLCMRequests, &synthesizer);
	//std::cout<<"Synthesizer is finished"<<std::endl;
	while(0 == lcm.handle()); //blocks
	
}

void AudioSynthesizer::handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::voiceCommand* msg)
{
	//extern bool returnFlag;
	//returnFlag=1;
//	std::cout << "Get Messages" << std::endl;
	std::string cmd = msg->command;
//	std::cout<<msg->command<<" "<<msg->message<<std::endl;
	
	if (cmd == "straight")
	{
		AudioSynthesizer::GetInstance().goStraight();
		return;
	}
	else if (cmd == "right")
	{
		AudioSynthesizer::GetInstance().turnRightFor90Degrees();
		return;
	}
	else if (cmd == "left")
	{
		AudioSynthesizer::GetInstance().turnLeftFor90Degrees();
		return;
	}
	else if(cmd == "reached")
	{
		extern bool returnFlag;
		returnFlag=1;
		std::string ori = msg->message;
		AudioSynthesizer::GetInstance().reachedDestination(ori);
		
		return;
	}
	else if(cmd == "hold")
	{
		AudioSynthesizer::GetInstance().holdOn();
		return;
	}
	else if(cmd == "relocate")
	{
		AudioSynthesizer::GetInstance().relocating();
		return;
	}
	else if(cmd == "respond")
	{
		AudioSynthesizer::GetInstance().respond();
		return;
	}	
	else if(cmd == "around")
	{
		AudioSynthesizer::GetInstance().turnAroundFor180Degrees();
		return;
	}
	else if(cmd == "take")
	{
		
		std::string loc = msg->message;
		AudioSynthesizer::GetInstance().takeYouToThe(loc);
		return;
	}
}
