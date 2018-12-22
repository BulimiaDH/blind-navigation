#ifndef __H_TIMERRECEIVER_
#define __H_TIMERRECEIVER_

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <ps4/PS4CameraManager.h>
#include <trigger.hpp>


using namespace std;
class timerReceiver{
public: 
	timerReceiver();
	~timerReceiver();
	void handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::trigger* msg);
	static void threadMain(string LCMChannel);
	static int _value;
};

#endif
