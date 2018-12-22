/*********************************
 *       Haptic Belt Header      *
 *   Created by Joshua Girard    *
 *         Jan 12, 2015          *
 *********************************/

#ifndef __HapticBelt_h_
#define __HapticBelt_h_

#include <lcm/lcm-cpp.hpp>

#include <SerialDevice.h>

#include <haptic_belt_command_t.hpp>


class HapticBelt{


	public:

		// invoked by SystemManger on a new thread
		static void threadMain(std::string lcmchannel);

		HapticBelt();
		~HapticBelt();

		// returns 0 if successful, otherwise -1
		int initDevice();
		void sendCommand(std::string cmd);

		// begin handling LCM requests
		void handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::haptic_belt_command_t* msg);


	private:

		Serial * _hapticbelt;


};

#endif
