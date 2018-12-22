/*********************************
 *          Haptic Belt          *
 *   Created by Joshua Girard    *
 *         Jan 12, 2015          *
 *********************************/

#include <chrono>
#include <thread>

#include <iostream>

#include "HapticBelt.h"


void HapticBelt::threadMain(std::string lcmchannel){

	lcm::LCM lcm;

	if(!lcm.good()) return;
	
	
	HapticBelt hb;

	// Keep trying to connect to the device until success	
	int fd = hb.initDevice();
	while(fd == -1){
		std::cout << "Retrying haptic belt initialization..." << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		fd = hb.initDevice();
	}

	lcm.subscribe(lcmchannel, &HapticBelt::handleLCMRequests, &hb);
	
	while(0 == lcm.handle()); // blocks

}

HapticBelt::HapticBelt(){
	_hapticbelt = new Serial();
	
}

HapticBelt::~HapticBelt(){
	delete _hapticbelt;
}


int HapticBelt::initDevice(){

	int fd = _hapticbelt->initPort();
	
	return fd == -1 ? -1 : 0;

}

void HapticBelt::sendCommand(std::string cmd)
{
	_hapticbelt->sendChar(cmd.at(0)); // for now just use the first char of the string as the command
}


void HapticBelt::handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::haptic_belt_command_t* msg){

	std::string cmd = msg->command;
//	std::cout << "Haptic Belt: The command is " << cmd << std::endl;
	//TODO: parse command

	std::cout << "Got Haptic Belt Command: " << cmd.at(0) << std::endl;

	// send the command to the belt
	_hapticbelt->sendChar(cmd.at(0)); // for now just use the first char of the string as the command
	
}


