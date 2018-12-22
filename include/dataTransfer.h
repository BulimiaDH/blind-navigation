/************************
  Data Transferer
  Created by Hongyi Fan
  Dec. 12. 2017
*************************/
#ifndef __DATATRANSFERER_H_
#define __DTTATRANSFERER_H_

#include <iostream>
#include <string>
#include <thread>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>

#include <localizationMsg.hpp>
#include <destinationMsg.hpp>
#include <locationBundle.hpp>
using namespace std;
class DataTransferer
{
public:
	//Constructor and Deconstructor
	DataTransferer(){};
	DataTransferer(string locationBundleChannel)
	{
		_locationBundleChannel = locationBundleChannel;
		_initialized = 0;
		_flagLocalization = 0;
		_flagDestination = 0;
		_initialized = 0;
	};
	~DataTransferer(){};	


	static void threadMain(string locationBundleChannel, string localizationChannel, string destinationChannel);
	void handleLocalizationLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::localizationMsg* msg);
	void handleDestinationLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::destinationMsg* msg);
	
	/**Only for integration test**/
	void handleLocalizationLCMRequestsTest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::localizationMsg* msg);
	void handleDestinationLCMRequestsTest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::destinationMsg* msg);


private:
	string _locationBundleChannel;
	int _imageIndex1;
	int _imageIndex2;
	int _destinationIndex;
	bool _flagLocalization;
	bool _flagDestination;
	bool _initialized;

	/**Data Type that is only for test**/
	int _xCurrent;
	int _yCurrent;
	int _oCurrent;

};

#endif
