/***************************
  Navigator Class Implementation
  Created by Hongyi Fan
  Dec. 04. 2017
  *************************/
#ifndef __NAVIGATOR_H_
#define __NAVIGATOR_H_

#include <iostream>
#include <stdio.h>
#include <staticMap.h>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <cmath>
#include <climits>
#include <algorithm>
#include <iterator>
#include <haptic_belt_command_t.hpp>
#include <voiceCommand.hpp>
#include <locationBundle.hpp>

using namespace std;
class navigator {
public:
	navigator();
	navigator(string filePath, string destinationPath, string lcmChannel ,string hapticChannel, string voiceChannel, int mode);
	~navigator();
	static navigator nav();
	/**Only for test**/
	void setDestination(int destinationNode,int currentOrientation, int currentNode, int destinationOrientation);

	/**Tris to make it a singleton**/
	
	static navigator& getInstance();//{return navi;}
	
	/**This function is only for test**/
	void reset()
	{
		this->_initialized=0;
	}

	void getShortestPath();
	vector<pair<int, int> > getPath(){return _path;};
	string generateHapticCommand();
	string generateVoiceCommand(string hapticCommand);	

	static void threadMain(string filePath, string destinationPath, string lcmChannel, string voiceChannel, string hapticChannel, int mode);
	void handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::locationBundle* msg);
	
	void navigateInit();
	void navigate();

	

	vector<int> Dijkstra(int** adjacentMatrix, int src);

	//getDataForTest; Will be removed from final version (Only for test)
	void getDataForTest();
	int printSolution(int dist[], int n);
private:
	string generateCommand0(int xCurrent, int yCurrent, int xNext, int yNext, int oNext);
	string generateCommand90(int xCurrent, int yCurrent, int xNext, int yNext, int oNext);
	string generateCommand180(int xCurrent, int yCurrent, int xNext, int yNext, int oNext);
	string generateCommand270(int xCurrent, int yCurrent, int xNext, int yNext, int oNext);
	
	bool _initialized;
	
	staticMap* _map;
	vector<pair<int, int> > _path; //The first entity is the index of the node, the second entity is the orientation
	int _currentNode;
	int _currentOrientation;
	pair<int, int> _currentCoordinate;
	int _destinationNode;
	int _destinationOrientation;
	pair<int, int> _destinationCoordinate;
	vector<int> _destinationList;
	int _positionInThePath;	
	lcm::LCM _lcmInstance;
	string _hapticChannel;
	string _voiceChannel;
	string _navigatorChannel;	
	//If mode == 0: Real-time mode: for real use
	//If mode == 1: Test mode: for LCM test
	int _mode; 
};
//navigator navigator::nav;
#endif
