/***********************
	Timer Trigger 
	Created By Hongyi Fan
	Dec. 11. 2017
***********************/
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <trigger.hpp>

using namespace std;
class timerTrigger
{
public:
	timerTrigger(){};
	timerTrigger(int time){_time = time;};
	~timerTrigger(){};
	static void threadMain(string timerChannel, int time);
	int _time;
};
