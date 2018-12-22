#include<thread>
#include<chrono>
#include<iostream>
#include<string>
#include<AudioRecognizer.h>
#include<AudioSynthesizer.h>
#include<dataTransferer.h>
#include<HapticBelt.h>
#include<navigator.h>
#include<map>
#include<ControlCenter.h>
//#include<staticMap.h>

void readDestinationIndex(const std::string& destinationIndex, std::map<std::string, int>& destIndexFile);
//void navigate(int xCurrent, int yCurrent, int oCurrent);

int main()
{
    /*
    std::string destMsgChannel = "DESTINATION_MESSAGE_CHANNEL";
    std::string localMsgChannel = "LOCALIZATION_MESSAGE_CHANNEL";
    std::string localBundleChannel = "LOCATION_BUNDLE_CHANNEL";
    std::string triggerChannel = "TIMER_TRIGGER_CHANNEL";
    std::string voiceCmdChannel = "VOICE_COMMAND_CHANNEL";
    std::string hapticBeltChannel = "HAPTIC_BELT_COMMAND_CHANNEL";
    std::string coorMap = "coorMap.txt";
    std::string dest = "destination.txt";
    std::string destinationIndex = "dstIndex.txt";
    std::thread transferThread(DataTransferer::threadMain, localBundleChannel, localMsgChannel, destMsgChannel);
    //Mode 1 hear(not 0) means we don't need to genereate voice command, this need to be changed in the soruce code. Very easy.
    std::thread navigatorThread(navigator::threadMain, coorMap, dest, localBundleChannel, voiceCmdChannel, hapticBeltChannel, 1);
    std::thread audioSynThread(AudioSynthesizer::threadMain, voiceCmdChannel);
    std::thread hapticThread(HapticBelt::threadMain, hapticBeltChannel);
    */

    std::string mapPath = "/home/blindfind/Documents/Blindfind3/data/map_new.txt";
    std::string destinationPath = "/home/blindfind/Documents/Blindfind3/data/destination.txt";
    std::string destinationIndexPath = "/home/blindfind/Documents/Blindfind3/data/destinationIndex.txt";
    std::string matchingDatabase = "/home/blindfind/Documents/Blindfind3/data/CNNdatabase_BH.yml.gz";
    std::string imageList = "/home/blindfind/Documents/Blindfind3/data/data_list_new.txt";
    std::string destinationChannel = "DESTINATION_CHANNEL";
    std::string voiceCommandChannel = "VOICE_COMMAND_CHANNEL";

    lcm::LCM lcm;

    if(!lcm.good())
    {
        std::cout<<"Data channel is not generated well"<<std::endl;
    }

    std::map<std::string, int> destinationIndexFile;
    readDestinationIndex(destinationIndexPath, destinationIndexFile);
    
   // std::map<std::string, int>::iterator iter;
  //  std::cout<<"The size of the destination Index file is "<<destinationIndexFile.size()<<std::endl;
    
   // for(iter = destinationIndexFile.begin(); iter!=destinationIndexFile.end(); iter++)
   // {
   //     std::cout<<iter->first<<" "<<iter->second<<std::endl;
   // }
    
    std::thread controlCenterThread(ControlCenter::threadMain, mapPath, 
        destinationPath, destinationIndexPath, matchingDatabase, imageList, destinationChannel, "CNN");
   // std::cout<<"Wait 5 seconds to wait for control center thread to finish"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    AudioRecognizer* recognizer = new AudioRecognizer("./en-us/en-us","./en-us/cmudict-en-us.dict", "./keyphrase.list","./blindfind_audio_menu.gram");
    std::cout<<"Recognizer initialization done"<<std::endl;
    /**Now the Audio recognizer should begin to recognize key words**/
    while(1)
    {
        std::string keyphrase = recognizer->recognize_keyphrase();
        std::cout << "keyphrase recognized: " << keyphrase << std::endl;
        int score;
	    std::string command = recognizer->recognize_command(&score);
	    if (score > -4500)
	    {
		    std::cout << "Command recognized" << command;
	    }
	    else
	    {
		    std::cout << "Command not recognized, please say again" << std::endl;
            continue;
	    }

        LCMDataType::voiceCommand commandMsg;
		commandMsg.command = "take";
		int index = command.find_last_of(" ");
		std::string substring = command.substr(index + 1);
		commandMsg.message = substring;
		lcm.publish(voiceCommandChannel, &commandMsg);	
		int destIndex = destinationIndexFile[substring];
		LCMDataType::destinationMsg destMsg;
		destMsg.destinationIndex = destIndex;
		destMsg.reset = 0;	
		lcm.publish(destinationChannel, &destMsg);
        controlCenterThread.join();
    }
    return 0;
}


void readDestinationIndex(const std::string& destinationIndex, std::map<std::string, int>& destIndexFile)
{
    std::cout<<destinationIndex<<std::endl;
    std::ifstream destIndex(destinationIndex);
    if(!destIndex.is_open())
    {
        std::cout<<"Cannot open the file"<<std::endl;
        exit(-1);
    }
    char oneWord[200];
    char idChar[200];
    int id;
    while(!destIndex.eof())
    {
        destIndex>>oneWord;
        std::stringstream stream;
        destIndex>>idChar;
        stream<<idChar;
        stream>>id;
        destIndexFile.insert(std::make_pair(oneWord, id));
    }
}
/*
void navigate(int xCurrent, int yCurrent, int oCurrent)
{
    std::string localMsgChannel = "LOCALIZATION_MESSAGE_CHANNEL";
    int init_x_current = xCurrent;
    int init_y_current = yCurrent;
    int init_o_current = oCurrent;
    LCMDataType::localizationMsg localMsg;
    
    localMsg.reset=0;
    localMsg.xCurrent = init_x_current;
    localMsg.yCurrent = init_y_current;
    localMsg.oCurrent = init_o_current;
    lcm::LCM lcm;
    if(!lcm.good())
    {
        std::cout<<"In the main thread, the lcm data is not generated well, please speak again"<<std::endl;
    }

    lcm.publish(localMsgChannel, &localMsg);
    std::cout<<"wait for two seconds"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    while(true)
    {
        std::cout<<"----------------INSTRUCTION---------------------"<<std::endl;
        std::cout<<"Input w for go ahead on the current direction"<<std::endl;
        std::cout<<"Input a for turn left from the current direction"<<std::endl;
        std::cout<<"Input s for turn around from the current direction"<<std::endl;
        std::cout<<"Input d for turn right from teh current direction"<<std::endl;
        std::cout<<"-------------------------------------------------"<<std::endl;
        char command;
        std::cin>>command;
        switch(command)
        {
            case 'w':
                std::cout<<"Go ahead from the current direction"<<std::endl;
                switch(localMsg.oCurrent)
                {
                    case 0:
                        localMsg.xCurrent+=1;
                        break;
                    case 90:
                        localMsg.yCurrent+=1;
                        break;
                    case 180:
                        localMsg.xCurrent-=1;
                        break;
                    case 270:
                        localMsg.yCurrent-=1;
                        break;
                }
                break;
            case 'a':
                std::cout<<"Turn left from the current direction"<<std::endl;
                switch(localMsg.oCurrent)
                {
                    case 0:
                        localMsg.oCurrent=270;
                        break;
                    case 90:
                        localMsg.oCurrent=0;
                        break;
                    case 180:
                        localMsg.oCurrent=90;
                        break;
                    case 270:
                        localMsg.oCurrent=180;
                        break;
                }
                break;
            case 's':
                std::cout<<"Turn around from the current direction"<<std::endl;
                switch(localMsg.oCurrent)
                {
                    case 0:
                        localMsg.oCurrent=180;
                        break;
                    case 90:
                        localMsg.oCurrent=270;
                        break;
                    case 180:
                        localMsg.oCurrent=0;
                        break;
                    case 270:
                        localMsg.oCurrent=90;
                        break;
                }
                break;
            case 'd':
                std::cout<<"Turn right from the current direction"<<std::endl;
                switch(localMsg.oCurrent)
                {
                    case 0:
                        localMsg.oCurrent=90;
                        break;
                    case 90:
                        localMsg.oCurrent=180;
                        break;
                    case 180:
                        localMsg.oCurrent=270;
                        break;
                    case 270:
                        localMsg.oCurrent=0;
                        break;
                }
                break;
        }
        lcm.publish(localMsgChannel, &localMsg);
        std::cout<<"wait for two seconds"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    
}
*/
