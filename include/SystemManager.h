#ifndef __SystemManager_h_
#define __SystemManager_h_

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <thread>
#include <string>
#include <map>
#include <utility>
#include "../AudioRecognition/AudioRecognizer.h"
#include "../AudioSynthesizer/AudioSynthesizer.h"
#include "../Navigator/navigator.h"
#include "../DataTransferer/dataTransferer.h"
#include "../LocalizationGeneratorTest/localizationGenerator.h"
#include "../HapticBelt/HapticBelt.h"
#include "../Timer/timerTrigger.h"
#include "../ImageMatcher/imageMatcherLCM.h"
#include "../ImageMatcher/imageMatcher.h"
#include "../LCMData/LCMDataType/haptic_belt_command_t.hpp"
#include "../LCMData/LCMDataType/voiceCommand.hpp"
#include "../LCMData/LCMDataType/locationBundle.hpp"
#include "../LCMData/LCMDataType/localizationMsg.hpp"
#include "../LCMData/LCMDataType/destinationMsg.hpp"
#include "../LCMData/LCMDataType/trigger.hpp"

class SystemManager {
public:
	SystemManager();
	~SystemManager();

	 int execute(int mode);
};

#endif
