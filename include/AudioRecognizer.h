/***************************

  Audio Recognizer Class
  Created by Hongyi Fan
  Nov. 27 2017
  **************************/
#ifndef __AudioRecognizer_h_
#define __AudioRecognizer_h_


#include <iostream>
#include <string>
#include <pocketsphinx.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/err.h>
//#include <sphinxbase/jsgf.h>
//#include <sphinxbase/fsg_model.h>
#include <sys/select.h>

using namespace std;


class AudioRecognizer
{
public:
	AudioRecognizer(string hmmd, string dictd, string kws, string jsgf)
	{
		_hmmd = hmmd.c_str();
		_dictd = dictd.c_str();
		_kws = kws.c_str();
		_jsgf = jsgf.c_str();

		init();
	}
	~AudioRecognizer(){};
	void init();
	string recognize_command(int* score);
	string recognize_keyphrase();	
	string listen_for_user(int* score);
	static void sleep_msec(int32 ms);
private:
	const char * _hmmd = "en-us/en-us";
	const char * _dictd = "en-us/cmudict-en-us.dict";
	const char * _kws = "keyphrase.list";
	const char * _jsgf = "blindfind_audio_menu.gram";

	ps_decoder_t *ps;                  // create pocketsphinx decoder structure
	cmd_ln_t *config;                  // create configuration structure
	ad_rec_t *ad;                      // create audio recording structure - for use with ALSA functions


};

#endif
