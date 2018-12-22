/***************************

Audio Recognizer Class Implementation
Created by Hongyi Fan
Nov. 27 2017
**************************/


#include "AudioRecognizer.h"


void AudioRecognizer::init()
{
	config = cmd_ln_init(NULL, ps_args(), TRUE,                   // Load the configuration structure - ps_args() passes the default values
			"-hmm", _hmmd,  // path to the standard english language model
			"-dict", _dictd,                                      // custom dictionary (file must be present)
			"-logfn", "/dev/null",                                      // suppress log info from being sent to screen
			NULL);

//	cout << "1 from audiorecognizer" << endl;
  ps = ps_init(config);                                                        // initialize the pocketsphinx decoder
	
 // 	cout << "2 from audiorecognizer" << endl;
 // std::cout<<666<<std::endl;
    //jsgf_t * grammar = jsgf_parse_file(jsgf, NULL);
    //jsgf_rule_t * rule = jsgf_get_rule(grammar, "blindfind_audio_menu.command");
    //logmath_t * logmath = ps_get_logmath(ps);
    //int lw = cmd_ln_int32_r(config, "-lw");
    //fsg_model_t * fsg = jsgf_build_fsg(grammar, rule, logmath, lw);

    //ps_set_fsg(ps, "blindfind_audio_menu", fsg);

    //fsg_model_free(fsg);
    //jsgf_grammar_free(grammar);
    ps_set_jsgf_file(ps, "blindfind_audio_menu", _jsgf);
	  ps_set_kws(ps, "keyphrase_recognizer", _kws);

//	  cout << "3 from audio recognizer" << endl;
	    ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"), (int) cmd_ln_float32_r(config, "-samprate")); // open default microphone at default samplerate

		  ad_start_rec(ad);
}


string AudioRecognizer::recognize_keyphrase(){
	ps_set_search(ps,"keyphrase_recognizer");
	return listen_for_user(NULL);
}

string AudioRecognizer::recognize_command(int * score)
{
	ps_set_search(ps,"blindfind_audio_menu");
	return listen_for_user(score);
}

string AudioRecognizer::listen_for_user(int * score)
{
    int16 adbuf[2048];                 // buffer array to hold audio date
	uint8 utt_started, in_speech;      // flags for tracking active speech - has speech started? - is speech currently happening?
	int32 k;                           // holds the number of frames in the audio buffer
	
	ps_start_utt(ps);
	utt_started = FALSE;
	
	for (;;) {
		if ((k = ad_read(ad, adbuf, 2048)) < 0)
			E_FATAL("Failed to read audio\n");
		ps_process_raw(ps, adbuf, k, FALSE, FALSE);
		in_speech = ps_get_in_speech(ps);
		if (in_speech && !utt_started) {
			utt_started = TRUE;
			E_INFO("Listening...\n");
		}
		if (!in_speech && utt_started) {
			// speech -> silence transition, time to start new utterance 
			ps_end_utt(ps);
			char const * hyp = ps_get_hyp(ps, score);
			if (hyp != NULL) {
				return string(hyp);
			}
			if (ps_start_utt(ps) < 0)
				E_FATAL("Failed to start utterance\n");
			utt_started = FALSE;
			E_INFO("Ready....\n");
		}
		sleep_msec(100);
	}
}
void AudioRecognizer::sleep_msec(int32 ms) 
{
	    struct timeval tmo;

		    tmo.tv_sec = 0;
			    tmo.tv_usec = ms * 1000;

				    select(0, NULL, NULL, NULL, &tmo);
}
