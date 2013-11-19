/*	Node that performs text-to-speech conversion by using Google's
	speech synthesiser.
	Alexander Reiter, Institute for Robotics (ROBIN), JKU Linz
	November 2013
	
	This node sends request to the soundplay_node to play audio files
	that contain synthesised speech. It checks with a dictonary if the 
	to-be-synthesised text has been handled before. If so, the existing
	audio file is used, otherwise, synthesised speech is downloaded from
	Google. The filename is a 50 character string and is chosen randomly.
	To determine whether an audio file that corresponds to the
	text already exists, the node accesses a json dictionary that 
	contains pre-downloaded files with synthesised speech. It holds 
	strings as keys and filenames as values.
	
	
	PARAMETERS:
	audioPath: path to folder with audio files (does not need to end with '/')
	jsonPath: path to json file with dictionary
	language: language code for speech synthesis (en, de, etc.)
	
	
	References:
	Google TTS API
	http://elinux.org/RPi_Text_to_Speech_%28Speech_Synthesis%29#Google_Text_to_Speech
	Jansson API
	https://jansson.readthedocs.org/en/2.5/apiref.html
*/

#include <iostream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sound_play/SoundRequest.h"
#include <ctype.h>
#include <stdio.h>
#include <curl/curl.h>
#include <jansson.h>
#include <time.h> 

using namespace std;

void stringCallback(const std_msgs::String msg);
void createFilename();

ros::Publisher soundPub;	// publisher for sound_play requests

// ROS parameters variables
string audioPath;	// path to audio folder
string jsonPath;	// path to json file with audio references
string language;	// language code

json_t* root;	// root node of json structure

// curl variables
CURL *curl;
CURLcode res;
char filename[51];
 
int main(int argc, char** argv) {
  
  // init ROS
  ros::init(argc, argv, "getSpeech");
  ros::NodeHandle n("~");
  ros::Subscriber sub;
  sub = n.subscribe("/speech", 10, stringCallback);
  soundPub = n.advertise<sound_play::SoundRequest>("/robotsound", 10);
  
  ros::Rate loop_rate(10);
  if(!n.getParam("audioPath", audioPath)) {
	  ROS_ERROR("Parameter audioPath not found");
	  ros::shutdown();
  }
  
  if(!n.getParam("language", language)) {
	  ROS_ERROR("Parameter language not found");
	  ros::shutdown();
  }
  if(!n.getParam("jsonPath", jsonPath)) {
	  ROS_ERROR("Parameter jsonPath not found");
	  ros::shutdown();
  }
  
  // prepare rand
  srand(time(NULL));
  srand(time(NULL) + rand());
  
  // init json file
  json_error_t* error;
  root = json_load_file(jsonPath.c_str(), 0, error);
  if(!root) {
	  ROS_DEBUG("json file %s does not exist, a new file will be created", jsonPath.c_str());
	  root = json_object();
  } else {
	  ROS_DEBUG("json file %s successfully opened", jsonPath.c_str());
  }
  
  // init curl
  curl = curl_easy_init();
 
  while (curl && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  curl_easy_cleanup(curl);
 
  return 0;
}

/* Callback function that checks if the current json structure already
 * contains an entry that corresponds to the string from the message. If
 * no entry exists, it a new a new speech file is downloaded and stored
 * using curl and a new json entry is created.
 * The path of the audio file that corresponds to the string in message
 * is stored in a SoundRequest message and advertised on the robotsound
 * topic.
 * 
 * INPUT:	const std_msgs::String msg: message with string to be translated
 * OUTPUT:	none
 */
void stringCallback(const std_msgs::String msg) {
	
	// check if entry for this string exists in json file
	json_t* entry = json_object_get(root, msg.data.c_str());	// holds json_string object with audio file path
	if(!entry) {
		ROS_DEBUG("Entry does not exist, downloading new file");
		
		// prepare URL for curl
		stringstream urlStream;
		urlStream << "http://translate.google.com/translate_tts?tl=";
		urlStream << language << "&q=";
		char* str = curl_easy_escape(curl, msg.data.c_str(), 0);
		urlStream << str;
		curl_free(str);
		
		stringstream filenameStream;
		FILE* file = fopen("/dev/null", "r");	// some existing file
		while(file) {	// generate new filenames until corresponding file does not exist
			curl_easy_setopt(curl, CURLOPT_URL, urlStream.str().c_str());
			filenameStream.str("");
			filenameStream.clear();
			filenameStream << audioPath;
			// check if audioPath ends with '/'
			if(audioPath.find_last_of('/') < audioPath.length()) {
				filenameStream << '/';
			}
			createFilename();
			filenameStream << filename;
			file = fopen(filenameStream.str().c_str(), "r");
		}
		// file does not exist, create it
		file = fopen(filenameStream.str().c_str(), "w");
		if(!file) {
			ROS_ERROR("Could not open file %s", filenameStream.str().c_str());
			fclose(file);
			return;
		}
		
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, file) ;
		res = curl_easy_perform(curl);
		fclose(file);
		
		/* check for curl errors */ 
		if(res == CURLE_OK) {
			ROS_DEBUG("File downloaded and stored in %s", filenameStream.str().c_str());
		} else {
			ROS_ERROR("Download failed, curl_easy_perform() error: %s", curl_easy_strerror(res));
			return;
		}
		
		entry = json_string(filenameStream.str().c_str());
		json_object_set(root, msg.data.c_str(), entry);
		
		if(json_dump_file(root, jsonPath.c_str(), 0) == 0) {
			ROS_DEBUG("json file successfully written to %s", jsonPath.c_str());
		} else {
			ROS_ERROR("json file could not be written, all changes are lost");
		}
	}
	
	// create stop message
	sound_play::SoundRequest req;
	req.sound = req.ALL;
	req.command = req.PLAY_STOP;
	soundPub.publish(req);
	
	// create speech message
	req.sound = req.PLAY_FILE;
	req.command = req.PLAY_ONCE;
	req.arg = json_string_value(entry);
	
	soundPub.publish(req);
}

/* Function that writes a random sequence of (human readable) characters
 * to the global filename c string (0 terminated).
 * 
 * INPUT:	none
 * OUTPUT:	none
 */
void createFilename() {
	int i;
	char ch;
	for(i = 0; i < 50; i++) {
		ch = (char) rand() % 62;
		if(ch < 10) ch = ch + 48; // numbers
		else if(ch < 36) ch = (ch - 10) + 65; // capital letters
		else ch = (ch - 36) + 97; // small letters
		filename[i] = ch;
	}
	filename[50] = 0;
}

