/* Command matching node.
 * 
 * Institute for Robotics (ROBIN), JKU Linz
 * Alexander Reiter
 * February 2014
 * 
 * DESCRIPTION
 * 
 * PARAMETERS:
 *  
 * 
 * PUBLISHED TOPICS:
 *  
 * 
 * SUBSCRIBED TOPICS:
 *  
 * 
 * OFFERED SERVICES:
 *  
*/

#include <stdio.h>
#include <vector>
#include <sstream>
#include <algorithm>  // transform(.)

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"

using namespace std;
#include "Dictionary.cpp"


enum Command_id { SERIVCE_MODE_0 = 0, 
                  SERIVCE_MODE_2 = 1, 
                  SERIVCE_MODE_3 = 2, 
                  ACK_SERVICE = 3, 
                  STOP_SERVICE = 4, 
                  TALK_INTRO = 5, 
                  TALK_LIKE = 6, 
                  TALK_PLEASURE = 7,
                  TALK_THANKS = 8
};

Dictionary dict;

// function declarations
void speech_callback(const std_msgs::String::ConstPtr& msg);

// publisher for commands
ros::Publisher cmd_pub;

int main(int argc, char** argv) {
  ros::init(argc, argv, "command_matcher_node");
  ros::NodeHandle n("~"); // local node handle for accessing local parameters
  
  ros::Subscriber speech_sub = n.subscribe<std_msgs::String>("/speech_recognition", 1000, speech_callback);
  
  // init publisher for commands
  cmd_pub = n.advertise<std_msgs::UInt16>("/cmd", 1000);
  
  // read commands from parameter server
  string cmd;
  size_t i = 0;
  char par_name[8];
  snprintf(par_name, 8, "cmd_%03d", i);
  while(n.hasParam(par_name)) {
    n.param<std::string>(par_name, cmd, "default_value");
    dict.add_entry(cmd);
    i++;
    snprintf(par_name, 8, "cmd_%03d", i);
  }
  
  ROS_INFO("number of dictionary entries: %d", dict.size());
  
  
  ros::Rate r(10);
  while(ros::ok()) {
    
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}

/* Callback function that is called when a string message containing 
 * recognised speech is received.
 * The received text is matched to pre-defined commands using a 
 * cost function-based matching algorithm.
 * 
 * INPUT: const std_msgs::String::ConstPtr& msg: pointer to string message to be matched
 * OUTPUT:  none
 */
void speech_callback(const std_msgs::String::ConstPtr& msg) {
  string data = msg->data;
  // transform characters to lower case
  transform(data.begin(), data.end(), data.begin(), ::tolower);
  
  int res = dict.findBestMatch(data);
  
  if(res >= 0) {
    std_msgs::UInt16 send_msg;
    send_msg.data = (uint16_t) res;
    cmd_pub.publish(send_msg);
  }
  
  
  ROS_INFO("%s matched to command ID %d", data.c_str(), res);
}
