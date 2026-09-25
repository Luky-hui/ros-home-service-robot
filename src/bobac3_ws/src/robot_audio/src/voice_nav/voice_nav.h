#include <iostream>
#include <vector>
#include <thread>
#include "ros/ros.h"
#include "robot_audio/Nav.h"
#include "robot_audio/robot_tts.h"
#include "robot_audio/robot_iat.h"
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std; 

class VoiceNav 
{
public:
	VoiceNav();
	~VoiceNav();

	ros::NodeHandle m_handle;

	void send_goal(int pnum);
	void nav_cancel();
	void state_detection();
	void tarvel();
        
    bool flag;
	bool send_flag;
	bool cancel_flag;
	bool travel_flag;

	vector<string> position_name;
	vector<double> position_x;
	vector<double> position_y;
	vector<double> orientation_z;
	vector<double> orientation_w;
	vector<string> position_introduction;

	void position_read();	
	 
	ros::ServiceServer voice_nav_service;
	ros::ServiceClient tts_client;
	ros::ServiceClient clear_client;
	robot_audio::robot_tts tts_srv;
	move_base_msgs::MoveBaseGoal goal;
	ros::Publisher pub;
	int goal_current_serial;
	bool voice_nav_deal(robot_audio::Nav::Request &req,robot_audio::Nav::Response &res);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;
	void turn();
private:
	thread travel_thread;
};
