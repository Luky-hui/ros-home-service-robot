#include "voice_control.h"
#include <iostream>
Control::Control(){
  controlServer = nh.advertiseService("voice_control", &Control::ControlCb, this);
  moveClient = nh.serviceClient<relative_move::SetRelativeMove>("relative_move");
  ttsClient = nh.serviceClient<robot_audio::robot_tts>("voice_tts");
}
 
Control::~Control(){
}

bool Control::ControlCb(robot_audio::Control::Request &req, robot_audio::Control::Response &res){

  robot_audio::robot_tts tts_srv;
	tts_srv.request.text = req.controlInfo[0];
	tts_srv.request.play = true;
	ttsClient.call(tts_srv);

	relative_move::SetRelativeMove moveSrv;
	moveSrv.request.global_frame = "odom";
	moveSrv.request.finishStopObstacle = true;
	if(req.controlInfo[0]=="前进"){
		moveSrv.request.goal.x = req.value;
	}
	else if(req.controlInfo[0]=="后退"){
		moveSrv.request.goal.x = -req.value;
	}
	else if(req.controlInfo[0]=="左移"){
		moveSrv.request.goal.y = req.value;
	}
	else if(req.controlInfo[0]=="右移"){
		moveSrv.request.goal.y = -req.value;
	}
	else if(req.controlInfo[0]=="左转"){
		moveSrv.request.goal.theta = req.value;
	}
	else if(req.controlInfo[0]=="右转"){
		moveSrv.request.goal.theta = -req.value;
	}
	
	moveClient.call(moveSrv);
	res.result= moveSrv.response.success;
  
	return true;
}

int main(int argc, char* argv[])
{
  /*初始化ros节点*/
  ros::init(argc, argv, "voiceControl");
  /*定义节点句柄*/
  Control control;
  ros::spin();
  return 0;
}
