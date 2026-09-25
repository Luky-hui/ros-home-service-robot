#include "ros/ros.h"
#include "robot_audio/Control.h"
#include "robot_audio/robot_tts.h"
#include <unistd.h>
#include <relative_move/SetRelativeMove.h>

class Control{
public:
    Control();
    ~Control();
private:
	ros::NodeHandle nh;
	ros::ServiceServer controlServer;
	bool ControlCb(robot_audio::Control::Request &req,robot_audio::Control::Response &res);
	ros::ServiceClient moveClient;
	ros::ServiceClient ttsClient;
};
