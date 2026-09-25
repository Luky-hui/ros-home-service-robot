#include "audio_collect.h"
#include <vector>
#include "robot_audio/Collect.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

class Work
{
public:
    Work();
    ~Work();

    AudioCollect  audio;
    vector<boost::shared_ptr<char> >  voice;
    std_msgs::String  audio_file;
    string file_name;

    ros::NodeHandle m_handle;
    ros::ServiceServer collect_service;
    bool collect_control(robot_audio::Collect::Request &req,robot_audio::Collect::Response &res);

    void run();
    //ros::Publisher m_voice_pub;
    //boost::thread pub_thread;

    //void getVoice();

};
