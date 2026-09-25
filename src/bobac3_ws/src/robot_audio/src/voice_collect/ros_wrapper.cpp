#include "ros_wrapper.h"

Work::Work()
{
    collect_service = m_handle.advertiseService("voice_collect",&Work::collect_control,this);

    ros::param::get("~audio_file",file_name);
    cout<<"----"<<file_name<<endl;

}

Work::~Work()
{


} 

bool Work::collect_control(robot_audio::Collect::Request &req,robot_audio::Collect::Response &res)
{
    if(req.collect_flag) {
        voice = audio.get_voice();
        audio.write_wav(voice,file_name);
        res.voice_filename = file_name;
    } else {
        cout<<"collect_flag error"<<endl;
        res.voice_filename = "collect_flag_error";
    }

    return true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"voice_collect");
    Work  voice_work;
    ros::spin();

}
