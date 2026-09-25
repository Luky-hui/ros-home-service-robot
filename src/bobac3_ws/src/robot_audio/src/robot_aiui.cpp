#include<ros/ros.h>
#include"reinovo_voice.h"
#include"robot_audio/robot_iat.h"
#include"robot_audio/robot_semanteme.h"
#include"robot_audio/robot_tts.h"
#include"robot_audio/up_sync.h"
using namespace reinovo;
MyTalker m_talker;
bool audio_sync_cb(robot_audio::up_sync::Request  &req,robot_audio::up_sync::Response &res)
{
   m_talker.UP_SYNC();
   res.result = 1;
   return 1;
}
bool audio_tts_cb(robot_audio::robot_tts::Request &req,robot_audio::robot_tts::Response &res)
{
  m_talker.wakeup();
  int temp = m_talker.testTTs("./AIUI/audio/tts.wav",req.text);
    if(temp)
    {
      res.audiopath = "./AIUI/audio/tts.wav";
      if(req.play){
        system("aplay ./AIUI/audio/tts.wav");
      }
      return 1;
    }
    return 0;
}
bool audio_iat_cb(robot_audio::robot_iat::Request &req,robot_audio::robot_iat::Response &res)
{
  Json::Value m_semanteme;
  m_talker.wakeup();
  int temp = m_talker.writeAudioFromLocal(req.audiopath,&m_semanteme);
   if(temp)
   {
     res.text = m_semanteme["text"].asString();
     return 1;
   }
   return 0;
}
bool audio_semanteme_cb(robot_audio::robot_semanteme::Request &req,robot_audio::robot_semanteme::Response &res)
{
  int temp = 0;
  Json::Value m_semanteme;
  m_talker.wakeup();
  if(req.mode == 1)
       temp = m_talker.writeText(req.textorpath,&m_semanteme);
  else if(req.mode == 2)
       temp = m_talker.writeAudioFromLocal(req.textorpath,&m_semanteme);
  else
       {
         ROS_ERROR("robot_semanteme input error mode");
         return 0;
       }
  if(temp)
  {
    res.tech = m_semanteme["tech"].asString();
    res.iat = m_semanteme["text"].asString();
    res.anwser = m_semanteme["answer"].asString();
    res.intent = m_semanteme["intent"].asString();
    Json::Value m_slots;
    for (int var = 0; var < m_semanteme["slots"].size(); ++var)
    {
       m_slots = m_semanteme["slots"][var];
       res.slots_name.push_back(m_slots["name"].asString());
       res.slots_value.push_back(m_slots["normValue"].asString());
       std::cout<<res.slots_value[var]<<std::endl;
    }

    if (res.intent == "robot_control")
    {
        res.anwser = "好的！现在就"+res.iat;
    }
    if(res.intent == "robot_nav")
    {
      res.anwser = "好的！现在就带你去"+res.slots_value[1];
    }
    return 1;
  }
  return 0;
}
//audio_sync_cb
int main(int argc, char *argv[])
{
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ros::init(argc,argv,"robot_aiui");
  ros::NodeHandle nh;

  m_talker.createAgent();
  m_talker.start();
  m_talker.wakeup();
  //m_talker.UP_SYNC();
  ros::ServiceServer tts_service = nh.advertiseService("voice_tts",audio_tts_cb);
  ros::ServiceServer iat_service = nh.advertiseService("voice_iat",audio_iat_cb);
  ros::ServiceServer semanteme_service = nh.advertiseService("voice_aiui",audio_semanteme_cb);
  ros::ServiceServer UP_SYNC_service = nh.advertiseService("voice_up_sync",audio_sync_cb);
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  return 0;
}
