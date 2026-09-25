#include <ros/ros.h>
#include <robot_audio/Collect.h>
#include <robot_audio/Control.h>
#include <robot_audio/Awake.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/robot_tts.h>
#include <robot_audio/robot_semanteme.h>
#include <robot_audio/Nav.h>
ros::ServiceClient collect_client;
ros::ServiceClient awake_client;
ros::ServiceClient control_client;
ros::ServiceClient nav_client;
ros::ServiceClient aiui_client;
ros::ServiceClient tts_client;
ros::ServiceClient iat_client;
int main(int argc, char** argv){
    ros::init(argc, argv, "voice_interaction");
    ros::NodeHandle m_handle;
    collect_client = m_handle.serviceClient<robot_audio::Collect>("voice_collect");
	awake_client = m_handle.serviceClient<robot_audio::Awake>("voice_awake");
	control_client = m_handle.serviceClient<robot_audio::Control>("voice_control");
	nav_client = m_handle.serviceClient<robot_audio::Nav>("voice_nav");
	aiui_client = m_handle.serviceClient<robot_audio::robot_semanteme>("voice_aiui");
	tts_client = m_handle.serviceClient<robot_audio::robot_tts>("voice_tts");
	iat_client = m_handle.serviceClient<robot_audio::robot_iat>("voice_iat");
    
    bool wakeup_flag = false;

    while(ros::ok()){
        robot_audio::Collect collect_srv;
        collect_srv.request.collect_flag = true;
		while(!wakeup_flag){
			if(collect_client.call(collect_srv)){
                robot_audio::robot_iat iat_srv;
				iat_srv.request.audiopath = collect_srv.response.voice_filename;
                if(iat_client.call(iat_srv)){
                    robot_audio::Awake awake_srv;
                    awake_srv.request.text = iat_srv.response.text;
                    if(awake_client.call(awake_srv)){
					if(awake_srv.response.awake_flag){
                            wakeup_flag = true;
                            system("aplay  ./AIUI/audio/awake.wav");
                            sleep(1);
                        }
                        else std::cout<< " 休眠中，请用唤醒词唤醒。" << std::endl;
                    }
                }
				
			}
		}
        if(collect_client.call(collect_srv)){
            robot_audio::robot_semanteme aiui_srv;
            aiui_srv.request.mode = 2;
            aiui_srv.request.textorpath = collect_srv.response.voice_filename;
            if(aiui_client.call(aiui_srv)){
                std::cout<<"请求---"<<aiui_srv.response.iat<<std::endl;
                std::cout<<"回答---"<<aiui_srv.response.anwser<<std::endl;
                if(aiui_srv.response.intent == "robot_nav"){
                    robot_audio::Nav nav_srv;
                    nav_srv.request.nav_order = aiui_srv.response.slots_value[1];
                    nav_client.call(nav_srv);
                }else if(aiui_srv.response.intent == "robot_guid"){
                    robot_audio::Nav nav_srv;
                    nav_srv.request.nav_order = "guidAround";
                    nav_client.call(nav_srv);
                }else if(aiui_srv.response.intent == "robot_control"){
                    robot_audio::Control ctrl_srv;
                    ctrl_srv.request.controlInfo.push_back(aiui_srv.response.slots_value[0]);
                    ctrl_srv.request.controlInfo.push_back(aiui_srv.response.anwser);
                    float dist;

                    if(aiui_srv.response.slots_value[2] == "厘米"){
                        dist = atof(aiui_srv.response.slots_value[1].c_str())/100.0;
                        ctrl_srv.request.value = dist;
                    }else if(aiui_srv.response.slots_value[2] == "毫米"){
                        dist = atof(aiui_srv.response.slots_value[1].c_str())/1000.0;
                        ctrl_srv.request.value = dist;
                    }else if(aiui_srv.response.slots_value[2] == "度"){
                        dist = atof(aiui_srv.response.slots_value[1].c_str()) * 3.1415/180.0;
                        ctrl_srv.request.value = dist;
                    }else ctrl_srv.request.value = atof(aiui_srv.response.slots_value[1].c_str());
                    control_client.call(ctrl_srv);
                }else{
                    robot_audio::robot_tts tts_srv;
                    tts_srv.request.play = true;
                    tts_srv.request.text = aiui_srv.response.anwser;
                    tts_client.call(tts_srv);
                } 
                wakeup_flag = false;
                sleep(1);
            }
        }

    }
    return 0;
}
