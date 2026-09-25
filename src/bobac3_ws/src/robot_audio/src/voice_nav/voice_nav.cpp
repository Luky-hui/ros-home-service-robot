#include "voice_nav.h"

VoiceNav::VoiceNav():cancel_flag(false),travel_flag(true),send_flag(false),goal_current_serial(0)
{

    position_read();
    
    voice_nav_service = m_handle.advertiseService("voice_nav",&VoiceNav::voice_nav_deal,this);
    tts_client = m_handle.serviceClient<robot_audio::robot_tts>("voice_tts");
    clear_client = m_handle.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");
    ac = new AC("move_base", true);
    
}
VoiceNav::~VoiceNav()
{
    if(travel_thread.joinable()){
        travel_thread.join();
    }
	delete ac;
}

void VoiceNav::position_read()
{
    FILE* fp;
    char str[300];
    double tmp;
    fp = fopen("./AIUI/dist/position_info.txt", "r");
    if(!fp){
        cout<<"open position info  file  erro"<<endl;
    }
    while(1){	
        if(fscanf(fp, "%s", str) == EOF)
            break;
        
        position_name.push_back(str);

        if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
        position_x.push_back(tmp);

        if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
        position_y.push_back(tmp);

        if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
        orientation_z.push_back(tmp);

        if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
        orientation_w.push_back(tmp);
        if(fscanf(fp,"%s",str) == EOF)
            break;
		position_introduction.push_back(str);
    }
    fclose(fp);

}

void VoiceNav::nav_cancel(){
	ac->cancelGoal();
    cancel_flag = true;
}

void VoiceNav::send_goal(int pnum){
    std_srvs::Empty clear_srv;
    clear_client.call(clear_srv);
    sleep(1);
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = position_x[pnum];
    goal.target_pose.pose.position.y = position_y[pnum];	

    goal.target_pose.pose.orientation.z = orientation_z[pnum];
    goal.target_pose.pose.orientation.w = orientation_w[pnum];
    tts_srv.request.text = position_introduction[pnum];
    cout<<"----"<<position_name[pnum]<<"----"<<endl;
    ROS_INFO("Sending goal");
    ac->sendGoal(goal);
    send_flag = true;
}
void VoiceNav::state_detection(){
    ros::Rate rate(2);
	while(!ac->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
    while(ros::ok()){
        if(send_flag){
            if((ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                if(tts_client.call(tts_srv)) {
                    //system("aplay  ./voice_config/tts.wav");
                }
                travel_flag = false;
                send_flag = false;
                return;
            }
            if(ac->getState() != actionlib::SimpleClientGoalState::ACTIVE){
                cancel_flag =true;
                send_flag = false;
                return;
            }
        cout << ac->getState().toString() <<endl;
        ac->getResult();
        }
        
        rate.sleep();
    }
}
void VoiceNav::tarvel(){
    ros::Rate loop(10);
    ROS_INFO("<<<<<start travel>>>>>");
    for(int i=0; i<position_name.size(); i++){
        travel_flag =true;
        goal_current_serial = i;
        send_goal(i);
        while(travel_flag&&ros::ok()){
            loop.sleep();
            if(cancel_flag) goto tflag;
        }
        
    }
    tts_srv.request.text = "所有地点都参观完了，谢谢使用";
    tts_client.call(tts_srv);
    flag = true;
tflag:
    flag = true;
    cancel_flag = false;
}
bool VoiceNav::voice_nav_deal(robot_audio::Nav::Request &req,robot_audio::Nav::Response &res)
{
    thread detect_thread(&VoiceNav::state_detection, this);
    cancel_flag = false;
    int position_count;
    tts_srv.request.play = true;
    if(req.nav_order=="goOrigin"){
        tts_srv.request.text = "好的，我回去了，有需要随时叫我";
        tts_client.call(tts_srv);
        send_goal(position_name.size()-1);
        goto flag;
    } 
    else if(req.nav_order.find("取消导航") != string::npos){
        res.position = "取消导航";
        nav_cancel();
        goto flag;
    }
    else if(req.nav_order== "guidAround"){
        res.position = "整体游览";
        cancel_flag = false;
        flag = false;
        tts_srv.request.text = "好的，我这就带着您参观一下";
        
        if(travel_thread.joinable()){
            ROS_INFO("正在游览");
            tts_srv.request.text = "不好意思，正在游览";
            tts_client.call(tts_srv);
            travel_thread.join();
        }
        else{
            //travel_thread = thread(&VoiceNav::tarvel, this);
            tarvel();
            tts_client.call(tts_srv);
        }
        while(!flag){
	
        }
        goto flag;
    } 
    else{
        for(int i=0;i<position_name.size();i++){
            if(req.nav_order == position_name[i]){
                res.position = position_name[i];
                goal_current_serial = position_count;
                tts_srv.request.text = "好的，这就带您去"+res.position;
                tts_client.call(tts_srv);
                send_goal(i);
                
                goto flag;
            }
        }
        cout<<"error position name "<<endl;
        res.position = req.nav_order;
        tts_srv.request.text = "不好意思，我不知道"+req.nav_order+"在哪儿";
    }
flag:
    if(detect_thread.joinable()){
        detect_thread.join();
    }
    return true;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"voice_nav");
    VoiceNav voicenav;
    
    ros::spin();


    return 0;
}
