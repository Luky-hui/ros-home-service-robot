#include"reinovo_voice.h"
using namespace reinovo;
std::string tts_save_path;
ofstream tts_out;
FILE*        fp           = NULL;
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int		size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int		fmt_size;		// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int		data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0
};
wave_pcm_hdr wav_hdr      =  default_wav_hdr;
//////lisener///////////////////////////////
MyListener::MyListener()
{
  Isnewsemanteme = 0 ;
  Isnewtts = 0;
  tts_datalen = 0;
  tts_percent = 0;
}
void MyListener::get_semanteme_data(Json::Value semanteme_data)
{
  //m_semanteme_data["text"]=""
    Json::Value& intent = semanteme_data["intent"];
    string service = intent["service"].asCString();
    cout<<"service is: "<<service<<endl;
    /*if (service.c_str()=="iFlytekQA")//tulingiFlytekQA
    {
        Json::Value& answer = intent["answer"];
        Json::Value& question = answer["question"];
        m_semanteme_data["tech"]="iFlytekQA";
        m_semanteme_data["text"] = question["question"];
        m_semanteme_data["answer"]=answer["text"];
        m_semanteme_data["intent"]=" ";
        m_semanteme_data["slots"]=" ";
    }*/
    if((service.find("OS2374344170")!=-1))//user intent
    {
        m_semanteme_data.clear();
        Json::Value& temp_semanteme = intent["semantic"][0];
        m_semanteme_data["tech"]="user";
        m_semanteme_data["text"]=intent["text"];
        m_semanteme_data["answer"]=" ";
        m_semanteme_data["intent"]=temp_semanteme["intent"];
        for (int var = 0; var < temp_semanteme["slots"].size(); ++var)
        {
           m_semanteme_data["slots"][var]= temp_semanteme["slots"][var];
        }
    }
    else//system intent
    {
        m_semanteme_data.clear();
        Json::Value& answer = intent["answer"];
        m_semanteme_data["tech"]="system";
        m_semanteme_data["text"]=intent["text"];
        m_semanteme_data["answer"]=answer["text"];
        m_semanteme_data["intent"]=" ";
        m_semanteme_data["slots"]=" ";
    }
}
void MyListener::__onEvent(const IAIUIEvent& event)
    {
        switch (event.getEventType()) {
            //SDK 状态回调
            case AIUIConstant::EVENT_STATE: {
                switch (event.getArg1()) {
                    case AIUIConstant::STATE_IDLE: {
                        cout << "EVENT_STATE:"
                             << "IDLE" << endl;
                    } break;

                    case AIUIConstant::STATE_READY: {
                        cout << "EVENT_STATE:"
                             << "READY" << endl;
                    } break;

                    case AIUIConstant::STATE_WORKING: {
                        cout << "EVENT_STATE:"
                             << "WORKING" << endl;
                    } break;
                }
            } break;
                //唤醒事件回调
            case AIUIConstant::EVENT_WAKEUP: {
                cout << "EVENT_WAKEUP:" << event.getInfo() << endl;
            } break;

                //休眠事件回调
            case AIUIConstant::EVENT_SLEEP: {
                cout << "EVENT_SLEEP:arg1=" << event.getArg1() << endl;
            } break;

                //VAD事件回调，如找到前后端点
            case AIUIConstant::EVENT_VAD: {
                switch (event.getArg1()) {
                    case AIUIConstant::VAD_BOS: {
                        cout << "EVENT_VAD: BOS" << endl;
                    } break;

                    case AIUIConstant::VAD_EOS: {
                        cout << "EVENT_VAD: EOS" << endl;
                    } break;

                    case AIUIConstant::VAD_VOL: {
                    } break;
                }
            } break;

                //最重要的结果事件回调
            case AIUIConstant::EVENT_RESULT: {
                Json::Value bizParamJson;
                Json::Reader reader;
                cout <<"hello is Me!"<< endl;
                cout<<"info is: "<<event.getInfo()<<endl;
                if (!reader.parse(event.getInfo(), bizParamJson, false)) {
                    cout << "parse error!" << endl << event.getInfo() << endl;
                    break;
                }
                Json::Value& data = (bizParamJson["data"])[0];
                Json::Value& params = data["params"];
                Json::Value& content = (data["content"])[0];
                string sub = params["sub"].asString();

                if (sub == "nlp") {
                    Json::Value empty;
                    Json::Value contentId = content.get("cnt_id", empty);
                    if (contentId.empty()) {
                        cout << "Content Id is empty" << endl;
                        break;
                    }
                    string cnt_id = contentId.asString();
                    cout<<cnt_id<<endl;
                    int dataLen = 0;
                    const char * resultStr = event.getData()->getBinary(cnt_id.c_str(),&dataLen);

                    if(resultStr == NULL){
                        return;
                    } 
                    cout<< resultStr <<endl;
                     //NSLog(@"resultStr=%s",resultStr);
                    Json::Value semanteme_data;
                    if (!reader.parse(resultStr, semanteme_data, false)) {
                        cout << "semanteme_data parse error!" << endl;
                        break;
                    }
                    get_semanteme_data(semanteme_data);
                    Isnewsemanteme =1;//new semanteme come
                    cout<<"talker Isnewsemanteme is "<<Isnewsemanteme<<endl;
                }
                else if(sub == "tts")
                {
                    Json::Value empty;
                    Json::Value contentId = content.get("cnt_id", empty);
                    int current_tts_percent = content["text_percent"].asInt();
                    int dts = content["dts"].asInt();
                    cout<<"the dts is:"<<dts<<endl;
                    if (contentId.empty()) {
                        cout << "Content Id is empty" << endl;
                        break;
                    }
                    string cnt_id = contentId.asString();
                    //tts_buffer = event.getData()->getBinary(cnt_id.c_str(), &tts_datalen);
                    tts_data = event.getData()->getBinary(cnt_id.c_str(), &tts_datalen);
                    //tts_out.write(tts_buffer,tts_datalen);
                    fwrite(tts_data, tts_datalen, 1, fp);
                    wav_hdr.data_size += tts_datalen;
                    if(dts >1)
                      {
                         Isnewtts = 2;
                         cout<<"the Isnestts is:"<<Isnewtts<<endl;
                       }
                    else
                      {
                         Isnewtts = 1;
                      }
                }
            } break;

                //上传资源数据的返回结果
            case AIUIConstant::EVENT_CMD_RETURN: {
                if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1()) {
                    if (event.getArg2() == 0) {
                        cout << "build grammar success." << endl;
                    } else {
                        cout << "build grammar error, errcode = " << event.getArg2() << endl;
                        cout << "error reasion is " << event.getInfo() << endl;
                    }
                } else if (AIUIConstant::CMD_UPDATE_LOCAL_LEXICON == event.getArg1()) {
                    if (event.getArg2() == 0) {
                        cout << "update lexicon success" << endl;
                    } else {
                        cout << "update lexicon error, errcode = " << event.getArg2() << endl;
                        cout << "error reasion is " << event.getInfo() << endl;
                    }
                }else if (AIUIConstant::CMD_SYNC == event.getArg1()) {//syn
                    if (event.getArg2() == 0) {
                        cout << "update SYNC success" << endl;
                        cout << "update SYNC info:" << event.getInfo()<< endl;
                        m_sid = event.getData()->getString("sid","0");
                        cout << "data is " <<m_sid << endl;
                    } else {
                        cout << "error reasion is " << event.getInfo() << endl;
                    }
                }else if (AIUIConstant::CMD_QUERY_SYNC_STATUS == event.getArg1()) {//syn check
                    cout << "the syn status check data is " << event.getData()->getString("result","0")<< endl;
                    if (event.getArg2() == 0) {
                        cout << "syn check OK!" << endl;
                        cout << "the syn status check info is " << event.getInfo() << endl;
                    } else {
                        cout << "error reasion is " << event.getInfo() << endl;
                    }
                }else if (AIUIConstant::CMD_SET_PARAMS == event.getArg1()) {//syn check
                    cout << "the set params info is: " << event.getInfo() << endl;
                    if (event.getArg2() == 0) {
                        cout << "set param OK!" << endl;
                        //cout << "the syn status check info is " << event.getInfo() << endl;
                    } else {
                        cout << "error reasion is " << event.getInfo() << endl;
                    }
                }
            } break;

            case AIUIConstant::EVENT_ERROR: {
                cout << "EVENT_ERROR:" << event.getArg1() << endl;
                cout << " ERROR info is " << event.getInfo() << endl;
            } break;
        }
    }

/////////////agent talker/////////////////////
MyTalker::MyTalker()
{
    std::cout << "Version: " << getVersion() << std::endl;
    AIUISetting::setAIUIDir(TEST_ROOT_DIR);
    AIUISetting::setMscDir(MSC_DIR);
    AIUISetting::setNetLogLevel(aiui_debug);
    AIUISetting::setLogLevel(aiui_debug);
    char mac[64] = { 0 };
    GenerateMACAddress(mac); // 设备唯一ID, 可自己重新实现
    printf("sn is: %s\n", mac);
    //为每一个设备设置对应唯一的SN（最好使用设备硬件信息(mac地址，设备序列号等）生成），以便正确统计装机量，避免刷机或者应用卸载重装导致装机量重复计数
    AIUISetting::setSystemInfo(AIUI_KEY_SERIAL_NUM, mac);
}
void MyTalker::createAgent()
{
    if (agent) {
        return;
    }

    Json::Value paramJson;

    std::cout << CFG_FILE_PATH << std::endl;

    string fileParam = readFileAsString(CFG_FILE_PATH);

    Json::Reader reader;
    if (reader.parse(fileParam, paramJson, false)) {
        agent = IAIUIAgent::createAgent(paramJson.toStyledString().c_str(), &listener);
    }

    if (!agent) {
        std::cout << "create agent fail" << std::endl;
    } else {
        std::cout << "create agent sucess" << std::endl;
    }
}
string MyTalker::readFileAsString(const string& path)
{
    std::ifstream t(path, std::ios_base::in | std::ios::binary);
    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

    return str;
}
void MyTalker::wakeup()
{
    if (NULL != agent) {
        IAIUIMessage* wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_WAKEUP);
        agent->sendMessage(wakeupMsg);
        wakeupMsg->destroy();
        cout<<"wakeup ok!"<<endl;
    }
}

void MyTalker::reset_wakeup()
{
    if (NULL != agent) {
        IAIUIMessage* wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_RESET_WAKEUP);
        agent->sendMessage(wakeupMsg);
        wakeupMsg->destroy();
    }
}

//停止AIUI服务，此接口是与stop()对应，调用stop()之后必须调用此接口才能继续与SDK交互，如果你没有调用过stop(),就不需要调用该接口
void MyTalker::start()
{
    if (NULL != agent) {
        IAIUIMessage* startMsg = IAIUIMessage::create(AIUIConstant::CMD_START);
        agent->sendMessage(startMsg);
        startMsg->destroy();
        cout<<"star ok!"<<endl;
    }
}

//停止AIUI服务
void MyTalker::stop()
{
    if (NULL != agent) {
        IAIUIMessage* stopMsg = IAIUIMessage::create(AIUIConstant::CMD_STOP);
        agent->sendMessage(stopMsg);
        stopMsg->destroy();
    }
}

// 写入测试音频
void MyTalker::writeAudio()
{
    if (NULL != agent) {
        IAIUIMessage* stopMsg =
            IAIUIMessage::create(AIUIConstant::CMD_START_RECORD, 0, 0, "data_type=audio");
        agent->sendMessage(stopMsg);
        stopMsg->destroy();
    }
}

bool MyTalker::writeAudioFromLocal(string audiopath, Json::Value* semanteme)
{
    if (NULL != agent) {
        ifstream testData(audiopath, std::ios::in | std::ios::binary);

        if (testData.is_open()) {
            char buff[1280];
            while (testData.good() && !testData.eof()) {
                memset(buff, '\0', 1280);
                testData.read(buff, 1280);

                // textData内存会在Message在内部处理完后自动release掉
                AIUIBuffer textData = aiui_create_buffer_from_data(buff, 1280);

                IAIUIMessage* writeMsg = IAIUIMessage::create(
                    AIUIConstant::CMD_WRITE,
                    0,
                    0,
                    "data_type=audio,pers_param={\"uid\":\"\"}",
                    textData);

                agent->sendMessage(writeMsg);
                writeMsg->destroy();

                // 必须暂停一会儿模拟人停顿，太快的话后端报错
                AIUI_SLEEP(40);
            }

            testData.close();

            IAIUIMessage* msg = IAIUIMessage::create(
                AIUIConstant::CMD_STOP_WRITE,
                0,
                0,
                "data_type=audio");
            agent->sendMessage(msg);
            msg->destroy();
        } else {
            printf("file[%s] open error", TEST_AUDIO_PATH);
            return 0;
        }
        int count =500;
        while(count)
        {
            count--;
            usleep(1000*10);
            if(listener.Isnewsemanteme)
            {
                *semanteme =listener.m_semanteme_data;
                listener.Isnewsemanteme = 0;
                return 1;
            }
        }
        return 0;
    }
}

void MyTalker::stopAudio()
{
    if (NULL != agent) {
        IAIUIMessage* stMsg = IAIUIMessage::create(AIUIConstant::CMD_STOP_RECORD);
        agent->sendMessage(stMsg);
        stMsg->destroy();
    }
}

//文本语义测试接口
bool MyTalker::writeText(string text,Json::Value* semanteme)
{
    if (NULL != agent) {
        // textData内存会在Message在内部处理完后自动release掉
        AIUIBuffer textData = aiui_create_buffer_from_data(text.c_str(), text.length());
        IAIUIMessage* writeMsg = IAIUIMessage::create(
            AIUIConstant::CMD_WRITE, 0, 0, "data_type=text,pers_param={\"uid\":\"\"}", textData);
        agent->sendMessage(writeMsg);
        writeMsg->destroy();
        int count =500;
        while(count)
        {
            count--;
            usleep(1000*10);
            if(listener.Isnewsemanteme)
            {
                *semanteme =listener.m_semanteme_data;
                listener.Isnewsemanteme = 0;
                return 1;
            }
        }
        return 0;
    }
}

int MyTalker::testTTs(std::string audiopath,std::string ttstext)
{
    if (NULL != agent)
    {
        tts_save_path = audiopath;
        fp = fopen( tts_save_path.c_str(), "wb");
        if(NULL == fp)
        {
            return 0 ;
        }
        fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
        AIUIBuffer textData = aiui_create_buffer_from_data(ttstext.c_str(), ttstext.length());
        IAIUIMessage* ttsMsg = IAIUIMessage::create(
            AIUIConstant::CMD_TTS, 1, 0, "text_encoding=utf-8,vcn=小娟", textData);
        agent->sendMessage(ttsMsg);
        ttsMsg->destroy();
        int count =10000;
        while(1)
        {
            count--;
            usleep(1000*1);
            if(listener.Isnewtts)
            {
                cout<<"the Isnewtts is: "<<listener.Isnewtts<<endl;
                if(listener.Isnewtts == 2)
                {
                    listener.Isnewtts = 0;
                    listener.tts_percent = 0;
                    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
                   /* 将修正过的数据写回文件头部,音频文件为wav格式 */
                    fseek(fp, 4, 0);
                    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
                    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
                    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
                    fclose(fp);
                    fp = NULL;
                    return 1;
                }
               listener.Isnewtts = 0;
            }
        }
        fclose(fp);
        fp = NULL;
        return 0;
    }
}

void MyTalker::destroy()
{
    if (NULL != agent) {
        agent->destroy();
        agent = NULL;
    }
}

void MyTalker::buildGrammar()
{
    if (NULL != agent) {
        string grammer = readFileAsString("AIUI/asr/call.bnf");

        auto msg =
            IAIUIMessage::create(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammer.c_str(), NULL);
        agent->sendMessage(msg);
        msg->destroy();
    }
}
/*****上传动态资源****/
void MyTalker::UP_SYNC()
{
   cout<<"开始上传了！"<<endl;
   Json::Value syncSchemaJson;
   Json::Value paramJson;
   Json::FastWriter json_write;

   paramJson["id_name"] = "uid";
   paramJson["res_name"] = "OS2374344170.pos_name";
   syncSchemaJson["param"] = paramJson;
   string filestr = readFileAsString("./AIUI/dist/pos_name.txt");
   cout<<"读取文件成功："<<filestr<<endl;
   int str_lengh = filestr.length();
   string base64str = m_base64.Encode((unsigned char *)(filestr.c_str()),str_lengh);
   int outcount;
   cout<<"转化为base64成功!"<<base64str<<endl;
   syncSchemaJson["data"] = base64str;
   string syncData = json_write.write(syncSchemaJson);

   cout<<"转化为字符串成功！"<<syncData<<endl;
   AIUIBuffer sendData = aiui_create_buffer_from_data(syncData.data(), syncData.length());
   cout<<"创建buffer成功!"<<endl;
   auto msg =IAIUIMessage::create(AIUIConstant::CMD_SYNC, AIUIConstant::SYNC_DATA_SCHEMA, 0, "", sendData);
   agent->sendMessage(msg);
   msg->destroy();
   //sleep(3);
}

void MyTalker::Check_SYNC()
{
   Json::Value paramJson;
   Json::FastWriter json_write;
   paramJson["sid"] = m_sid;
   string param_temp = json_write.write(paramJson);
   auto msg =IAIUIMessage::create(AIUIConstant::CMD_QUERY_SYNC_STATUS, AIUIConstant::SYNC_DATA_SCHEMA, 0, param_temp.c_str(), NULL);
   agent->sendMessage(msg);
   msg->destroy();
}

void MyTalker::GenerateMACAddress(char* mac) {

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            return;
        };

        struct ifconf ifc;
        char buf[1024];
        int success = 0;

        ifc.ifc_len = sizeof(buf);
        ifc.ifc_buf = buf;
        if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) {
            return;
        }

        struct ifreq* it = ifc.ifc_req;
        const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));
        struct ifreq ifr;

        for (; it != end; ++it) {
            strcpy(ifr.ifr_name, it->ifr_name);
            if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
                if (!(ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                    if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                        success = 1;
                        break;
                    }
                }
            } else {
                return;
             }
        }

        unsigned char mac_address[6];
        if (success) memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);
        sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
}
