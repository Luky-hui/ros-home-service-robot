#include <string>
#include"zbase64.h"
#include "aiui/AIUI_V2.h"
#include "json/json.h"
#include <fstream>
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std;
using namespace VA;
using namespace aiui_v2;

#define AIUI_KEY_SERIAL_NUM       "sn"
#define AIUI_KEY_AUTH_PUBLIC_KEY  "auth_public_key"
#define AIUI_KEY_AUTH_PRIVATE_KEY "auth_private_key"

#define TEST_ROOT_DIR   "./AIUI/"
#define CFG_FILE_PATH   "./AIUI/cfg/aiui.cfg"
#define TEST_AUDIO_PATH "./AIUI/audio/test.pcm"
#define LOG_DIR         "./AIUI/log/"
#define MSC_DIR         "./AIUI/msc/"

#define AIUI_SLEEP(x) usleep(x * 1000)
#undef AIUI_LIB_COMPILING
namespace reinovo {
class MyListener : public IAIUIListener
{
public:
    string m_sid;//动态实体上传查询id
    void onEvent(const IAIUIEvent& event) override
        {
            try {
                __onEvent(event);
            } catch (std::exception& e) {
                printf("%s", e.what());
            }
        }
     void get_semanteme_data(Json::Value semanteme_data);
     int Isnewsemanteme;
     int Isnewtts;
     const char* tts_buffer;
     const void* tts_data;
     int tts_percent;
     int tts_datalen;
     Json::Value m_semanteme_data;
     MyListener();
private:
    void __onEvent(const IAIUIEvent& event);
};

class MyTalker
{
public:
    ZBase64 m_base64;//解码器
    string m_sid;//动态实体上传查询id
    MyListener listener;
    IAIUIAgent* agent = nullptr;//交互对象指针
    string readFileAsString(const string& path);//读取文件内容并转化为字符串
    streamsize Read(istream& stream, char* buffer, streamsize count);
    MyTalker();
    void createAgent();//创建aiui对象
    void wakeup();//唤醒
    void reset_wakeup();//重置唤醒，调用后需要重新唤醒
    void start();//开启aiui交互
    void stop();//停止aiui交互；
    void writeAudio();
    //int writeAudioFromLocal(string );//从本地写入交互音频
    bool writeAudioFromLocal(string audiopath, Json::Value* semanteme);
    void stopAudio();//
    //文本语义测试接口
    bool writeText(string text,Json::Value* semanteme);//写入交互语言
    int testTTs(std::string audiopath,std::string ttstext);//语音合成
    void destroy();//销毁交互对象
    void buildGrammar();
    void UP_SYNC();//上传动态实体
    void Check_SYNC();//检查动态实体是否生效
    void GenerateMACAddress(char* mac);//获取mac地址
};
}
