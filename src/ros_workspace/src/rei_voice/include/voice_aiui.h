#ifndef REIAIUI_H
#define REIAIUI_H

#include <unistd.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include "reinovo_key_comm.h"
#include "aiui/AIUI_V2.h"
#include "aiui/PcmPlayer_C.h"
#include "json/json.h"
#include "StreamNlpTtsHelper.h"
#include "IatResultUtil.h"
#include "Base64Util.h"
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

#define AIUI_SLEEP(x) usleep(x * 1000)
// 使用的AIUI服务版本：1（语义技能），2（语义技能 + 大模型），3（语义技能 + 大模型 + 极速交互）
#define AIUI_VER 3

#if AIUI_VER == 3
    // 是否使用语义后合成。当在AIUI平台应用配置页面打开"语音合成"开关时，需要打开该宏
    #define USE_POST_SEMANTIC_TTS
#endif
using namespace std;
using namespace aiui_va;
using namespace aiui_v2;

// AIUI结果结构体声明
struct AIUIResultData {
    string type;  // "iat", "tts", "nlp", "wakeup", "error"
    string content;
    string sid;
    int code;
    int eos_time;
};

void onStarted();
void onPaused();
void onResumed();
void onStopped();
void onError(int error, const char* des);
int playLocalPcmFile(const string& filePath);
// 从IAIUIListener派生自己的结果监听器
class REIListener : public IAIUIListener
{
private:
    // 从StreamNlpTtsHelper::Listener派生流式合成监听器，用于监听大模型结果的合成
    class TtsHelperListener : public StreamNlpTtsHelper::Listener
    {
    public:
        void onText(const StreamNlpTtsHelper::OutTextSeg& textSeg) override;
        void onFinish(const string& fullText) override;
        void onTtsData(const Json::Value& bizParamJson, const char* audio, int len) override;
    };
    void onEvent(const IAIUIEvent& event) override;
public:
    REIListener();
    ~REIListener()
    {
        // 析构时销毁播放器，释放资源
        aiui_pcm_player_destroy();
        subResult = " ";
        ttsend = 0;
    }
    Json::Value inient_query;
    fstream mFs;
    string mCurTtsSid;// 当前合成sid
    string mCurIatSid;// 当前识别sid#include"zbase64.h"
    string mIatTextBuffer;// 识别结果缓存
    string mStreamNlpAnswerBuffer;// 流式nlp的应答语缓存
    std::shared_ptr<StreamNlpTtsHelper> m_pTtsHelper;
    int mStreamTtsIndex{0};
    string subResult;
    
    int ttsend;
    int gPcmPlayerIndex = -1; //播放器索引
    int mTtsLen = 0;// 当前收到了tts音频的长度
    int mIntentCnt = 0;// 意图的数量
    bool mMoreDetails = true;
    void handleEvent(const IAIUIEvent& event);
};

#define TEST_ROOT_DIR "./REIAIUI/"
#define CFG_FILE_PATH "./REIAIUI/cfg/aiui.cfg"
#define TEST_AUDIO_PATH "./REIAIUI/audio/test.pcm"
#define MSC_DIR         "./REIAIUI/msc/"
#define SYNC_PATH "./REIAIUI/dist/"
#define TTS_PATH "./REIAIUI/audio/tts.pcm"
#define PCM_BUFFER_SIZE 4096  // 文件播放时的单次读取缓冲区大小
#define PCM_PLAYER_DRAIN_DELAY 500000 // 播放完成后等待延时（500ms，替代drain接口）


IAIUIAgent* g_pAgent = nullptr;
string gSyncSid;

class ReiTalker{
public:
    ReiTalker();
    REIListener* g_pListener = nullptr; // NOLINT
    string gSyncSid;
    int gPcmPlayerIndex = -1;

    
    void destroyAgent();
    void wakeup();
    void resetWakeup();
    void stop();
    void start();
    void resetAIUI();
    void writeAudioFromLocal(bool repeat);
    void writeText(const string& text, bool needWakeup = true);
    int startTTS(const string& text, const bool& tag);
    void syncSchemaData(const string& name,int type);

    void startRecordAudio();
    void stopRecordAudio();
    
private:
    void GenerateMACAddress(char* mac);
    void initSetting(bool log = true);
    int createAgent(bool more = true);
    string readFileAsString(const string& path);
};

//消息发送宏
#define SEND_AIUIMESSAGE(cmd, arg1, arg2, params, data)                               \
    do {                                                                         \
        if (!g_pAgent) break;                                                       \
        IAIUIMessage* msg = IAIUIMessage::create(cmd, arg1, arg2, params, data); \
        g_pAgent->sendMessage(msg);                                                 \
        msg->destroy();                                                          \
    } while (false)
//简化向g_pAgent发送消息的操作。
#define SEND_AIUIMESSAGE4(cmd, arg1, arg2, params) SEND_AIUIMESSAGE(cmd, arg1, arg2, params, nullptr)
#define SEND_AIUIMESSAGE3(cmd, arg1, arg2)              SEND_AIUIMESSAGE4(cmd, arg1, arg2, "")
#define SEND_AIUIMESSAGE2(cmd, arg1)                         SEND_AIUIMESSAGE3(cmd, arg1, 0)
#define SEND_AIUIMESSAGE1(cmd)                                    SEND_AIUIMESSAGE2(cmd, 0)

#endif