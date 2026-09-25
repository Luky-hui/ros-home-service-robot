#include "voice_aiui.h"
#include "reinovo_key_comm.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "rei_voice/REITts.h"
#include "rei_voice/REIPlayer.h"
#include "rei_voice/REINlp.h"
#include "rei_voice/REIResult.h"
#include "rei_voice/REIResultNlp.h"
#include "rei_voice/UserData.h"
#include "rei_voice/SendEmail.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"


ros::ServiceServer tts_service;
ros::ServiceServer playPcm_service;
ros::ServiceServer nlp_service;
ros::ServiceServer upSync_service;
ros::ServiceServer recordAudio_service;
ros::ServiceServer login_service;
ros::ServiceServer register_service;
ros::ServiceServer send_email_service;

ros::Publisher aiuiResult;
ros::Publisher aiuiState;

rei_voice::REIResult resultmsg;

void REIListener::handleEvent(const IAIUIEvent& event)
{
    switch (event.getEventType()) {
        // SDK状态
        case AIUIConstant::EVENT_STATE: {
            switch (event.getArg1()) {
                case AIUIConstant::STATE_IDLE: {
                    // 空闲状态，即最初始的状态
                    cout << "EVENT_STATE: STATE_IDLE" << endl;
                    std_msgs::String msg;
                    msg.data = "EVENT_STATE: STATE_IDLE";
                    aiuiState.publish(msg); 
                } break;

                case AIUIConstant::STATE_READY: {
                    // 准备好状态（待唤醒），可以进行唤醒
                    cout << "EVENT_STATE: STATE_READY" << endl;
                    std_msgs::String msg;
                    msg.data = "EVENT_STATE: STATE_READY";
                    aiuiState.publish(msg); 
                } break;

                case AIUIConstant::STATE_WORKING: {
                    // 工作状态（即已唤醒状态），可以语音交互，也可以再次唤醒
                    cout << "EVENT_STATE: STATE_WORKING" << endl;
                    std_msgs::String msg;
                    msg.data = "EVENT_STATE: STATE_WORKING";
                    aiuiState.publish(msg); 
                } break;
            }
        } break;
        // 唤醒事件
        case AIUIConstant::EVENT_WAKEUP: {
            cout << "EVENT_WAKEUP: " << event.getInfo() << endl;
            // 唤醒时停止播放
            aiui_pcm_player_stop();
            std_msgs::String msg;
            msg.data = "EVENT_WAKEUP";
            aiuiState.publish(msg); 
        } break;

        // 休眠事件，即一段时间无有效交互或者外部主动要求，SDK会自动进入STATE_READY状态
        case AIUIConstant::EVENT_SLEEP: {
            // arg1用来区分休眠类型，是自动休眠还是外部要求，可参考AIUIConstant.h中EVENT_SLEEP的注释
            cout << "EVENT_SLEEP: arg1=" << event.getArg1() << endl;
        } break;
        // VAD事件，如语音活动检测
        case AIUIConstant::EVENT_VAD: {
            // arg1为活动类型
            switch (event.getArg1()) {
                case AIUIConstant::VAD_BOS_TIMEOUT: {
                    cout << "EVENT_VAD: VAD_BOS_TIMEOUT" << endl;
                } break;

                // 检测到前端点，即开始说话
                case AIUIConstant::VAD_BOS: {
                    cout << "EVENT_VAD: BOS" << endl;
                    std_msgs::String msg;
                    msg.data = "EVENT_VAD: BOS";
                    aiuiState.publish(msg); 
                } break;

                // 检测到后端点，即说话结束
                case AIUIConstant::VAD_EOS: {
                    cout << "EVENT_VAD: EOS" << endl;
                    std_msgs::String msg;
                    msg.data = "EVENT_VAD: EOS";
                    aiuiState.publish(msg); 
                } break;

                // 音量，arg2为音量级别（0-30）
                case AIUIConstant::VAD_VOL: {
                    cout << "EVENT_VAD: vol=" << event.getArg2() << endl;
                } break;
            }
        } break;
        // 结果事件
        case AIUIConstant::EVENT_RESULT: {
            Json::Value bizParamJson;
            Json::Reader reader;
            if (!reader.parse(event.getInfo(), bizParamJson, false)) {
                cout << "parse error! info=" << event.getInfo() << endl;
                break;
            }

            Json::Value& data = (bizParamJson["data"])[0];
            Json::Value& params = data["params"];
            Json::Value& content = (data["content"])[0];

            string sub = params["sub"].asString();
            if (sub != "iat" && sub != "nlp" && sub != "tts" && sub != "cbm_tidy" && sub != "cbm_semantic") {
                return;
            }

            // sid即唯一标识一次会话的id
            string sid = event.getData()->getString("sid", "");
            if (sub == "iat") {
                if (sid != mCurIatSid) {
                    resultmsg = rei_voice::REIResult();
                    cout << "**********************************" << endl;
                    cout << "sid=" << sid << endl;
                    mCurIatSid = sid;
                    resultmsg.sid = sid;
                    // 新的会话，清空之前识别缓存
                    mIatTextBuffer.clear();
                    mStreamNlpAnswerBuffer.clear();
                    m_pTtsHelper->clear();
                    mIntentCnt = 0;
                }
            } else if (sub == "tts") {
                if (sid != mCurTtsSid) {
                    cout << "**********************************" << endl;
                    cout << "sid=" << sid << endl;
                    mTtsLen = 0;
                    mCurTtsSid = sid;
                }
            }

            Json::Value empty;
            string cnt_id = content.get("cnt_id", empty).asString();

            int dataLen = 0;

            // 注意：当buffer里存字符串时也不是以0结尾，当使用C语言时，转成字符串则需要自已在末尾加0
            const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
            if (sub == "tts") {
                Json::Value&& isUrl = content.get("url", empty);
                if (isUrl.asString() == "1") {
                    // 云端返回的是url链接，可以用播放器播放
                    cout << "tts_url=" << string(buffer, dataLen) << endl;
                } else {
                    // 云端返回的是pcm音频，分成一块块流式返回
                    int progress = 0;
                    int dts = content["dts"].asInt();

                    string tag = event.getData()->getString("tag", "");
                    if (tag.find("stream_nlp_tts") == 0) {
                        // 流式语义应答的合成
                        m_pTtsHelper->onOriginTtsData(tag, bizParamJson, buffer, dataLen);
                    } else {
                        
                        mTtsLen += dataLen;
                        // 音频开始
                        if (dts == AIUIConstant::DTS_BLOCK_FIRST ||
                            dts == AIUIConstant::DTS_ONE_BLOCK ||
                            /* 特殊情况:合成字符比较少时只有一包数据而且状态是２ */
                            (dts == AIUIConstant::DTS_BLOCK_LAST && 0 == mTtsLen)) {
                            mFs.open(TTS_PATH, ios::binary | ios::out);
                        }
                        if (dataLen > 1) {
                            mFs.write(buffer, dataLen);
                        }
                        // 音频结束
                        if (dts == AIUIConstant::DTS_BLOCK_LAST || dts == AIUIConstant::DTS_ONE_BLOCK) {
                            mFs.close();
                            resultmsg.type = "tts";
                            aiuiResult.publish(resultmsg);
                            ttsend = 1;
                        }
                    }
                }
            } else if (sub == "iat") {
                // 语音识别结果
                string resultStr = string(buffer, dataLen);     // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾
                Json::Value resultJson;
                if (reader.parse(resultStr, resultJson, false)) {
                    Json::Value textJson = resultJson["text"];
                    bool isWpgs = false;
                    if (textJson.isMember("pgs")) {
                        isWpgs = true;
                    }
                    if (isWpgs) {
                        mIatTextBuffer = IatResultUtil::parsePgsIatText(textJson);
                    } else {
                        // 结果拼接起来
                        mIatTextBuffer.append(IatResultUtil::parseIatResult(textJson));
                    }

                    // 是否是该次会话最后一个识别结果
                    bool isLast = textJson["ls"].asBool();
                    if (isLast) {
                        cout << "params: " << params.asString() << endl;
                        cout << "iat: " << mIatTextBuffer << endl;
                        mIatTextBuffer.clear();
                    }
                }
            } else if (sub == "nlp") {
                // 语义理解结果
                // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾
                string resultStr = string(buffer, dataLen);

                // 从说完话到语义结果返回的时长
                long eosRsltTime = event.getData()->getLong("eos_rslt", -1);
                Json::Value resultJson;
                if (reader.parse(resultStr, resultJson, false)) {
                    // 判断是否为有效结果
                    if (resultJson.isMember("intent") &&
                        resultJson["intent"].isMember("rc")) {
                        // AIUI v1的语义结果
                        Json::Value intentJson = resultJson["intent"];
                    } else if (resultJson.isMember("nlp")) {
                        // AIUI v2的语义结果
                        Json::Value nlpJson = resultJson["nlp"];
                        string text = nlpJson["text"].asString();

                        if (text.find("{\"intent\":") == 0) {
                            // 通用语义结果
                            Json::Value textJson;
                            if (reader.parse(text, textJson, false)) {
                                Json::Value intentJson = textJson["intent"];
                                // processIntentJson(params, intentJson, resultStr, eosRsltTime, sid);
                            }
                        } else {
                            // 大模型语义结果
                            // 流式nlp结果里面有seq和status字段
                            int seq = nlpJson["seq"].asInt();
                            int status = nlpJson["status"].asInt();

                            if (status == 0) {
                                mStreamTtsIndex = 0;
                            }

                            /* 多意图取最后一次问题的结果进行tts合成 */
                            if (mIntentCnt > 1) {
                                int currentIntentIndex = 0;
                                Json::Value metaNlpJson;
                                Json::Value textJson = resultJson["cbm_meta"].get("text", metaNlpJson);
                                mStreamNlpAnswerBuffer.append(resultJson["nlp"]["text"].asString());
                                if (status == 2) {
                                    cout << "ignore nlp=" << mStreamNlpAnswerBuffer << endl;
                                    resultmsg.anwser = mStreamNlpAnswerBuffer;
                                    resultmsg.type = "nlp";
                                    aiuiResult.publish(resultmsg);
                                    mStreamNlpAnswerBuffer.clear();
                                }
                                if (reader.parse(textJson.asString(), metaNlpJson, false)) {
                                    currentIntentIndex = metaNlpJson["nlp"]["intent"].asInt(); 
                                    if ((mIntentCnt - 1) != currentIntentIndex) {
                                        cout << "ignore nlp:" << resultStr << endl;
                                        return;
                                    }
                                } else {
                                    // cout << "ignore nlp:" << resultStr << endl;
                                    return;
                                }
                            }
                            // 如果使用应用的语义后合成不需要在调用下面的函数否则tts的播报会重复
                            m_pTtsHelper->addText(text, mStreamTtsIndex++, status);
                            // cout << "----------------------------------" << endl;
                            // cout << "params: " << params.asString() << endl;
                            // cout << "nlp: " << resultStr << endl;
                            if (seq == 0) {
                                long eosRsltTime = event.getData()->getLong("eos_rslt", -1);
                                cout << "eos_result=" << eosRsltTime << "ms" << endl;
                            }
                            // cout << "sid=" << sid << endl;
                            // cout << "seq=" << seq << ", status=" << status << ", answer（应答语）: " << text << endl;
                            // cout << "fullAnswer=" << (mStreamNlpAnswerBuffer.append(text)) << endl;
                            mStreamNlpAnswerBuffer.append(text);
                            if (status == 2) {
                                cout << "fullAnswer=" << mStreamNlpAnswerBuffer << endl;
                                resultmsg.anwser = mStreamNlpAnswerBuffer;
                                resultmsg.type = "nlp";
                                aiuiResult.publish(resultmsg);
                                mStreamNlpAnswerBuffer.clear();
                            }
                        }
                    } else {
                        cout << "----------------------------------" << endl;
                        cout << "nlp: " << resultStr << endl;
                        cout << "sid=" << sid << endl;
                    }
                }
            } else if (sub == "cbm_tidy") {
                // 意图拆分的结果
                string intentStr = string(buffer, dataLen); // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾
                Json::Value tmpJson;
                inient_query.clear();
                if (reader.parse(intentStr, tmpJson, false)) {
                    Json::Value intentTextJson = tmpJson["cbm_tidy"]["text"];
                    if (!intentTextJson.empty() &&
                        reader.parse(intentTextJson.asString(), tmpJson, false)) {
                        mIntentCnt = tmpJson["intent"].size();
                        resultmsg.type = "iat";
                        resultmsg.iat = tmpJson["query"].asString();
                        aiuiResult.publish(resultmsg); 
                        for (Json::Value tmpj : tmpJson["intent"]){
                            inient_query[tmpj["value"].asString()] = tmpj["index"];
                        }
                        cout << "cbm_intent_cnt: " << mIntentCnt << " text: " << tmpJson.toString() << endl;
                    }
                }
            } else if (sub == "cbm_semantic"){
                string semanticStr = string(buffer, dataLen);
                Json::Value semJson;
                rei_voice::REIResultNlp nlpmsg;
                if (reader.parse(semanticStr, semJson, false)) {
                    Json::Value semTextJson = semJson["cbm_semantic"]["text"];
                    if (!semTextJson.empty() && reader.parse(semTextJson.asString(),semJson, false)) {
                        cout << "cbm_semantic_cnt: " << mIntentCnt << " text: " << semJson.toString() << endl;
                        Json::Value semmsgs;
                        nlpmsg.query = semJson["text"].asString();
                        nlpmsg.index = inient_query[semJson["text"].asString()].asUInt();
                        if (!semJson["semantic"].empty() && reader.parse(semJson["semantic"].asString(),semmsgs, false)){
                            nlpmsg.name = semmsgs[0]["intent"].asString();
                            cout << "-------------------------" << endl;
                            for (Json::Value smsg : semmsgs[0]["slots"]){
                                cout << smsg["name"].asString() << ":" << smsg["normValue"].asString() << "; ";
                                nlpmsg.slots_name.push_back(smsg["name"].asString());
                                nlpmsg.slots_value.push_back(smsg["normValue"].asString());
                            }
                            cout << endl;
                        }
                        resultmsg.intent.push_back(nlpmsg); 
                    }
                }                
            } else {
                // 其他结果
                string resultStr = string(buffer, dataLen);     // 注意：这里不能用string resultStr = buffer，因为buffer不一定以0结尾
                // cout << "其他结果" << endl;
                // cout << sub << ": " << event.getInfo() << endl << resultStr << endl;
            }
        } break;

        // 与CMD命令对应的返回结果，arg1为CMD类型，arg2为错误码
        case AIUIConstant::EVENT_CMD_RETURN: {
            if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1()) {
                // 语法构建命令的结果
                // 注：需要集成本地esr引擎才能构建语法
                if (event.getArg2() == 0) {
                    cout << "build grammar success." << endl;
                } else {
                    cout << "build grammar, error=" << event.getArg2() << ", des=" << event.getInfo() << endl;
                }
            } else if (AIUIConstant::CMD_UPDATE_LOCAL_LEXICON == event.getArg1()) {
                // 更新本地语法槽的结果
                if (event.getArg2() == 0) {
                    cout << "update lexicon success" << endl;
                } else {
                    cout << "update lexicon, error=" << event.getArg2() << "des=" << event.getInfo() << endl;
                }
            } else if (AIUIConstant::CMD_SYNC == event.getArg1()) {
                // 数据同步的返回
                int dtype = event.getData()->getInt("sync_dtype", -1);
                int retCode = event.getArg2();
                string dataTypeStr;
                string text;
                if (dtype == AIUIConstant::SYNC_DATA_UPLOAD) {
                    dataTypeStr = "上传实体";
                } else if (dtype == AIUIConstant::SYNC_DATA_DELETE) {
                    dataTypeStr = "删除实体";
                } else if (dtype == AIUIConstant::SYNC_DATA_DOWNLOAD) {
                    dataTypeStr = "下载实体";
                } else if (dtype == AIUIConstant::SYNC_DATA_SEE_SAY) {
                    dataTypeStr = "所见即可说";
                }

                if (AIUIConstant::SUCCESS == retCode ) {
                    // 上传成功，会话的唯一id，用于反馈问题的日志索引字段，注意留存
                    // 注：上传成功立即生效
                    gSyncSid = event.getData()->getString("sid", "");
                    // 获取上传调用时设置的自定义tag
                    string tag = event.getData()->getString("tag", "");
                    // 获取上传调用耗时，单位：ms
                    long timeSpent = event.getData()->getLong("time_spent", -1);
                    cout << "同步" << dataTypeStr << "成功"
                            << "，耗时：" << timeSpent
                            << "ms, sid=" + gSyncSid + "，tag=" + tag;
                    if (dtype == AIUIConstant::SYNC_DATA_UPLOAD) {
                        cout << "，你可以开始尝试了" << endl;
                    } else {
                        cout << endl;
                    }
                    // 实体内容
                    if (dtype == AIUIConstant::SYNC_DATA_DOWNLOAD) {
                        text = event.getData()->getString("text", "");
                        cout << "下载的实体内容:\n" << Base64Util::decode(text) << endl;
                    }
                } else {
                    gSyncSid = "";
                    string result = event.getData()->getString("result", "");
                    cout << "同步" << dataTypeStr << "失败，错误码：" <<
                        retCode << " info:" << event.getInfo() << " result:" << result << endl;
                }
            }
        } break;

        // 开始录音事件
        case AIUIConstant::EVENT_START_RECORD: {
            cout << "EVENT_START_RECORD " << endl;
            ROS_INFO("开始录音");
        } break;

        // 停止录音事件
        case AIUIConstant::EVENT_STOP_RECORD: {
            cout << "EVENT_STOP_RECORD " << endl;
            ROS_INFO("停止录音");
        } break;

        // 出错事件
        case AIUIConstant::EVENT_ERROR: {
            // 打印错误码和描述信息
            cout << "EVENT_ERROR: error=" << event.getArg1() << ", des=" << event.getInfo() << endl;
        } break;
        // 连接到服务器
        case AIUIConstant::EVENT_CONNECTED_TO_SERVER: {
            // 获取uid（为客户端在云端的唯一标识）并打印
            string uid = event.getData()->getString("uid", "");
            // cout << "EVENT_CONNECTED_TO_SERVER, uid=" << uid << endl;
            cout << "连接到服务器, uid=" << uid << endl;
        } break;
        // 与服务器断开连接
        case AIUIConstant::EVENT_SERVER_DISCONNECTED: {
            cout << "与服务器断开连接 " << endl;
        } break;

        //唤醒词操作的结果
        case AIUIConstant::EVENT_CAE_WAKEUP_WORD_RESULT: {
            int type = event.getArg1();
            int ret = event.getArg2();

            //保存唤醒词资源
            if (0 == ret && (type == AIUIConstant::CAE_GEN_WAKEUP_WORD ||
                                type == AIUIConstant::CAE_ADD_WAKEUP_WORD)) {
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary("data", &dataLen);
                fstream fs;
                fs.open("./wakeup_word_res.bin", ios::binary | ios::out);
                fs.write(buffer, dataLen);
                fs.close();
            }

            if (ret != 0) {
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary("error_msg", &dataLen);
                // 注意：这里不能用string errorMsg = buffer，因为buffer不一定以0结尾
                string errorMsg = string(buffer, dataLen);
                cout << "wakeup word operate fail, type = " << type << ", error code = " << ret
                        << ", error msg: " << errorMsg << endl;
            } else {
                cout << "wakeup word operate success, type = " << type << endl;
            }

        } break;

        //唤醒音频事件
        case AIUIConstant::EVENT_CAE_WAKEUP_AUDIO: {
            int type = event.getArg1();
            //保存唤醒音频
            if (type == AIUIConstant::CAE_WAKEUP_AUDIO) {;
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary("audio", &dataLen);
                fstream fs;
                fs.open("wakeup_audio.pcm", ios::binary | ios::out);
                fs.write(buffer, dataLen);
                fs.close();
            }
        } break;
    };
}

ReiTalker m_talker;

bool handleSendEmailCallback(rei_voice::SendEmail::Request& req, rei_voice::SendEmail::Response& res){
    string email = req.email;
    auto r = send_email_code(email.c_str());
    cout << "[" << r.code << "] " << r.msg << endl;
    if (r.code == 0){
        res.success = true;
    }
    res.message = r.msg.c_str();
    return true;
}

bool handleRegisterCallback(rei_voice::UserData::Request& req, rei_voice::UserData::Response& res){
    string email = req.email;
    string password = req.password;
    string code = req.code;
    auto r = user_register(email.c_str(), password.c_str(),code.c_str());
    cout << "[" << r.code << "] " << r.msg << endl;
    if (r.code == 0){
        res.success = true;
    }
    res.message = r.msg.c_str();
    return true;
}

bool handleTTSCallback(rei_voice::REITts::Request& req, rei_voice::REITts::Response& res){
    ROS_INFO("开始TTS合成: %s", req.text.c_str());
    m_talker.startTTS(req.text, req.is_play);
    res.success = true;
    res.message = "TTS synthesis has been initiated";
    res.filePath = TTS_PATH;
    return true;
}

bool handlePlayCallback(rei_voice::REIPlayer::Request& req, rei_voice::REIPlayer::Response& res){
    if(playLocalPcmFile(req.PcmPath) != 0){
        res.success = false;
        res.message = "pcm audio playback failed";
        return true;
    }
    res.success = true;
    res.message = "pcm audio playback successful";
    return true;
}

bool handleNLPCallback(rei_voice::REINlp::Request& req, rei_voice::REINlp::Response& res){
    m_talker.writeText(req.text,false);
    res.success = true;
    return true;
}

bool handleUpSyncCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    m_talker.syncSchemaData("pos_name",AIUIConstant::SYNC_DATA_UPLOAD);
    sleep(1);
    m_talker.syncSchemaData("object_name",AIUIConstant::SYNC_DATA_UPLOAD);
    sleep(1);
    m_talker.syncSchemaData("pos_name",AIUIConstant::SYNC_DATA_DOWNLOAD);
    sleep(1);
    m_talker.syncSchemaData("object_name",AIUIConstant::SYNC_DATA_DOWNLOAD);
    return true;
}

bool handleRecordAudio(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (req.data) {
        m_talker.startRecordAudio();
        ROS_INFO("开始录音");
    }else{
        m_talker.stopRecordAudio();
        ROS_INFO("结束录音");
    }
    res.success = true;
    return true;
}

bool handleLogin(rei_voice::UserData::Request& req, rei_voice::UserData::Response& res) {
    string email = req.email;
    string password = req.password;
    string code = req.code;
    auto r = user_login(email.c_str(), password.c_str(),code.c_str());
    cout << "[" << r.code << "] " << r.msg << endl;
    if (r.code == 0){
        res.success = true;
    }
    res.message = r.msg.c_str();
    return true;
}

int main(int argc,char* argv[]){
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc,argv,"rei_voiceui_node");
    ros::NodeHandle nh;
    tts_service = nh.advertiseService("/REIService/voice_tts", handleTTSCallback);
    playPcm_service = nh.advertiseService("/REIService/PcmPlayer", handlePlayCallback);
    nlp_service = nh.advertiseService("/REIService/voice_nlp", handleNLPCallback);
    upSync_service = nh.advertiseService("/REIService/upSync", handleUpSyncCallback);
    recordAudio_service = nh.advertiseService("/REIService/RecordAudio", handleRecordAudio);
    login_service = nh.advertiseService("REIService/Login",handleLogin);
    register_service = nh.advertiseService("REIService/Register",handleRegisterCallback);
    send_email_service = nh.advertiseService("REIService/SendEmail",handleSendEmailCallback);
    aiuiState = nh.advertise<std_msgs::String>("/REITopic/AIUIState", 10);
    aiuiResult = nh.advertise<rei_voice::REIResult>("/REITopic/AIUIResult", 10);
    ros::spin();
    return 0;
}
