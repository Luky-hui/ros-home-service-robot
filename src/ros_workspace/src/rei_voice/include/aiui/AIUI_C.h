/**
* AIUI_C.h
*
*  Created on: 2020年06月26日
*      Author: AIUI开放平台（https://aiui.xfyun.cn/）
*/

#ifndef AIUI_SRC_AIUI_C_H
#define AIUI_SRC_AIUI_C_H

#include "AIUICommon.h"

#include <stddef.h>

/** AIUI纯C接口，是AIUI.h中接口的C语言封装 **/

/* clang-format off */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * 获取AIUI版本信息。
 *
 * @return
 */
const char AIUIEXPORT * AIUIAPI aiui_get_version();

/*******************************AIUIDataBundle********************************/
/**
 * 数据捆绑对象指针。支持int、long、string和Buffer*类型数据按名存取。
 */
typedef void* AIUIDataBundle;

/**
 * 从db中按key获取int类型值，若不存在则返回默认值。
 *
 * @param db AIUIDataBundle指针
 * @param key 变量名称
 * @param defaultVal 默认值
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_db_int(AIUIDataBundle db, const char* key, int defaultVal);

/**
 * 从db中按key获取long类型值，若不存在则返回默认值。
 *
 * @param db AIUIDataBundle指针
 * @param key 变量名称
 * @param defaultVal 默认值
 * @return
 */
long AIUIEXPORT AIUIAPI aiui_db_long(AIUIDataBundle db, const char* key, long defaultVal);

/**
 * 从db中按key获取字符串类型值，若不存在则返回默认值。
 *
 * @param db AIUIDataBundle指针
 * @param key 变量名称
 * @param defaultVal 默认值
 * @return
 */
const char AIUIEXPORT * AIUIAPI aiui_db_string(AIUIDataBundle db, const char* key, const char* defaultVal);

/**
 * 从db中按key获取buffer，若不存在则返回NULL。
 *
 * @param db AIUIDataBundle指针
 * @param key 变量名称
 * @param dataLen [OUT]用于返回buffer长度
 * @return
 */
const char AIUIEXPORT * AIUIAPI aiui_db_binary(AIUIDataBundle db, const char* key, int* dataLen);

/*******************************AIUIEvent*********************************/
/**
 * AIUI事件对象指针。业务结果、SDK内部状态变化等输出信息都通过事件抛出。
 */
typedef const void* AIUIEvent;

/**
 * 事件回调函数定义。
 */
typedef void (*AIUIMessageCallback)(AIUIEvent event, void *data);

/**
 * 从ae中获取事件类型eventType成员变量。取值参见AIUIConstant中EVENT_开头的常量定义。
 *
 * @param event
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_event_type(AIUIEvent event);

/**
 * 从ae中获取扩展参数arg1成员变量。
 *
 * @param event
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_event_arg1(AIUIEvent event);

/**
 * 从ae中获取扩展参数arg2成员变量。
 *
 * @param event
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_event_arg2(AIUIEvent event);

/**
 * 从ae中获取info成员变量。
 *
 * @param event
 * @return
 */
const char AIUIEXPORT * AIUIAPI aiui_event_info(AIUIEvent event);

/**
 * 从ae中获取data成员变量。
 *
 * @param event
 * @return
 */
AIUIDataBundle AIUIEXPORT AIUIAPI aiui_event_databundle(AIUIEvent event);

int AIUIEXPORT AIUIAPI aiui_strlen(const char * str);

/**********************************AIUIBuffer***********************************/
/**
 * buffer对象指针，用来存取一段二进制数据。
 */
typedef void* AIUIBuffer;

/**
 * 销毁buffer对象。
 *
 * @param buffer
 */
void AIUIEXPORT AIUIAPI aiui_buffer_destroy(AIUIBuffer buffer);

/**
 * 创建buffer对象，存入data。
 *
 * @param data 数据
 * @param len 长度
 * @return
 */
AIUIBuffer AIUIEXPORT AIUIAPI aiui_create_buffer_from_data(const void* data, size_t len);
/******************************************************************************/

/**********************************AIUIMessage**********************************/
/**
 * AIUI消息对象指针。AIUI所有的输入都是通过消息发送到SDK内部。
 */
typedef void* AIUIMessage;

/**
 * 创建消息。
 *
 * @param msgType
 * @return
 */
AIUIMessage AIUIEXPORT AIUIAPI aiui_msg_create1(int msgType);
AIUIMessage AIUIEXPORT AIUIAPI aiui_msg_create2(int msgType, int arg1);
AIUIMessage AIUIEXPORT AIUIAPI aiui_msg_create3(int msgType, int arg1, int arg2);
AIUIMessage AIUIEXPORT AIUIAPI aiui_msg_create4(int msgType, int arg1, int arg2, const char* params);
AIUIMessage AIUIEXPORT AIUIAPI aiui_msg_create(int msgType, int arg1, int arg2, const char* params, AIUIBuffer data);

void AIUIEXPORT AIUIAPI aiui_msg_destroy(AIUIMessage msg);

/***********************************AIUIAgent************************************/
/**
 * AIUI代理单例对象指针，应用通过代理对象与AIUI交互。
 */
typedef void* AIUIAgent;

/**
 * 创建agent。
 *
 * @param params 参数配置，即aiui.cfg文件的内容
 * @param callback 回调函数
 * @param data 用户自定义数据，将传递给回调函数
 * @return
 */
AIUIAgent AIUIEXPORT AIUIAPI aiui_agent_create(const char* params, AIUIMessageCallback callback, void *data);

/**
 * 向agent发送消息。
 *
 * @param agent 代理对象
 * @param msg 消息
 */
void AIUIEXPORT AIUIAPI aiui_agent_send_message(AIUIAgent agent, AIUIMessage msg);

/**
 * 销毁agent对象。
 *
 * @param agent 代理对象
 */
void AIUIEXPORT AIUIAPI aiui_agent_destroy(AIUIAgent agent);

/***********************************AIUISetting**********************************/
/**
 * AIUI配置相关方法。
 *
 * 注意：配置方法需要在aiui_agent_create之前调用。
 */
typedef enum {
    aiui_debug, // 输出详细信息，调试的时候使用
    aiui_info,
    aiui_warn,
    aiui_error,
    aiui_none   // 关闭日志输出
} LogLevel;

typedef enum  {
   AIUI_INTELLIGENT_HDW,    // 智能硬件版本
   AIUI_MOBILE_PHONE,        // 移动互联网版本
   AIUI_DESKTOP_PC           // 桌面PC版本
} VersionType;

/**
 * 设置AIUI工作目录，SDK会在该路径下保存日志、数据等文件。
 *
 * 默认目录：1）Android平台为/sdcard/AIUI/；
 *           2）iOS平台为应用沙盒的AIUI文件夹；
 *           3）Win和Linux平台为应用当前目录的AIUI文件夹。
 *
 * @param szDir 路径（windows下以"\\"结尾，其他平台以"/"结尾），不能为空
 * @return 0表示成功，否则失败
 */
int AIUIEXPORT AIUIAPI aiui_set_aiui_dir(const char* szDir);

/**
 * 获取AIUI目录。
 *
 * @return
 */
const char AIUIEXPORT * AIUIAPI aiui_get_aiui_dir();

/**
 * 设置MSC工作目录，SDK会从该路径下读取基本配置，保存云端交互日志aiui.log。
 *
 * @param szDir 路径（windows下以"\\"结尾，其他平台以"/"结尾），不能为空
 * @return 0表示成功，否则失败
 */
int AIUIEXPORT AIUIAPI aiui_set_msc_dir(const char* szDir);

/**
 * 设置msc.cfg的内容到SDK中。
 *
 * @param szCfg msc.cfg中的实际内容
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_set_msc_cfg(const char* szCfg);

/**
 * 初始化日志记录器，设置日志保存目录。一般不需要调用。日志默认保存在AIUI/log/下。
 *
 * @param szLogDir 日志目录（windows下以"\\"结尾，其他平台以"/"结尾），为空则在AIUI工作目录下创建log目录
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_init_logger(const char* szLogDir);

/**
 * 设置上层调试日志打印级别，默认级别为info。
 *
 * 注：
 * 		1.Android上通过logcat输出调试日志；
 * 		2.iOS上通过控制台输出调试日志；
 * 		3.Linux/Windows上暂无调试日志输出。
 *
 * @param level
 */
void AIUIEXPORT AIUIAPI aiui_set_log_level(LogLevel level);

/**
 * 设置底层网络交互日志打印级别。
 *
 * 注：网络交互日志默认打印到aiui.log文件。
 *
 * @param level
 */
void AIUIEXPORT AIUIAPI aiui_set_net_log_level(LogLevel level);

/**
 * 设置是否保存数据日志，即输入的音频和云端返回的结果。
 *
 * @param save 是否保存
 * @param logSizeMB 日志大小
 */
void AIUIEXPORT AIUIAPI aiui_set_save_data_log(int save, int logSizeMB);

/**
 * 设置数据日志保存目录，不设置则默认是在AIUI工作目录下的data目录。
 *
 * @param szDir 目录（windows下以"\\"结尾，其他平台以"/"结尾）
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_set_data_log_dir(const char* szDir);

/**
 * 设置原始音频保存目录，不设置则默认是在AIUI工作目录下的audio/raw/目录。
 *
 * @param szDir 目录（windows下以"\\"结尾，其他平台以"/"结尾）
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_set_raw_audio_dir(const char* szDir);

/**
 * 是否为移动互联网版本。
 *
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_is_mobile_version();

/**
 * 获取版本类型。
 *
 * @return
 */
VersionType AIUIEXPORT AIUIAPI aiui_get_version_type();

/**
 * 设置系统信息，如net.mac，sn等。
 *
 * @param key
 * @param val
 */
void AIUIEXPORT AIUIAPI aiui_set_system_info(const char* key, const char* val);

/**
 * 获取系统信息。
 *
 * @param key
 * @param out
 * @param maxSize
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_get_system_info(const char * key, char *out, int maxSize);

/**
 * 设置GPS坐标。参数优先级：CMD_WRITE消息params参数中携带 > aiui.cfg的audioparams中设置 > aiui_set_gps_pos方法设置。
 *
 * @param lng 经度
 * @param lat 纬度
 */
void AIUIEXPORT AIUIAPI aiui_set_gps_pos(float lng, float lat);

#ifdef __cplusplus
};
#endif

/* clang-format on */

#endif    //AIUI_SRC_AIUI_C_H
