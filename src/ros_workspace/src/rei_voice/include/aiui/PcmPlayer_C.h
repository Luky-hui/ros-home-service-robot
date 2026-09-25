//
// Created by hj on 2023/2/9.
//

#ifndef AIUI_SDK_PCMPLAYER_C_H
#define AIUI_SDK_PCMPLAYER_C_H

#include "AIUICommon.h"

/** PcmPlayer的C语言接口 **/

/* clang-format off */
#ifdef __cplusplus
extern "C" {
#endif

/***********************************PcmPlayer**********************************/
/**
 * 回调函数类型定义。
 */
typedef void (*pcm_player_onstarted_cb)();
typedef void (*pcm_player_onpaused_cb)();
typedef void (*pcm_player_onresumed_cb)();
typedef void (*pcm_player_onstopped_cb)();
typedef void (*pcm_player_onplayprogress_cb)(int streamId, int progress, const char* audio, int len, bool isCompleted);
typedef void (*pcm_player_onerror_cb)(int error, const char* des);

/**
 * 播放器状态，分别表示未初始化，已初始化，已开启，暂停，停止和已销毁。
 *
 * 关于是否正在播放可由外部用一个标记isPlaying实现：当有进度回调时设置isPlaying=true，
 * 进度回调结束时，设置isPlaying=false。
 */
#define PCM_PLAYER_STATE_NOT_INITED 1
#define PCM_PLAYER_STATE_INITED 2
#define PCM_PLAYER_STATE_STARTED 3
#define PCM_PLAYER_STATE_PAUSED 4
#define PCM_PLAYER_STATE_STOPPED 5
#define PCM_PLAYER_STATE_DESTROYED 6

/**
 * 数据状态，第一块音频。在write音频时传入。
 */
#define PCM_PLAYER_DTS_BLOCK_FIRST 0

/**
 * 数据状态，中间音频。在write音频时传入。
 */
#define PCM_PLAYER_DTS_BLOCK_FOLLOW 1

/**
 * 数据状态，最后一块音频。在write音频时传入。
 */
#define PCM_PLAYER_DTS_BLOCK_LAST 2

/**
 * 数据状态，总共就一块音频。在write音频时传入。
 */
#define PCM_PLAYER_DTS_ONE_BLOCK 3

/**
 * 创建播放器。当innerPlay设置为false时，需求外部在onPlayProgressCb回调中拿到PCM数据自己调用第三方播放器播放。
 *
 * @param innerPlay 是否由SDK内部播放（即将PCM数据写入底层）
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_create(bool innerPlay = true);

/**
 * 获取可用的音频输出设备数。
 *
 * @return 设备数，-1表示出错
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_get_output_device_count();

/**
 * 获取设备名称。
 *
 * @param devIndex 设备索引号，取值：0到设备数-1
 * @return 名称字符串，null表示出错
 */
const char AIUIEXPORT * AIUIAPI aiui_pcm_player_get_device_name(int devIndex);

/**
 * 初始化播放器。
 *
 * @param devIndex 输出设备索引号
 * @param sampleRate 采样率
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_init(int devIndex = -1, int sampleRate = 16000);

/**
 * 设置回调函数。
 *
 * @param onStartCb
 * @param onPausedCb
 * @param onResumedCb
 * @param onStoppedCb
 * @param onPlayProgressCb
 * @param onErrorCb
 */
void AIUIEXPORT AIUIAPI aiui_pcm_player_set_callbacks(pcm_player_onstarted_cb onStartCb,
                                                      pcm_player_onpaused_cb onPausedCb,
                                                      pcm_player_onresumed_cb onResumedCb,
                                                      pcm_player_onstopped_cb onStoppedCb,
                                                      pcm_player_onplayprogress_cb onPlayProgressCb,
                                                      pcm_player_onerror_cb onErrorCb);

/**
 * 获取播放器状态。
 *
 * @return
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_get_state();

/**
 * 开启播放器，开启成功后才可以写入音频播放。
 *
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_start();

/**
 * 写入音频进行播放。
 * @param streamId 音频流标识
 * @param pcm 音频数据
 * @param len 数据长度，单位:字节
 *  注意:len <=0会返回 10107 不会有这次音频的回调,如果还想要回调就自己构造一段音频数据，音频数据内容全部为0建议长度为2
 * @param dts 数据状态，即PCM_PLAYER_DTS_XXX注
 *  注意1:dtS == PCM_PLAYER_DTS_BLOCK_FIRST || dtS ==PCM_PLAYER_DTS_ONE_BLOCK时，会将之前写入的音频数据给清除掉
 *  注意2:当这个写入的函数返回是0，当dtS=PCM_PLAYER_DTS_BLOCK_LAST || dtS == PCM_PLAYER_DTS_ONE_BLOCK时,
 *        如果有设置onPlayProgressCb这个回调当播放完这次写入的音频，回调中的isCompleted将会是true
 * @param progress 数据进度，取值:0-100
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_write(int streamId, const char* pcm, int len, int dts, int progress);

/**
 * 暂停播放。
 *
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_pause();

/**
 * 恢复播放。
 *
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_resume();

/**
 * 消除队列中的音频。
 *
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_clear();

/**
 * 停止播放。
 *
 * @return 0表示成功，否则为错误码
 */
int AIUIEXPORT AIUIAPI aiui_pcm_player_stop();

/**
 * 销毁播放器。
 *
 * @return 0表示成功，否则为错误码
 */
void AIUIEXPORT AIUIAPI aiui_pcm_player_destroy();

#ifdef __cplusplus
};
#endif

/* clang-format on */

#endif    //AIUI_SDK_PCMPLAYER_C_H
