//
// Created by hj on 2023/2/9.
//

#ifndef AIUI_SDK_PCMPLAYER_H
#define AIUI_SDK_PCMPLAYER_H

#include "AIUICommon.h"

namespace aiui {

/**
 * 基于PortAudio实现的PCM音频播放器。
 *
 * 注意：暂时只支持Linux和Windows平台。
 */
class PcmPlayer
{
public:
    /**
     * 播放器监听器。
     *
     * 注意：在回调函数中不要做耗时操作。
     */
    class PlayerListener
    {
    public:
        /**
         * 已开启回调，开启之后才可以写数据。
         */
        virtual void onStarted() = 0;

        /**
         * 暂停回调。
         */
        virtual void onPaused() = 0;

        /**
         * 暂停后再恢复的回调。
         */
        virtual void onResumed() = 0;

        /**
         * 已停止回调。
         */
        virtual void onStopped() = 0;

        /**
         * 播放进度回调。
         *
         * @param streamId 流标识，在write时传入
         * @param progress 进度，在write时随音频传入，有可能多次回调具有相同的值
         * @param audio PCM音频数据
         * @param len 数据长度
         * @param isCompleted 音频是否播放完成
         */
        virtual void onPlayProgress(int streamId, int progress, const char* audio, int len,  bool isCompleted) = 0;

        /**
         * 出错回调。
         *
         * @param error 错误码
         * @param des 描述信息
         */
        virtual void onError(int error, const char* des) = 0;
    };

public:
    /**
     * PCM格式，32位float型编码。
     */
    static const int PCM_FORMAT_FLOAT32 = 0x00000001;

    /**
     * PCM格式，32位int型编码。
     */
    static const int PCM_FORMAT_INT32 = 0x00000002;

    /**
     * PCM格式，24位int型编码。
     */
    static const int PCM_FORMAT_INT24 = 0x00000004;

    /**
     * PCM格式，16位int型编码。
     */
    static const int PCM_FORMAT_INT16 = 0x00000008;

    /**
     * 未初始化，即最初状态。
     */
    static const int STATE_NOT_INITED = 1;

    /**
     * 已初始化状态。
     */
    static const int STATE_INITED = 2;

    /**
     * 已开启状态（开启后可写入音频播放），并不等于已开始播放。关于是否正在播放可由外部用一个标记isPlaying实现：
     * 当有进度回调时设置isPlaying=true，进度回调结束时，设置isPlaying=false。
     */
    static const int STATE_STARTED = 3;

    /**
     * 暂停状态。
     */
    static const int STATE_PAUSED = 4;

    /**
     * 停止状态。
     */
    static const int STATE_STOPPED = 5;

    /**
     * 销毁状态。
     */
    static const int STATE_DESTROYED = 6;

    /**
     * 数据状态，第一块数据。
     */
    static const int DTS_BLOCK_FIRST = 0;

    /**
     * 数据状态，后续数据。
     */
    static const int DTS_BLOCK_FOLLOW = 1;

    /**
     * 数据状态，最后一块数据。
     */
    static const int DTS_BLOCK_LAST = 2;

    /**
     * 数据状态，总共一块数据。
     */
    static const int DTS_ONE_BLOCK = 3;

    static const int FRAME_TIME_LEN_MS = 20;

private:
    void* m_pCtx;

public:
    /**
     * 构造函数。当innerPlay设置为false时，需要外部在onPlayProgress回调中拿到PCM数据自己调用第三方播放器播放。
     *
     * @param innerPlay 是否由SDK内部播放（即将PCM数据写入底层）
     */
    AIUIEXPORT explicit PcmPlayer(bool innerPlay = true);

    AIUIEXPORT ~PcmPlayer();

    /**
     * 获取输出设备数。
     *
     * @return
     */
    AIUIEXPORT int getOutputDeviceCount();

    /**
     * 获取设备名称。
     *
     * @param index 索引号，取值：0到getOutputDeviceCount() - 1
     * @return 设备名称
     */
    AIUIEXPORT const char* getDeviceName(int index);

    /**
     * 获取状态。
     *
     * @return 状态
     */
    AIUIEXPORT int getState();

    /**
     * 设置监听器。
     *
     * @param listener
     */
    AIUIEXPORT void setListener(PlayerListener* listener);

    /**
     * 初始化播放器。
     *
     * @param outDevIndex 输出设备索引号，默认值：-1（表示使用默认输出设备）
     * @param channelCount 通道数
     * @param pcmFormat PCM格式，默认为16位编码的整型
     * @param sampleRate 采样率，默认为16k
     * @return 0表示成功，否则为错误码
     */
    AIUIEXPORT int init(int outDevIndex = -1,
                        int channelCount = 1,
                        int pcmFormat = PcmPlayer::PCM_FORMAT_INT16,
                        int sampleRate = 16000);

    /**
     * 开启播放，开启后才能调用write方法写入音频。
     *
     * @return
     */
    AIUIEXPORT int start();

    /**
     * 写入音频。
     *
     * 注意：调用start()后才能写入音频。
     *
     * @param streamId 音频流标识
     * @param pcm 音频数据
     * @param len 数据长度
     * @param dts 数据状态，取值：DTS_BLOCK_FIRST（第一块数据），DTS_BLOCK_FOLLOW（中间数据），DTS_BLOCK_LAST（最后一块数据），DTS_ONE_BLOCK（总共一块数据）
     * @param progress 进度，取值：0-100（0表示开始，100表示结束）
     * @return
     */
    AIUIEXPORT int write(int streamId, const char* pcm, int len, int dts, int progress);

    /**
     * 暂停播放，暂停之后通过resume()方法恢复。
     *
     * @return 0表示成功，否则为错误码
     */
    AIUIEXPORT int pause();

    /**
     * 用于在暂停播放后恢复播放。
     *
     * @return 0表示成功，否则为错误码
     */
    AIUIEXPORT int resume();

    /**
     * 消除缓存队列中的音频。
     *
     * @return 0表示成功，否则为错误码
     */
    AIUIEXPORT int clear();

    /**
     * 停止播放，清除缓存队列中的音频。
     *
     * @return 0表示成功，否则为错误码
     */
    AIUIEXPORT int stop();

    /**
     * 销毁播放器，销毁之后若要重新播放，请新建一个Player对象。
     *
     * 注意：该方法会释放内部资源，但并不会释放Player对象所占内存（如果是new的对象，别忘记delete）。
     */
    AIUIEXPORT void destroy();

private:
    static int audioCallback(const void *input, void *output,
                      unsigned long frameCount,
                      const void* timeInfo,
                      unsigned long statusFlags,
                      void *userData);
};

}    // namespace aiui

#endif    //AIUI_SDK_PCMPLAYER_H
