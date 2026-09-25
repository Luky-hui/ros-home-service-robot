//
// Created by hj on 2023/6/5.
//

#ifndef AIUI_SDK_STREAMNLPTTSHELPER_H
#define AIUI_SDK_STREAMNLPTTSHELPER_H

#include <string>
#include <chrono>
#include <utility>
#include <vector>
#include <memory>
#include <regex>

#if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
    #include "ConvertUtil.h"
#else
    #include <codecvt>
#endif

#include "json/json.h"

using namespace aiui_va;

/**
 * 流式语义结果合成帮助类。
 */
class StreamNlpTtsHelper
{
public:
    static const int STATUS_BEGIN = 0;

    static const int STATUS_CONTINUE = 1;

    static const int STATUS_END = 2;

    static const int STATUS_ALLONE = 3;

private:
    static const std::wstring REGEX_SENTENCE_DIVIDER;

    class InTextSeg {
    public:
        // 都得用宽字符
        std::wstring mText;

        int mIndex;

        int mStatus;

    public:
        InTextSeg(const std::string& text, int index, int status) {
    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
            mText = ConvertUtil::utf8ToWstring(text);
    #else
            std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
            mText = converter.from_bytes(text);
    #endif
            mIndex = index;
            mStatus = status;
        }

        int getTextLen() const {
            return mText.length();
        }

        bool isBegin() const {
            return mStatus == STATUS_BEGIN;
        }

        bool isEmpty() const {
            return mText.empty();
        }

        bool isEnd() const {
            return mStatus == STATUS_END;
        }
    };

public:
    class OutTextSeg {
    public:
        std::string mTag;

        int mIndex{};

        std::string mText;

        int mStatus{};

        int mOffset{};

    public:
        OutTextSeg() = default;

        OutTextSeg(int index, const std::string& text, int status, int offset) {
            long long timeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
            mTag = "stream_nlp_tts-" + ConvertUtil::toString(timeMs) + "-" + ConvertUtil::toString(index);
    #else
            mTag = "stream_nlp_tts-" + std::to_string(timeMs) + "-" + std::to_string(index);
    #endif

            mIndex = index;
            mText = text;
            mStatus = status;
            mOffset = offset;
        }

        int getTextLen() const {
            return mText.length();
        }

        std::string getTag() const {
            return mTag;
        }

        bool isBegin() const {
            return mStatus == STATUS_BEGIN;
        }

        bool isEmpty() const {
            return mText.empty();
        }

        bool isEnd() const {
            return mStatus == STATUS_END;
        }

        std::string toString() const {
    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
            return std::string("OutTextSeg{") +
                   "mTag='" + mTag +
                   ", mIndex=" + ConvertUtil::toString(mIndex) +
                   ", mText='" + mText +
                   ", mStatus=" + ConvertUtil::toString(mStatus) +
                   ", mOffset=" + ConvertUtil::toString(mOffset) +
                   '}';
    #else
            return std::string("OutTextSeg{") +
                   "mTag='" + mTag +
                   ", mIndex=" + std::to_string(mIndex) +
                   ", mText='" + mText +
                   ", mStatus=" + std::to_string(mStatus) +
                   ", mOffset=" + std::to_string(mOffset) +
                   '}';
    #endif
        }
    };

    class Listener
    {
    public:
        virtual void onText(const OutTextSeg& textSeg) = 0;

        virtual void onTtsData(const Json::Value& bizParamJson, const char* audio, int len) = 0;

        virtual void onFinish(const std::string& fullText) = 0;
    };

private:
#if defined(__ANDROID__) || defined(USE_SELF_CONVERT)

#else
    std::wstring_convert<std::codecvt_utf8<wchar_t>> mStrConverter;
#endif

    std::vector<InTextSeg> mInTextSegList;

    std::vector<std::shared_ptr<OutTextSeg>> mOutTextSegList;

    int mTextMinLimit = 100;

    int mOrderedEndSegIndex = -1;

    int mTotalOrderedTextLen = 0;

    int mFetchedTextLen = 0;

    // 这里要用宽字符
    std::wstring mOrderedTextBuffer;

    std::shared_ptr<Listener> m_pOutListener;

    std::shared_ptr<OutTextSeg> m_pCurOutTextSeg;

    enum FetchStatus {
        INIT,
        STARTED,
        INTERRUPTED
    };

    FetchStatus mFetchStatus = FetchStatus::INIT;

    int mOutTextSegIndex = 0;

    bool mFoundFirstStatusBeg = false;

    int mTtsFrameIndex = 1;

public:
    explicit StreamNlpTtsHelper(std::shared_ptr<Listener> listener) {
        m_pOutListener = std::move(listener);
    }

    void setTextMinLimit(int limit) {
        mTextMinLimit = limit;
    }

    bool isAddCompleted() {
        if (mInTextSegList.empty()) {
            return false;
        }

        int size = mInTextSegList.size();
        const InTextSeg& last = mInTextSegList[size - 1];
        if (last.isEnd() && size == last.mIndex + 1) {
            return true;
        }

        return false;
    }

    /**
     * 添加合成文本。
     *
     * @param text   stream_nlp返回的answer文本
     * @param index  stream_nlp返回的index
     * @param status stream_nlp返回的状态
     */
    void addText(const std::string& text, int index, int status) {
        if (isAddCompleted()) {
            return;
        }

        InTextSeg seg(text, index, status);

        int begin = mInTextSegList.size() - 1;
        int pos = begin;
        while (pos >= 0) {
            InTextSeg cur = mInTextSegList[pos];
            if (index < cur.mIndex) {
                pos--;
            } else {
                break;
            }
        }

        if (pos == begin) {
            // list为空，或者插入位置为尾部
            mInTextSegList.push_back(seg);
        } else {
            auto it = mInTextSegList.begin();
            mInTextSegList.insert(it + pos + 1, seg);
        }

        for (int i = mOrderedEndSegIndex + 1; i < mInTextSegList.size(); i++) {
            InTextSeg cur = mInTextSegList[i];
            if (i == cur.mIndex) {
                mOrderedEndSegIndex = i;
                mTotalOrderedTextLen += cur.getTextLen();

                // 把有序文本段追加到buffer
                mOrderedTextBuffer.append(cur.mText);
            } else {
                break;
            }
        }

        if (mFetchStatus == FetchStatus::INIT || mFetchStatus == FetchStatus::INTERRUPTED) {
            processOrderedText();
        }
    }

    std::shared_ptr<OutTextSeg> fetchOrderedText() {
        bool needFetch = true;
        int tryFetchLen = 0;

        if (isAddCompleted()) {
            // 已经接收完成，取limit和剩余长度的最小值
    #if defined(WIN32) || defined(_WIN64)
            tryFetchLen = (std::min)(mTextMinLimit, mTotalOrderedTextLen - mFetchedTextLen);
    #else
            tryFetchLen = std::min(mTextMinLimit, mTotalOrderedTextLen - mFetchedTextLen);
    #endif
        } else {
            // 没接收完成
            if (mTotalOrderedTextLen - mFetchedTextLen < mTextMinLimit) {
                // 剩余的长度不够，则这次不需要取
                needFetch = false;
            } else {
                // 剩余长度足够，尝试取limit长度
                tryFetchLen = mTextMinLimit;
            }
        }

        if (!needFetch) {
            return nullptr;
        }

        // 取剩余部分（这里向前取一个长度），在里面查找第一个分隔符位置
        std::wstring leftPart = mOrderedTextBuffer.substr(mFetchedTextLen + tryFetchLen - 1);
        std::wregex p(REGEX_SENTENCE_DIVIDER);
        std::wsmatch m;

        bool find = std::regex_search(leftPart, m, p);
        int dividerPos = 0;
        if (find) {
            dividerPos = m.position();
        } else {
            if (!isAddCompleted()) {
                // 没找到且没收完成，不处理
                return nullptr;
            }

            // 已接收完成，取到结尾即可
            dividerPos = leftPart.length() - 1;
        }

        // 得到真实的获取长度和文本
        tryFetchLen += dividerPos;
        std::wstring fetchedText = mOrderedTextBuffer.substr(mFetchedTextLen,
                                                      tryFetchLen);

        int status;
        if (isAddCompleted() && dividerPos == leftPart.length() - 1) {
            // 这一次把文本全取完了
            status = STATUS_END;
        } else {
            if (mOutTextSegList.empty()) {
                status = STATUS_BEGIN;
            } else {
                status = STATUS_CONTINUE;
            }
        }

    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
        std::shared_ptr<OutTextSeg> outTextSeg = std::make_shared<OutTextSeg>(
            mOutTextSegIndex++, ConvertUtil::wstringToUtf8(fetchedText), status, mFetchedTextLen);
    #else
        std::shared_ptr<OutTextSeg> outTextSeg = std::make_shared<OutTextSeg>(
            mOutTextSegIndex++, mStrConverter.to_bytes(fetchedText), status, mFetchedTextLen);
    #endif

        mFetchedTextLen += fetchedText.length();
        mOutTextSegList.push_back(outTextSeg);

        return outTextSeg;
    }

    /**
     * 在AIUI返回合成结果时调用，传入原始合成结果。
     *
     * @param tag          结果中的标签
     * @param bizParamJson 结果描述
     * @param audio        音频数据
     * @param len           音频长度
     */
    void onOriginTtsData(const std::string& tag, Json::Value& bizParamJson, const char* audio, int len) {
        if (m_pCurOutTextSeg == nullptr || m_pCurOutTextSeg->mTag != tag) {
            m_pCurOutTextSeg = findTextSegByTag(tag);
        }

        if (m_pCurOutTextSeg == nullptr) {
            return;
        }

        bool isLastSeg = m_pCurOutTextSeg->isEnd();

        Json::Value& data = bizParamJson["data"][0];
        Json::Value& content = data["content"][0];

        int dts = content["dts"].asInt();
        int originDts = dts;

        // 修正局部文本位置为全局位置
        int text_start = content["text_start"].asInt() + m_pCurOutTextSeg->mOffset;
        int text_end = content["text_end"].asInt() + m_pCurOutTextSeg->mOffset;

        // 修正局部dts为全局dts
        if (dts == STATUS_CONTINUE) {
            // continue状态不用变
        } else {
            if (dts == STATUS_BEGIN) {
                if (!mFoundFirstStatusBeg) {
                    mFoundFirstStatusBeg = true;
                } else {
                    dts = STATUS_CONTINUE;
                }
            } else if (dts == STATUS_END) {
                if (!isLastSeg) {
                    dts = STATUS_CONTINUE;
                }
            } else if (dts == STATUS_ALLONE) {
                if (!mFoundFirstStatusBeg) {
                    mFoundFirstStatusBeg = true;

                    if (!isLastSeg) {
                        dts = STATUS_CONTINUE;
                    }
                } else {
                    if (isLastSeg) {
                        dts = STATUS_END;
                    } else {
                        dts = STATUS_CONTINUE;
                    }
                }
            }
        }

        // 修改局部percent为全局
        int text_percent = content["text_percent"].asInt();
        if (!isAddCompleted()) {
            // 由于文本没有添加完，总长度未定，这里的全局进度算不了，直接取0
            text_percent = 0;
        } else {
            if (text_percent == 100 && isLastSeg) {
                // 最后一个文本的100进度不用变
            } else {
                int localOffset = text_percent * m_pCurOutTextSeg->getTextLen() / 100;
                int globalOffset = m_pCurOutTextSeg->mOffset + localOffset;
                text_percent = (int) (globalOffset * 100 / (float) mTotalOrderedTextLen);
            }
        }

        content["dts"] = dts;
        content["text_start"] = text_start;
        content["text_end"] = text_end;
        content["text_percent"] = text_percent;
        content["frame_id"] = mTtsFrameIndex++;

        if (m_pOutListener != nullptr) {
            m_pOutListener->onTtsData(bizParamJson, audio, len);
        }

        // 这里要用原始的dts来判断
        if (originDts == STATUS_END || originDts == STATUS_ALLONE) {
            if (isLastSeg) {
                // 全部处理完成
                if (m_pOutListener != nullptr) {
    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
                    m_pOutListener->onFinish(ConvertUtil::wstringToUtf8(mOrderedTextBuffer));
    #else
                    m_pOutListener->onFinish(mStrConverter.to_bytes(mOrderedTextBuffer));
    #endif
                }

                clear();
            } else {
                // 处理下一个
                processOrderedText();
            }
        }
    }

    /**
     * 获取全量文本。
     *
     * @return 全量文本，当没有接收完全时返回空
     */
    std::string getFullText() {
        if (isAddCompleted()) {
    #if defined(__ANDROID__) || defined(USE_SELF_CONVERT)
            return ConvertUtil::wstringToUtf8(mOrderedTextBuffer);
    #else
            return mStrConverter.to_bytes(mOrderedTextBuffer);
    #endif
        }

        return "";
    }

    /**
     * 清除待合成文本和状态，在合成出错或者取消合成时调用。
     */
    void clear() {
        mInTextSegList.clear();
        mOutTextSegList.clear();
        mOrderedEndSegIndex = -1;
        mOrderedTextBuffer.clear();
        mTotalOrderedTextLen = 0;
        mFetchedTextLen = 0;
        mFetchStatus = FetchStatus::INIT;
        mOutTextSegIndex = 0;
        mFoundFirstStatusBeg = false;
        mTtsFrameIndex = 1;
    }

    std::shared_ptr<OutTextSeg> findTextSegByTag(const std::string& tag) {
        for (auto& seg : mOutTextSegList) {
            if (seg->mTag == tag) {
                return seg;
            }
        }

        return nullptr;
    }

    void processOrderedText() {
        auto outTextSeg = fetchOrderedText();
        if (outTextSeg != nullptr) {
            switch (mFetchStatus) {
                case INTERRUPTED:
                case INIT: {
                    mFetchStatus = FetchStatus::STARTED;
                } break;
            }

            if (!outTextSeg->isEmpty()) {
                if (m_pOutListener != nullptr) {
                    m_pOutListener->onText(*outTextSeg);
                }
            } else {
                if (outTextSeg->isEnd()) {
                    // 最后一段合成文本为空，直接造一个假结果
                    mockLastOutSegTtsResult(*outTextSeg);
                }
            }
        } else {
            switch (mFetchStatus) {
                case STARTED: {
                    mFetchStatus = FetchStatus::INTERRUPTED;
                }
            }
        }
    }

private:
    void mockLastOutSegTtsResult(OutTextSeg& lastSeg) {
        Json::Value contentJson;
        contentJson["cancel"] = "0";
        contentJson["cnt_id"] = "0";
        contentJson["dte"] = "speex-wb;7";
        contentJson["dtf"] = "audio/L16;rate=16000";
        contentJson["dts"] = STATUS_ALLONE;
        contentJson["error"] = "";
        contentJson["frame_id"] = 1;
        contentJson["text_end"] = lastSeg.mOffset + lastSeg.getTextLen();
        contentJson["text_percent"] = 100;
        contentJson["text_seg"] = "";
        contentJson["text_start"] = lastSeg.mOffset;
        contentJson["url"] = "0";

        Json::Value contentArray;
        contentArray.append(contentJson);

        Json::Value paramsJson;
        paramsJson["cmd"] = "tts";
        paramsJson["lrst"] = "1";
        paramsJson["rstid"] = 1;
        paramsJson["sub"] = "tts";

        Json::Value dataJson;
        dataJson["content"] = contentArray;
        dataJson["params"] = paramsJson;

        Json::Value dataArray;
        dataArray.append(dataJson);

        Json::Value bizParamJson;
        bizParamJson["data"] = dataArray;

        char audio[] = {0};
        onOriginTtsData(lastSeg.mTag, bizParamJson, audio, 0);
    }
};

#endif    //AIUI_SDK_STREAMNLPTTSHELPER_H
