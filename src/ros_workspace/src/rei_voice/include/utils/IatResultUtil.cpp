//
// Created by hj on 2023/7/11.
//

#include "IatResultUtil.h"
#include "ConvertUtil.h"

std::map<int, std::string> IatResultUtil::sPgsResult;

std::string IatResultUtil::parsePgsIatText(const Json::Value& textJson)
{
    int sn = textJson["sn"].asInt();
    std::string pgs = textJson["pgs"].asString();
    Json::Value rgArray = textJson["rg"];
    bool ls = textJson["ls"].asBool();

    if ("rpl" == pgs) {
        int begin = rgArray[0].asInt();
        int end = rgArray[1].asInt();
        for (int i = begin; i <= end; i++) {
            sPgsResult.erase(i);
        }
    }

    sPgsResult[sn] = parseIatResult(textJson);

    std::string builder;
    for (auto& it : sPgsResult) {
        builder.append(it.second);
    }

    if (ls) {
        sPgsResult.clear();
    }

    return builder;
}

std::string IatResultUtil::parseIatResult(const Json::Value& textJson)
{
    std::string ret;
    Json::Value wordsArray = textJson["ws"];
    for (int i = 0; i < wordsArray.size(); i++) {
        // 转写结果词，默认使用第一个结果
        Json::Value defVal;
        Json::Value itemsArray = (wordsArray[i])["cw"];
        std::string w = (itemsArray[0])["w"].asString();
        ret.append(w);
    }

    return ret;
}