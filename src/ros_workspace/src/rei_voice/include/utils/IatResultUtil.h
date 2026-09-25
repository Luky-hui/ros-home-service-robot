//
// Created by hj on 2023/7/11.
//

#ifndef AIUI_SDK_IATRESULTUTIL_H
#define AIUI_SDK_IATRESULTUTIL_H

#include "json/json.h"

#include <string>
#include <map>

using namespace aiui_va;

class IatResultUtil
{
private:
    static std::map<int, std::string> sPgsResult;

public:
    static void clearPgsResult() { sPgsResult.clear(); }

    static std::string parsePgsIatText(const Json::Value& textJson);

    static std::string parseIatResult(const Json::Value& textJson);
};

#endif    //AIUI_SDK_IATRESULTUTIL_H
