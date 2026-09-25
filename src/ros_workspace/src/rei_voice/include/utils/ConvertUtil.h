//
// Created by hj on 2023/6/7.
//

#ifndef AIUI_SDK_CONVERTUTIL_H
#define AIUI_SDK_CONVERTUTIL_H

#include <string>

class ConvertUtil
{
public:
    static std::wstring utf8ToWstring(const std::string& src);

    static std::string wstringToUtf8(const std::wstring& src);

    static std::string toString(int val);

    static std::string toString(long long val);
};

#endif    //AIUI_SDK_CONVERTUTIL_H
