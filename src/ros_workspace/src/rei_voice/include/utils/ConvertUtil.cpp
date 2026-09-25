//
// Created by hj on 2023/6/7.
//

#include "ConvertUtil.h"

#if defined(__ANDROID__)
    #include <locale>
    #include <bits/codecvt.h>
#elif defined(USE_SELF_CONVERT)
    #include <stdio.h>
#else
   #include <codecvt>
#endif

#include "string.h"

#ifdef USE_SELF_CONVERT
static uint32_t utf8ToUtf32(const char* str, size_t len)
{
    uint32_t codepoint = 0;
    if (len >= 1 && (str[0] & 0x80) == 0) { // ASCII×Ö·û
        codepoint = str[0];
    } else if (len >= 2 && (str[0] & 0xE0) == 0xC0) { // 2×Ö½ÚUTF-8×Ö·û
        codepoint = ((str[0] & 0x1F) << 6) | (str[1] & 0x3F);
    } else if (len >= 3 && (str[0] & 0xF0) == 0xE0) { // 3×Ö½ÚUTF-8×Ö·û
        codepoint = ((str[0] & 0x0F) << 12) | ((str[1] & 0x3F) << 6) | (str[2] & 0x3F);
    } else if (len >= 4 && (str[0] & 0xF8) == 0xF0) { // 4×Ö½ÚUTF-8×Ö·û
        codepoint = ((str[0] & 0x07) << 18) | ((str[1] & 0x3F) << 12) | ((str[2] & 0x3F) << 6) | (str[3] & 0x3F);
    }
    return codepoint;
}

static std::string utf32ToUtf8(uint32_t codepoint)
{
    std::string utf8Str;
    if (codepoint <= 0x7F) { // ASCII×Ö·û
        utf8Str.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FF) { // 2×Ö½ÚUTF-8×Ö·û
        utf8Str.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        utf8Str.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else if (codepoint <= 0xFFFF) { // 3×Ö½ÚUTF-8×Ö·û
        utf8Str.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        utf8Str.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        utf8Str.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else if (codepoint <= 0x10FFFF) { // 4×Ö½ÚUTF-8×Ö·û
        utf8Str.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
        utf8Str.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
        utf8Str.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        utf8Str.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    return utf8Str;
}

std::wstring ConvertUtil::utf8ToWstring(const std::string& src)
{
    std::wstring wstr;
    size_t i = 0;
    while (i < src.length()) {
        size_t len = 1;
        while (i + len < src.length() && (src[i + len] & 0xC0) == 0x80) { // ¼ÆËãUTF-8×Ö·û³¤¶È
            len++;
        }
        uint32_t codepoint = utf8ToUtf32(&src[i], len); // ½«UTF-8×Ö·û×ª»»ÎªUnicodeÂëµã
        wchar_t wc = static_cast<wchar_t>(codepoint); // ½«UnicodeÂëµã×ª»»Îª¿í×Ö·û
        wstr += wc; // ½«¿í×Ö·ûÌí¼Óµ½¿í×Ö·û´®ÖÐ
        i += len; // ÒÆ¶¯µ½ÏÂÒ»¸öUTF-8×Ö·û
    }
    return wstr;
}

std::string ConvertUtil::wstringToUtf8(const std::wstring& src)
{
    std::string utf8Str;
    for (size_t i = 0; i < src.length(); i++) {
        uint32_t codepoint = static_cast<uint32_t>(src[i]);
        utf8Str += utf32ToUtf8(codepoint);
    }
    return utf8Str;
}
#else
std::wstring ConvertUtil::utf8ToWstring(const std::string& src)
{
    std::locale sys_locale("");

    const char* srcData = src.c_str();
    const char* srcEnd = src.c_str() + src.size();
    const char* srcNext = 0;

    auto* dstData = new wchar_t[src.size() + 1];
    wchar_t* dstEnd = dstData + src.size() + 1;
    wchar_t* dstNext = 0;

    wmemset(dstData, 0, src.size() + 1);
    typedef std::codecvt<wchar_t, char, mbstate_t> convert_facet;
    mbstate_t inState = {0, 0};

    auto result = std::use_facet<convert_facet>(sys_locale)
                      .in(inState, srcData, srcEnd, srcNext, dstData, dstEnd, dstNext);
    if (result == convert_facet::ok) {
        std::wstring dst = dstData;
        delete[] dstData;

        return dst;
    } else {
        delete[] dstData;
    }

    return L"";
}

std::string ConvertUtil::wstringToUtf8(const std::wstring& src)
{
    std::locale sys_locale("");

    const wchar_t* fromData = src.c_str();
    const wchar_t* fromEnd = src.c_str() + src.size();
    const wchar_t* fromNext = 0;

    int wchar_size = 4;
    char* dstData = new char[(src.size() + 1) * wchar_size];
    char* dstEnd = dstData + (src.size() + 1) * wchar_size;
    char* dstNext = 0;

    memset(dstData, 0, (src.size() + 1) * wchar_size);

    typedef std::codecvt<wchar_t, char, mbstate_t> convert_facet;
    mbstate_t outState = {0, 0};
    auto result = std::use_facet<convert_facet>(sys_locale)
                      .out(outState, fromData, fromEnd, fromNext, dstData, dstEnd, dstNext);
    if (result == convert_facet::ok) {
        std::string dst = dstData;
        delete[] dstData;

        return dst;
    } else {
        delete[] dstData;
    }

    return "";
}
#endif

std::string ConvertUtil::toString(int val)
{
    char buffer[20] = {0};
    snprintf(buffer, sizeof(buffer), "%d", val);

    return std::string(buffer);
}

std::string ConvertUtil::toString(long long val)
{
    char buffer[20] = {0};
    snprintf(buffer, sizeof(buffer), "%lld", val);

    return std::string(buffer);
}
