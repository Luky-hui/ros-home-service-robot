/*
 * Base64Util.h
 *
 *  Created on: 2017年3月3日
 *      Author: hj
 */

#ifndef BASE64UTIL_H_
#define BASE64UTIL_H_

#include <string>

/**
 * Base64工具。
 */
class Base64Util
{
public:
    static std::string encode(const unsigned char* bytes_to_encode, unsigned int len);

    static std::string encode(const std::string& s);

    static std::string decode(const std::string& s);
};

#endif /* BASE64UTIL_H_ */
