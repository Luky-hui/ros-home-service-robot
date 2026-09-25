/*
 * @Author: Maintainer maintainer@example.com
 * @Date: 2022-09-12 14:21:48
 * @LastEditors: Maintainer maintainer@example.com
 * @LastEditTime: 2023-01-12 17:56:07
 * @Gitee: https://gitee.com/REINOVO
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved. 
 * @Description: 
 */
#ifndef __REINOVO_PRINT_H_
#define __REINOVO_PRINT_H_

#define RESET   "\033[0m"
#define YELLOW  "\033[33m"      /* Yellow */
#define RED     "\033[31m"      /* Red */
#define BLUE    "\033[34m"      /* Blue */
#define DEBUG_OUT true
#define PCOUT_DEBUG DEBUG_OUT && std::cout<<BLUE<<"[DEBUG "<<__TIMESTAMP__<<"]:"<<__FUNCTION__<<": "
#define PCOUT_ERROR DEBUG_OUT && std::cout<<RED<<"[ERROR "<<__TIMESTAMP__<<"]:"<<__FUNCTION__<<"line "<<__LINE__<<": "
#define PCOUT_WARRN DEBUG_OUT && std::cout<<YELLOW<<"[WARRN "<<__TIMESTAMP__<<"]:"<<__FUNCTION__<<"line "<<__LINE__<<": "
#define PCOUT_TIME DEBUG_OUT && std::cout<<RED<<"[TIME "<<__TIMESTAMP__<<"]: "

#endif