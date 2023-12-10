/*
 * log.hpp
 *
 *  Created on: 2023/11/12
 *      Author: user
 */

#ifndef SRC_LOG_HPP_
#define SRC_LOG_HPP_

#include "stm32h7xx_hal.h"
#pragma GCC push_options
#pragma GCC optimize("no-inline")
#include "etl/string.h"
#include "etl/string_stream.h"
#include "etl/ostream.h"
#include <etl/queue.h>
#pragma GCC pop_options

namespace Log
{
    constexpr size_t LOG_MAX_LEN = 512; // ログ情報の最大値
    constexpr size_t bufferSize = 4;    // リングバッファのサイズを指定

    void test();
#if 0
    void createLog(etl::string_view func, etl::string_view msg);
#endif
}
#endif /* SRC_LOG_HPP_ */
