/*
 * log.cpp
 *
 *  Created on: 2023/11/11
 *      Author: user
 */
#pragma GCC push_options
#pragma GCC optimize("no-inline")
#include "etl/string.h"
#include "etl/string_stream.h"
#include "etl/ostream.h"
#include <etl/queue.h>
#include "etl/circular_buffer.h"
#pragma GCC pop_options

#include "log.hpp"

namespace Log
{

//	etl::queue<etl::string<LOG_MAX_LEN>, bufferSize, etl::memory_model::MEMORY_MODEL_SMALL> ringBuffer  __attribute__((section(".log_queue")));
	static etl::circular_buffer<int, 500> circularBuffer __attribute__((section(".log_queue")));
void test()
{
	   for (int i = 0; i < 10; ++i) {
    circularBuffer.push(i);
	   }
//        ringBuffer.push("test1");
//        ringBuffer.push("test2");
//        ringBuffer.push("test3");
//        ringBuffer.push("test4");
//        etl::cout << "test!" << etl::endl;
}

#if 0
    void createLog(etl::string_view func, etl::string_view msg)
    {
        static etl::string<LOG_MAX_LEN> msgBuffer __attribute__((section(".log_queue")));

        // initialize
        msgBuffer.clear();
        msgBuffer.initialize_free_space();
        etl::string_stream ss(msgBuffer);

        // date
        // message
        ss << msg << ",";

        // terminate
        msgBuffer += "\r\n";

        if (!ringBuffer.full())
        {
            ringBuffer.push(msgBuffer);
        }
    }
#endif
}
