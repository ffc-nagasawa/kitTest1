/*
 * main_sub.cpp
 *
 *  Created on: 2023/11/12
 *      Author: user
 */

#include "log.hpp"
#include "jsonTest.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void mainSubInit()
{
//	Log::test();
	json::testJson();
}
#ifdef __cplusplus
}
#endif
