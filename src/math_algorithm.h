/*---------------------------------------------------------------------
版权声明：HITCRT(哈工大竞技机器人队)
文件名：math_algorithm.h
最近修改日期：2022.10.14
版本：1.0
---------------------------------------------------------------------*/
#ifndef __MATH_ALGORITHM_H__
#define __MATH_ALGORITHM_H__

float ClipFloat(float fpValue, float fpMin,
               float fpMax);                         // 浮点数削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值

#endif
