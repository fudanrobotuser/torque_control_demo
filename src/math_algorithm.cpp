/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人队)
文件名：math_algorithm.cpp
最近修改日期：2022.10.14
版本：第N版
作者：JIN
**********************************************************************/

#include "math_algorithm.h"

/*******************************************************************
函数名称：ClipFloat()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    fpValue:实际值
          fpMin:下限值
          fpMax:上限值
输出：    fpValue：削波后的值
备注：	  适用于浮点数变量的消波
********************************************************************/
float ClipFloat(float fpValue, float fpMin, float fpMax)
{
    if (fpValue < fpMin)
    {
        return fpMin;
    }
    else if (fpValue > fpMax)
    {
        return fpMax;
    }
    else
    {
        return fpValue;
    }
}