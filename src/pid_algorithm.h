#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x) ((x) * (x))


#include "filter_algorithm.h"
#include "math.h"
#include "math_algorithm.h"

class C_PID
{
public:
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpE;    // 本次偏差
    float fpPreE; // 上次偏差
    float fpSumE; // 总偏差
    float fpInput;
    float fpInputpre;
    float fpOutput;
    float fpOutputpre;
    float fpEpMax;   // 比例项输出最大值
    float fpEiMax;   // 积分项输出最大值
    float fpEdMax;   // 微分项输出最大值
    float fpEMin;    // 积分下限
    float fpEMax;    // 积分上限
    float fpSumEMax; // 积分项sumE最大值

    float fpUp; // 比例输出
    float fpUi; // 积分输出
    float fpUd; // 微分输出
    float fpU;  // 本次PID运算结果
    float fpUpre;
    float fpUMax; // PID运算后输出最大值及做遇限削弱时的上限值
    float fpTs;   // PID控制周期，单位：s

    C_PID() {}
    C_PID(float Kp, float Ki, float Kd, float UMax, float UiMax, float UdMax, float ts)
        : fpKp(Kp),
          fpKi(Ki),
          fpKd(Kd),
          fpUMax(UMax),
          fpEpMax(UMax),
          fpEiMax(UiMax),
          fpEdMax(UdMax),
          fpTs(ts) {} // initialize PID
    ~C_PID() {}

    void CalPID(void);
    void CalISeparatedPID(void);
    void CalIResistedPID(void);
    void CalIWeakenPID(void);
    void CalFilterPID(void);
    void CalComprehensivePID(void);
};

class C_Tr
{
public:
    float fpInput1;
    float fpInput2;
    float fpInput3;
    float fpInputpre1;
    float fpInputpre2;
    float fpInputpre3;
    float fpOutput1;
    float fpOutput2;
    float fpOutput3;
    float fpOutputpre1;
    float fpOutputpre2;
    float fpOutputpre3;
    float fpTs;

    C_Tr() {}
    C_Tr(float Ts) : fpTs(Ts) {} // 采样周期
    ~C_Tr() {}

    void TrF1(float t1, float t2);
    void TrF2(float t);
    void TrF3(float t);
    void LagCompensator(float gain, float t1, float t2);
};

// 滑模相关
class C_TD
{
public:
    float m_x1;
    float m_x2;
    float m_x3;
    float m_x;
    float m_r;
    float m_h;
    float m_T;
    float m_aim;

    C_TD() {}
    C_TD(float r, float h, float T) : m_r(r), m_h(h), m_T(T) {}

    void TD_Function(void);

private:
    int Sign_Judge(float fp_Judge_Number) { return fp_Judge_Number >= 0 ? 1 : -1; }
};

#endif
