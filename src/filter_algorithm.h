#ifndef __FILTER_ALGORITHM_H__
#define __FILTER_ALGORITHM_H__


// 可以用RC低通滤波电路推导原理
// 推导出传递函数1/(1+RCS),RC=时间常数，故1/RC为截止频率
// 再对传递函数进行z变换，即可推导出离散的一阶滤波公式
// 其中m_off_freq为截止频率，m_samp_tim为采样周期
class C_LPF
{
public:
    float m_preout;
    float m_out;
    float m_in;
    float m_off_freq; // 权重
    float m_samp_tim; // 采样步长

    C_LPF(){};
    ~C_LPF(){};
    C_LPF(float off_freq, float samp_tim) : m_off_freq(off_freq), m_samp_tim(samp_tim){};
    void LpFilter(void);
};

extern C_LPF lpf_PID;

#endif
