#ifndef _ADRC_H_
#define _ADRC_H_

// 定义ADRC控制器数据结构体
typedef struct {
    // TD（跟踪微分器）参数
    float r;        // 快速跟踪因子，值越大跟踪越快，但噪声可能放大
    float h;        // 系统采样周期（控制周期）
    
    // ESO（扩张状态观测器）参数
    float b;        // 系统系数，描述控制量对被控对象的影响程度
    float delta;    // fal函数的线性区间宽度，通常取5h~10h
    float beta_01;  // ESO增益1，与状态z1的收敛速度相关
    float beta_02;  // ESO增益2，与状态z2的收敛速度相关
    float beta_03;  // ESO增益3，与状态z3（总扰动）的收敛速度相关
    
    // NLSEF（非线性状态误差反馈）参数
    float alpha1;   // 非线性参数1，满足0<alpha1<1
    float alpha2;   // 非线性参数2，满足1<alpha2
    float beta_1;   // 误差e1的增益
    float beta_2;   // 误差e2的增益
    
    // 状态变量（不需要初始化）
    float x1, x2;   // TD状态：x1为跟踪信号，x2为微分信号
    float z1, z2, z3; // ESO状态：z1跟踪输出y，z2跟踪微分，z3估计总扰动
    float u;        // 最终控制量输出
} ADRC_Controller;

// 函数声明
void ADRC_Init(ADRC_Controller *adrc, float r, float h, float b, float delta, 
               float beta_01, float beta_02, float beta_03, 
               float alpha1, float alpha2, float beta_1, float beta_2);
float ADRC_Control(ADRC_Controller *adrc, float v, float y);

// 辅助函数声明
float fal(float e, float alpha, float delta);
float fhan(float x1, float x2, float r, float h);
float constrain_float(float amt, float low, float high);

#endif
