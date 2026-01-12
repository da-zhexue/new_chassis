#include "ADRC.h"
#include <math.h>
#include "user_lib.h"

/**
 * @brief 初始化ADRC控制器参数和状态
 * @param adrc: ADRC控制器结构体指针
 * @param r: 快速跟踪因子
 * @param h: 采样周期
 * @param b: 系统系数
 * @param delta: fal函数线性区间宽度
 * @param beta_01, beta_02, beta_03: ESO增益参数
 * @param alpha1, alpha2: NLSEF非线性参数
 * @param beta_1, beta_2: NLSEF增益参数
 */
void ADRC_Init(ADRC_Controller *adrc, float r, float h, float b, float delta, 
               float beta_01, float beta_02, float beta_03, 
               float alpha1, float alpha2, float beta_1, float beta_2) {
    adrc->r = r;
    adrc->h = h;
    adrc->b = b;
    adrc->delta = delta;
    adrc->beta_01 = beta_01;
    adrc->beta_02 = beta_02;
    adrc->beta_03 = beta_03;
    adrc->alpha1 = alpha1;
    adrc->alpha2 = alpha2;
    adrc->beta_1 = beta_1;
    adrc->beta_2 = beta_2;
    
    // 初始化状态变量
    adrc->x1 = 0;
    adrc->x2 = 0;
    adrc->z1 = 0;
    adrc->z2 = 0;
    adrc->z3 = 0;
    adrc->u = 0;
}

/**
 * @brief fal函数：非线性函数，用于ESO和NLSEF
 * @param e: 误差
 * @param alpha: 非线性幂次
 * @param delta: 线性区间宽度
 * @return 非线性函数值
 */
float fal(float e, float alpha, float delta) {
    float result;
    if (fabs(e) <= delta) {
        // 在线性区间内，避免小误差时增益过大
        result = e / powf(delta, 1.0f - alpha);
    } else {
        // 在非线性区间，采用幂次函数
        result = powf(fabs(e), alpha) * sign(e);
    }
    return result;
}

/**
 * @brief fhan函数：最速跟踪微分器，用于TD安排过渡过程
 * @param x1: 当前跟踪值
 * @param x2: 当前微分值
 * @param r: 快速因子
 * @param h: 采样周期
 * @return 微分更新量
 */
float fhan(float x1, float x2, float r, float h) {
    float d, a, a0, y, a1, a2, fhan;
    
    d = r * h * h;           // 计算步长相关参数
    a0 = h * x2;             // 计算当前微分贡献
    y = x1 + a0;             // 预测下一步位置
    a1 = sqrtf(d * (d + 8 * fabs(y)));  // 计算非线性参数
    a2 = a0 + sign(y) * (a1 - d) / 2.0f;  // 计算中间变量
    
    // 分段函数合成
    if (fabs(y) > d) {
        a = a2;
    } else {
        a = y / h;
    }
    
    // 最终输出
    if (fabs(a) > d) {
        fhan = -r * sign(a);
    } else {
        fhan = -r * a / d;
    }
    
    return fhan;
}

/**
 * @brief 限幅函数
 * @param amt: 待限幅值
 * @param low: 下限
 * @param high: 上限
 * @return 限幅后的值
 */
float constrain_float(float amt, float low, float high) {
    if (amt < low) return low;
    else if (amt > high) return high;
    else return amt;
}

/**
 * @brief ADRC核心控制函数
 * @param adrc: ADRC控制器结构体指针
 * @param v: 目标值（输入）
 * @param y: 反馈值（系统输出）
 * @return 控制量u
 */
float ADRC_Control(ADRC_Controller *adrc, float v, float y) {
    float e, e1, e2, u0;
    
    // 1. TD（跟踪微分器）：安排过渡过程，提取微分信号
    adrc->x1 = adrc->x1 + adrc->h * adrc->x2;  // 更新跟踪状态
    adrc->x2 = adrc->x2 + adrc->h * fhan(adrc->x1 - v, adrc->x2, adrc->r, adrc->h);  // 更新微分状态
    
    // 2. ESO（扩张状态观测器）：实时估计系统状态和总扰动
    e = adrc->z1 - y;  // 观测误差
    adrc->z1 = adrc->z1 + adrc->h * (adrc->z2 - adrc->beta_01 * e);  // 更新状态z1（跟踪输出y）
    adrc->z2 = adrc->z2 + adrc->h * (adrc->z3 - adrc->beta_02 * fal(e, 0.5f, adrc->delta) + adrc->b * adrc->u);  // 更新状态z2（跟踪微分）
    adrc->z3 = adrc->z3 + adrc->h * (-adrc->beta_03 * fal(e, 0.25f, adrc->delta));  // 更新状态z3（估计总扰动）
    
    // 状态限幅，防止溢出
    adrc->z1 = constrain_float(adrc->z1, -30000, 30000);
    adrc->z2 = constrain_float(adrc->z2, -30000, 30000);
    adrc->z3 = constrain_float(adrc->z3, -30000, 30000);
    
    // 3. NLSEF（非线性状态误差反馈）：生成初步控制量
    e1 = adrc->x1 - adrc->z1;  // 安排过渡后的目标与观测状态的误差
    e2 = adrc->x2 - adrc->z2;  // 微分误差
    u0 = adrc->beta_1 * fal(e1, adrc->alpha1, adrc->delta) + adrc->beta_2 * fal(e2, adrc->alpha2, adrc->delta);  // 非线性组合
    
    // 4. 扰动补偿：生成最终控制量
    adrc->u = u0 - adrc->z3 / adrc->b;  // 补偿总扰动
    
    return adrc->u;
}
