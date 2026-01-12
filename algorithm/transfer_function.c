/**
  ******************************************************************************
  * @file    transfer_function.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/5/12
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#include "transfer_function.h"

void Second_Order_TF_Init(Second_Order_TF_t *tf, float *c)
{
    tf->c[0] = c[0];
    tf->c[1] = c[1];
    tf->c[2] = c[2];

    tf->y = 0;
    tf->y_dot = 0;
    tf->y_ddot = 0;
    tf->Last_y_dot = 0;
    tf->Last_y_ddot = 0;
}

double Second_Order_TF_Calculate(Second_Order_TF_t *tf, double input)
{
    tf->dt = DWT_GetDeltaT(&tf->DWT_CNT);

    tf->u = input;

    tf->y_ddot = tf->u / tf->c[2] - tf->y * tf->c[0] / tf->c[2] - tf->y_dot * tf->c[1] / tf->c[2];

    //Trapezoid intergral
    tf->y_dot += (tf->Last_y_ddot + tf->y_ddot) * tf->dt / 2;
    tf->y += (tf->Last_y_dot + tf->y_dot) * tf->dt / 2;

    tf->Last_y_dot = tf->y_dot;
    tf->Last_y_ddot = tf->y_ddot;
    return tf->y;
}

double Gauss_Rand(void)
{
    static double U, V;
    static int phase = 0;
    double Z;

    if (phase == 0)
    {
        U = rand() / (RAND_MAX + 1.0);
        V = rand() / (RAND_MAX + 1.0);
        Z = sqrt(-2.0 * log(U)) * sin(2.0 * 3.1415926535 * V);
    }
    else
    {
        Z = sqrt(-2.0 * log(U)) * cos(2.0 * 3.1415926535 * V);
    }

    phase = 1 - phase;
    return Z;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

