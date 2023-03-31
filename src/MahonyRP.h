#ifndef MAHONY_RP_H_
#define MAHONY_RP_H_

#include <mbed.h>

#include "param.h"

class MahonyRP
{
public:
    MahonyRP();
    MahonyRP(float kp, float ki, float Ts);
    virtual ~MahonyRP();

    void Setup(float& kp, float& ki, float& Ts);
    void Update(Eigen::Vector3f& gyro, Eigen::Vector3f& acc);
    Eigen::Quaternionf GetOrientationAsQuaternion();
    Eigen::Vector3f GetOrientationAsRPYAngles();
        
private:
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_Ts = 1.0f;
    Eigen::Quaternionf m_quat;
    Eigen::Vector3f m_bias;
    Eigen::Vector3f m_rpy;

    void initialise();
    Eigen::Vector3f quat2rpy(Eigen::Quaternionf& quat);
};

#endif /* MAHONY_RP_H_ */
