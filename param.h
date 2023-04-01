#ifndef PARAM_H_
#define PARAM_H_

#include <eigen/Dense.h>

#define SDCARD_THREAD_TS_MS 1000
#define SDCARD_THREAD_PRIORITY osPriorityNormal
#define SDCARD_THREAD_SIZE 4096
#define SDCARD_PIN_ADDITIONAL_LED PB_9
//#define SDCARD_DO_PRINTF false

#define IMU_THREAD_TS_MS 20
#define IMU_THREAD_PRIORITY osPriorityNormal
#define IMU_THREAD_SIZE 4096
#define IMU_PIN_SDA PC_9
#define IMU_PIN_SCL PA_8
#define IMU_DO_PRINTF true
#define IMU_DO_USE_ACC_CALIBRATION false // in case you lift off while not leveled, this has to be false
#define IMU_DO_USE_MAG_CALIBRATION true
#define IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE true

class Data
{
public:
    Data() {
        initialise();
    };
    virtual ~Data() {};

    Eigen::Vector3f gyro, acc, mag;
    Eigen::Quaternionf quat;
    Eigen::Vector3f rpy;

private:
    void initialise() {
        gyro.setZero();
        acc.setZero();
        mag.setZero();
        quat.setIdentity();
        rpy.setZero();
    };
};

namespace Param {
    namespace IMU {
        // % bessel
        // p = 2;         % pole at p rad/s
        // kp = 2 * p;
        // ki = kp^2 / 3;
        static const float kp = 2.0f * 2.0f;
        static const float ki = kp * kp / 3.0f;
        static const Eigen::Matrix3f A_mag = ( Eigen::Matrix3f() <<  0.9714836f,  0.0000000f,  0.0000000f,
                                                                     0.0642447f,  0.9902065f,  0.0000000f,
                                                                    -0.0060685f,  0.0375249f,  1.0383099f ).finished();
        static const Eigen::Vector3f b_mag = ( Eigen::Vector3f() << -0.0942469f, -0.2001867f, -0.4814042f ).finished();
    }
}
#endif /* PARAM_H_ */