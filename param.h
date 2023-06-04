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
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true  // if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_MAG_CALIBRATION true  // if this is false then no mag calibration gets applied, e.g. A_mag = I, b_mag = 0
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
        static const Eigen::Matrix3f A_mag = ( Eigen::Matrix3f() <<  0.9644691f,  0.0000000f,  0.0000000f,
                                                                     0.0547890f,  0.9948578f,  0.0000000f,
                                                                     0.0048828f,  0.0374821f,  1.0406731f ).finished();
        static const Eigen::Vector3f b_mag = ( Eigen::Vector3f() <<  0.0554715f,  0.2206336f,  0.5110115f ).finished();
        static const Eigen::Vector3f b_acc = ( Eigen::Vector3f() <<  0.0970105f, -0.2439377f,  0.0000000f ).finished();
    }
}
#endif /* PARAM_H_ */