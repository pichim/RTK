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

namespace Param{
    namespace IMU{
        // kinematic parameters
        static const float kp = 2.0f;                   // mahonyRP kp
        static const float ki = kp * (2.0f*M_PI* 0.4f); // mahonyRP ki
        static const Eigen::Matrix3f A_mag = ( Eigen::Matrix3f() <<  0.9686518f,  0.0000000f,  0.0000000f,
                                                                     0.0547396f,  0.9901187f,  0.0000000f,
                                                                     0.0029033f,  0.0346707f,  1.0412294f ).finished();
        static const Eigen::Vector3f b_mag = ( Eigen::Vector3f() << -0.0767850f, -0.2091931f, -0.4915223f ).finished();
    }
    /*
    namespace Config{
        // default parameters for robots movement
        const float DISTANCE_THRESHOLD = 0.1f;        // minimum allowed distance to obstacle in [m]
        const float VELOCITY_THRESHOLD = 0.05;        // velocity threshold before switching off, in [m/s] and [rad/s]
    }
    namespace Physics{
        const float max_voltage = 12.0f;                // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
        const float counts_per_turn = 64.0f * 19.0f;    // define counts per turn at gearbox end: counts/turn * gearratio
        const float kn = 530.0f / 12.0f;                // define motor constant in rpm per V
    }
    */
}
#endif /* PARAM_H_ */