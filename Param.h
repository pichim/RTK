#ifndef PARAM_H_
#define PARAM_H_

#include <eigen/Dense.h>

#define SDCARD_THREAD_TS_MS 1000
#define SDCARD_THREAD_PRIORITY osPriorityNormal
#define SDCARD_THREAD_SIZE 4096
#define SDCARD_THREAD_PRINTF true

#define IMU_THREAD_TS_MS 20
#define IMU_THREAD_PRIORITY osPriorityNormal
#define IMU_THREAD_SIZE 4096
#define IMU_SDA PC_9
#define IMU_SCL PA_8
#define IMU_THREAD_PRINTF false

typedef struct Data_s {
    Eigen::Vector3f gyro, acc, mag;
    Data_s() :
    gyro(0.0f, 0.0f, 0.0f),
    acc(0.0f, 0.0f, 0.0f),
    mag(0.0f, 0.0f, 0.0f) {}
} Data_t;

/*
namespace Param{
    namespace Kinematics{
        // kinematic parameters
        const float r_wheel = 0.0766f / 2.0f;   // wheel radius
        const float l_wheel = 0.176f;           // distance from wheel to wheel
    }
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
}
*/
#endif /* PARAM_H_ */