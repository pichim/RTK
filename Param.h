#ifndef PARAM_H_
#define PARAM_H_

#include <eigen/Dense.h>

#define GNSS_THREAD_TS_MS 200
#define GNSS_THREAD_PRIORITY osPriorityNormal
#define GNSS_THREAD_SIZE 4096
#define GNSS_DO_PRINTF true
#define GNSS_RX PA_10
#define GNSS_TX PB_6
#define GNSS_UART_BAUD 57600 //default for the SIK Telemetry module



#define SDCARD_THREAD_TS_MS 200
#define SDCARD_THREAD_PRIORITY osPriorityNormal
#define SDCARD_THREAD_SIZE 4096
#define SDCARD_PIN_ADDITIONAL_LED PB_9
#define SDCARD_DO_PRINTF false
#define SDCARD_BUFFER_SIZE 512


#define IMU_THREAD_TS_MS 100
#define IMU_THREAD_PRIORITY osPriorityNormal
#define IMU_THREAD_SIZE 4096
#define IMU_PIN_SDA PC_9
#define IMU_PIN_SCL PA_8
#define IMU_DO_PRINTF true
#define IMU_DO_USE_ACC_CALIBRATION false // in case you lift of not leveled, this has to be false
#define IMU_DO_USE_MAG_CALIBRATION true

class Data
{
public:
    Data() {
        initialise();
    };
    virtual ~Data() {};

    Eigen::Vector3f gyro, acc, mag;

    int32_t itow;

    //UBX-NAV-SVIN
    bool base_time_mode;
    bool base_svin_valid;
    float meanAcc_SVIN; // [m]

    //UBX-NAV-STATUS
    uint8_t gnss_fix; //
    bool rtk_float; //
    bool rtk_fix;   //
    uint32_t ttff;  //
    uint32_t msss;  //

    //UBX-NAV-RELPOSNED
    uint16_t refstationid; //
    Eigen::Vector3f relPosNED, accNED; //
    uint32_t rtk_flags; //

    //UBX-NAV-HPPOSLLH
    bool invalidLLH; //
    Eigen::Vector3f hpLlh; //
    float hMSL; //
    float hAcc;  //
    float vAcc;  //

    //UBX-NAV-PVT
    uint8_t fix_type; //
    uint8_t numSV; //
    Eigen::Vector3f llh, velNED; //
    float sAcc; //
    float gSpeed;   //
    float pDOP; //
    uint8_t lastCorrectionAge; //

    //UBX-NAV-TIMEUTC
    bool gnss_time_valid;
    uint16_t gnss_year;
    uint8_t gnss_month;
    uint8_t gnss_day;
    uint8_t gnss_hour;
    uint8_t gnss_minutes;
    uint8_t gnss_seconds;

private:
    void initialise() {
        gyro.setZero();
        acc.setZero();
        mag.setZero();

        base_time_mode = 0;
        base_svin_valid = 0;
        meanAcc_SVIN = 0;
        itow = 0;
    };
};

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