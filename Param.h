#ifndef PARAM_H_
#define PARAM_H_

#include <eigen/Dense.h>

#define GNSS_THREAD_TS_MS 100
#define GNSS_THREAD_PRIORITY osPriorityNormal
#define GNSS_RTK_FIX_LED PB_9
#define GNSS_FIX_LED PB_8
#define GNSS_THREAD_SIZE 4096
#define GNSS_DO_PRINTF true
#define GNSS_RX PA_10
#define GNSS_TX PB_6
#define GNSS_UART_BAUD 115200



#define SDCARD_THREAD_TS_MS 20
#define SDCARD_THREAD_PRIORITY osPriorityNormal
#define SDCARD_THREAD_SIZE 4096
#define SDCARD_PIN_ADDITIONAL_LED PB_9
#define SDCARD_DO_PRINTF false
#define SDCARD_BUFFER_SIZE 512


#define IMU_THREAD_TS_MS 20
#define IMU_THREAD_PRIORITY osPriorityNormal
#define IMU_THREAD_SIZE 4096
#define IMU_PIN_SDA PC_9
#define IMU_PIN_SCL PA_8
#define IMU_DO_PRINTF false
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

    uint32_t itow;

    //UBX-NAV-SVIN
    bool base_time_mode;
    bool base_svin_valid;
    float meanAcc_SVIN; // [m]

    //UBX-NAV-COV
    bool posCOVvalid;
    bool velCOVvalid;
    float posCovNN; //rewrite in matrix form but for now this should suffice
    float posCovNE;
    float posCovND;
    float posCovEE;
    float posCovED;
    float posCovDD;
    float velCovNN; //rewrite in matrix form but for now this should suffice
    float velCovNE;
    float velCovND;
    float velCovEE;
    float velCovED;
    float velCovDD;

    //UBX-NAV-DOP
    float gDOP;
    float pDOP;
    float tDOP;
    float vDOP;
    float hDOP;
    float nDOP;
    float eDOP;


    //UBX-NAV-STATUS
    uint8_t gnss_fix; 
    bool diffCorr_available;        //true if RTCM data is received
    bool rtk_float; 
    bool rtk_fix;   
    uint32_t ttff;                  //time to first fix
    uint32_t msss;                  //milliseconds since startup

    //UBX-NAV-RELPOSNED
    uint16_t refstationid;          // ID of the basesation its receiving correction data from
    Eigen::Vector3f relPosNED, accNED; //relative position to BASE in NED frame and thier estimated accuracy
    uint32_t rtk_flags;             // various status flags for RTK related systems

    //UBX-NAV-HPPOSLLH
    bool invalidLLH;                // if GNSS has no fix this is 1
    Eigen::Vector3f hpLlh;          // High precision position in Global frame
    float hMSL;                     // Height above mean sea level
    float hAcc;                     // estimated horizontal accuracy
    float vAcc;                     // estimated vertical accuracy

    //UBX-NAV-PVT
    uint8_t fix_type;               // see interface description for details
    uint8_t numSV;                  // number of satellites used for position solution
    Eigen::Vector3f llh, velNED;    // position and in Global frame and volecites in NED frame
    float sAcc;                     // estimated speed accuracies
    float gSpeed;                   // ground speed
    float headMotion;               // angle of motion relative to north
    uint8_t lastCorrectionAge;      // time since last correction data hase be received (for details see interface description)

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
        meanAcc_SVIN = 0;       //maybe dont set it to 0 per default
        itow = 0;

        refstationid = 0;
        relPosNED.setZero();
        accNED.setZero();
        rtk_flags = 0;

        invalidLLH = 1;
        hpLlh.setZero();
        hMSL = 0;
        hAcc = 0;               //maybe dont set it to 0 per default
        vAcc = 0;               //maybe dont set it to 0 per default

        fix_type = 0;
        numSV = 0;
        llh.setZero();
        velNED.setZero();
        sAcc = 0;               //maybe dont set it to 0 per default
        gSpeed = 0;
        pDOP = 0;               //maybe dont set it to 0 per default
        lastCorrectionAge = 0;  //maybe dont set it to 0 per default
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