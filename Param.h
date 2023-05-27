#ifndef PARAM_H_
#define PARAM_H_

#include <eigen/Dense.h>

#define GNSS_THREAD_TS_MS 100
#define GNSS_THREAD_PRIORITY osPriorityNormal
#define GNSS_RTK_FIX_LED PB_9
#define GNSS_FIX_LED PB_8
#define GNSS_THREAD_SIZE 4096
#define GNSS_DO_PRINTF false
#define GNSS_RX PA_10
#define GNSS_TX PB_6
#define GNSS_UART_BAUD 921600
#define GNSS_MAX_UBX_MSG 10
#define GNSS_UART_MAX_CARRY_BYTES 128



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
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true  // if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_MAG_CALIBRATION true // if this is false then no mag calibration gets applied, e.g. A_mag = I, b_mag = 0
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

    uint32_t itow;
    uint32_t t;

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
    Eigen::Vector3d hpLlh;          // High precision position in Global frame
    //double longitude_hp;            // High precision position
    //double latitude_hp;             // High precision position
    float hMSL;                     // Height above mean sea level
    float hAcc;                     // estimated horizontal accuracy
    float vAcc;                     // estimated vertical accuracy

    //UBX-NAV-PVT
    uint8_t fix_type;               // see interface description for details
    bool validMag;
    uint8_t numSV;                  // number of satellites used for position solution
    Eigen::Vector3f llh, velNED;    // position and in Global frame and volecites in NED frame
    float sAcc;                     // estimated speed accuracies
    float gSpeed;                   // ground speed
    float headMot;               // angle of motion relative to north$
    float headAcc;
    float magDec;
    float magAcc;
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
        quat.setIdentity();
        rpy.setZero();

        base_time_mode = 0;
        base_svin_valid = 0;
        meanAcc_SVIN = 1000000;       //maybe dont set it to 0 per default
        itow = 0;

        refstationid = 0;
        relPosNED.setZero();
        accNED.setZero();
        rtk_flags = 0;

        invalidLLH = 1;
        hpLlh.setZero();
        hMSL = 0;
        hAcc = 10000000;               //maybe dont set it to 0 per default
        vAcc = 10000000;               //maybe dont set it to 0 per default

        fix_type = 0;
        numSV = 0;
        llh.setZero();
        velNED.setZero();
        sAcc = 10000000;               //maybe dont set it to 0 per default
        gSpeed = 0;
        headMot = 0;
        headAcc = 1000000;
        pDOP = 10000000;               //maybe dont set it to 0 per default
        lastCorrectionAge = 255;       //maybe dont set it to 0 per default
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
        static const Eigen::Matrix3f A_mag = ( Eigen::Matrix3f() <<  0.9858873f,  0.0000000f,  0.0000000f,
                                                                     0.0105822f,  0.9857083f,  0.0000000f,
                                                                     0.0572703f,  0.0053311f,  1.0284045f ).finished(); //from 22.05.23
        static const Eigen::Vector3f b_mag = ( Eigen::Vector3f() <<  0.4574184f, -0.3294953f,  0.3824048f ).finished(); //from 22.05.23
        static const Eigen::Vector3f b_acc = ( Eigen::Vector3f() <<  0.1982683f, -0.1430836f,  0.0000000f ).finished();
    }
}
#endif /* PARAM_H_ */