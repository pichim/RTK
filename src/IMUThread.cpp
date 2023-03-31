#include "IMUThread.h"
#include <chrono>
#include <cstdint>

IMUThread::IMUThread(Data& data) :
    m_data(data),
    m_imu(IMU_PIN_SDA, IMU_PIN_SCL),
    m_mahonyRP(Param::IMU::kp, Param::IMU::ki, IMU_THREAD_TS_MS * 1.0e-3f),
    m_thread(IMU_THREAD_PRIORITY, IMU_THREAD_SIZE)
{
#if IMU_DO_USE_MAG_CALIBRATION
    m_magCalib.SetCalibrationParameter(Param::IMU::A_mag, Param::IMU::b_mag);   
#endif
}

IMUThread::~IMUThread()
{
    m_ticker.detach();
}

void IMUThread::StartThread()
{
    m_thread.start(callback(this, &IMUThread::run));
    m_ticker.attach(callback(this, &IMUThread::sendThreadFlag), std::chrono::milliseconds{ static_cast<long int>( IMU_THREAD_TS_MS ) });
}

void IMUThread::run()
{        
    static const uint16_t Navg = (uint16_t)( 1.0f / (IMU_THREAD_TS_MS * 1.0e-3f) );
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset;
    static Eigen::Vector3f acc_offset;
    gyro_offset.setZero();
    acc_offset.setZero();

/*
#if IMU_DO_USE_MAG_CALIBRATION
    static Eigen::Matrix3f A_mag;
    static Eigen::Vector3f b_mag;
    A_mag <<  0.9686518f,  0.0000000f,  0.0000000f,
              0.0547396f,  0.9901187f,  0.0000000f,
              0.0029033f,  0.0346707f,  1.0412294f;
    b_mag << -0.0767850f, -0.2091931f, -0.4915223f;
    m_magCalib.SetCalibrationParameter(A_mag, b_mag);   
#endif
*/

    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);

        m_imu.updateGyro();
        m_imu.updateAcc();
        m_imu.updateMag();
        
        Eigen::Vector3f gyro(m_imu.readGyroX(), m_imu.readGyroY(), m_imu.readGyroZ());       
        Eigen::Vector3f acc (m_imu.readAccX() , m_imu.readAccY() , m_imu.readAccZ() );
        Eigen::Vector3f mag (m_imu.readMagX() , m_imu.readMagY() , m_imu.readMagZ() );

        if (!imu_is_calibrated) {
            gyro_offset += gyro;
            acc_offset  += acc;
            avg_cntr++;
            if (avg_cntr == Navg) {
                imu_is_calibrated = true;
                gyro_offset /= avg_cntr;
                acc_offset  /= avg_cntr;
                // we have to keep gravity in acc z direction
                acc_offset(2) = 0.0f;
            }
        }
        
        if (!imu_is_calibrated) {
            m_data.gyro = gyro;
            m_data.acc  = acc;
        } else {
            m_data.gyro = gyro - gyro_offset;
            m_data.acc  = acc;
#if IMU_DO_USE_ACC_CALIBRATION
            m_data.acc -= acc_offset;
#endif
        }

        mag = m_magCalib.ApplyCalibration(mag);
        m_data.mag = mag;

        m_mahonyRP.Update(gyro, acc);
        m_data.quat = m_mahonyRP.GetOrientationAsQuaternion();
        m_data.rpy = m_mahonyRP.GetOrientationAsRPYAngles();

#if IMU_DO_PRINTF
        static Timer timer;
        timer.start();
        float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() * 1.0e-3f;
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                             m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                             m_data.mag(0) , m_data.mag(1) , m_data.mag(2) , time_ms );
        printf("%.6f, %.6f, %.6f, %.6f,", m_data.quat.w(), m_data.quat.w(), m_data.quat.w(), m_data.quat.w() );
        printf("%.6f, %.6f, %.6f\n", m_data.rpy(0) * 180.0f/M_PI, m_data.rpy(1) * 180.0f/M_PI, m_data.rpy(2) * 180.0f/M_PI );
#endif
    }
}

void IMUThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}