#include "IMUThread.h"
#include <chrono>
#include <cstdint>

IMUThread::IMUThread(Data& data) :
    m_data(data),
    m_imu(IMU_PIN_SDA, IMU_PIN_SCL),
    m_thread(IMU_THREAD_PRIORITY, IMU_THREAD_SIZE)
{

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
    static Eigen::Vector3f gyro;
    static Eigen::Vector3f acc;
    static Eigen::Vector3f mag;
        
    static const uint16_t Navg = (uint16_t)( 1.0f / (IMU_THREAD_TS_MS * 1.0e-3f) );
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset;
    static Eigen::Vector3f acc_offset;
    gyro_offset.setZero();
    acc_offset.setZero();

#if IMU_DO_USE_MAG_CALIBRATION
    static Eigen::Matrix3f A_mag;
    static Eigen::Vector3f b_mag;
    A_mag <<  0.9879945f,  0.0000000f,  0.0000000f,
              0.0144004f,  0.9974589f,  0.0000000f,
              0.0012928f,  0.0005467f,  1.0145466f;
    b_mag <<  0.4167900f, -0.2733855f,  0.4077340f;
    m_magCalib.SetCalibrationParameter(A_mag, b_mag);   
#endif

    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);

        m_imu.updateGyro();
        m_imu.updateAcc();
        m_imu.updateMag();
        
        gyro << m_imu.readGyroX(), m_imu.readGyroY(), m_imu.readGyroZ();       
        acc  << m_imu.readAccX() , m_imu.readAccY() , m_imu.readAccZ() ;
        mag  << m_imu.readMagX() , m_imu.readMagY() , m_imu.readMagZ() ;

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

#if IMU_DO_PRINTF
        static Timer timer;
        timer.start();
        int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %i\n", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                               m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                               m_data.mag(0) , m_data.mag(1) , m_data.mag(2) , time_ms);
#endif
    }
}

void IMUThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}