#include "IMUThread.h"
#include <chrono>
#include <cstdint>

IMUThread::IMUThread(Data& data, Mutex& mutex) :
    m_data(data),
    m_imu(IMU_PIN_SDA, IMU_PIN_SCL),
    m_mahony(Param::IMU::kp, Param::IMU::ki, IMU_THREAD_TS_MS * 1.0e-3f),
    m_thread(IMU_THREAD_PRIORITY, IMU_THREAD_SIZE),
    m_dataMutex(mutex)
{
#if IMU_DO_USE_STATIC_MAG_CALIBRATION
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
    static Timer timer;
    timer.start();

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
#if IMU_DO_USE_STATIC_ACC_CALIBRATION
                acc_offset = Param::IMU::b_acc;
#else
                printf("Averaged acc offset: %.7ff, %.7ff, %.7f\n", acc_offset(0), acc_offset(1), acc_offset(2) );
#endif
            }
        }
        
        if (imu_is_calibrated) {
            gyro -= gyro_offset;
            acc -= acc_offset;
            mag = m_magCalib.ApplyCalibration(mag);

            m_dataMutex.lock();
            m_data.t = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
            m_data.gyro = gyro;
            m_data.acc = acc;
            m_data.mag = mag;

#ifdef IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE
            m_mahony.Update(m_data.gyro, m_data.acc, m_data.mag);
#else
            m_mahony.Update(m_data.gyro, m_data.acc);
#endif
            m_data.quat = m_mahony.GetOrientationAsQuaternion();
            m_data.rpy = m_mahony.GetOrientationAsRPYAngles();

            m_dataMutex.unlock();
        }

#if IMU_DO_PRINTF
        
        float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() * 1.0e-3f;
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, ", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                               m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                               m_data.mag(0) , m_data.mag(1) , m_data.mag(2) , time_ms );
        printf("%.6f, %.6f, %.6f, %.6f, ", m_data.quat.w(), m_data.quat.x(), m_data.quat.y(), m_data.quat.z() );
        printf("%.6f, %.6f, %.6f\n", m_data.rpy(0), m_data.rpy(1), m_data.rpy(2) );
#endif
    }
}

void IMUThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}