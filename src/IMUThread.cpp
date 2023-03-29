#include "IMUThread.h"
#include <chrono>

IMUThread::IMUThread(Data_t& data) :
    m_data(data),
    m_imu(IMU_SDA, IMU_SCL),
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
    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);
        m_imu.updateGyro();
        m_imu.updateAcc();
        m_imu.updateMag();
        m_data.gyro << m_imu.readGyroX(), m_imu.readGyroY(), m_imu.readGyroZ();
        m_data.acc << m_imu.readAccX() , m_imu.readAccY() , m_imu.readAccZ();
        m_data.mag << m_imu.readMagX() , m_imu.readMagY() , m_imu.readMagZ();
#if IMU_THREAD_PRINTF
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\r\n", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                           m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                           m_data.mag(0) , m_data.mag(1) , m_data.mag(2) );
#endif
    }
}

void IMUThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}