#ifndef IMU_THREAD_H_
#define IMU_THREAD_H_

#include <mbed.h>

#include "param.h"
#include "ThreadFlag.h"
#include "LSM9DS1_i2c.h"

class IMUThread
{
public:
    IMUThread(Data_t& data);
    virtual ~IMUThread();

    void StartThread();
    
private:
    Data_t& m_data;
    LSM9DS1 m_imu;

    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* IMU_THREAD_H_ */
