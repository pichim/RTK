#ifndef IMU_THREAD_H_
#define IMU_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "LSM9DS1_i2c.h"
#include "LinearCharacteristics3.h"

class IMUThread
{
public:
    IMUThread(Data& data);
    virtual ~IMUThread();

    void StartThread();
    
private:
    Data& m_data;
    LSM9DS1 m_imu;
    LinearCharacteristics3 m_magCalib;

    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* IMU_THREAD_H_ */
