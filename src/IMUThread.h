#ifndef IMU_THREAD_H_
#define IMU_THREAD_H_

#include <mbed.h>

#include "param.h"
#include "ThreadFlag.h"
#include "LSM9DS1_i2c.h"
#include "LinearCharacteristics3.h"
#include "MahonyRP.h"

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
    MahonyRP m_mahonyRP;

    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* IMU_THREAD_H_ */
