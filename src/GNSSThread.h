#ifndef GNSS_THREAD_H_
#define GNSS_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "GNSS.h"

class GNSSThread
{
public:
    GNSSThread(Data& data);
    virtual ~GNSSThread();

    void StartThread();
    
private:
    GNSS m_GNSS;
    Data& m_data;
    DigitalOut m_additional_led;
    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* GNSS_THREAD_H_ */