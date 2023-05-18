#ifndef GNSS_THREAD_H_
#define GNSS_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "GNSS.h"

class GNSSThread
{
public:
    GNSSThread(Data& data, Mutex& _mutex_1);
    virtual ~GNSSThread();

    void StartThread();
    void run();
    
private:
    GNSS m_GNSS;
    Data& m_data;
    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    void sendThreadFlag();
};

#endif /* GNSS_THREAD_H_ */
