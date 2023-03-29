#ifndef SDCARD_THREAD_H_
#define SDCARD_THREAD_H_

#include <mbed.h>

#include "param.h"
#include "ThreadFlag.h"
#include "SDCard.h"

class SDCardThread
{
public:
    SDCardThread(Data_t& data);
    virtual ~SDCardThread();

    void StartThread();
    void CloseFile();
    
private:
    Data_t& m_data;
    SDCard m_sd;
    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* SDCARD_THREAD_H_ */
