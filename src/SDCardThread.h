#ifndef SDCARD_THREAD_H_
#define SDCARD_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "SDCard.h"

class SDCardThread
{
public:
    SDCardThread(Data& data);
    virtual ~SDCardThread();

    void StartThread();
    void CloseFile();
    
private:
    Data& m_data;
    DigitalOut m_additional_led;
    SDCard m_sd;
    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    uint8_t* m_buffer;
    
    void run();
    void sendThreadFlag();
};

#endif /* SDCARD_THREAD_H_ */
