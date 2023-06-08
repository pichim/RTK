#ifndef SDCARD_THREAD_H_
#define SDCARD_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "SDCard.h"

class SDCardThread
{
public:
    SDCardThread(Data& data, Mutex& mutex_1, Mutex& mutex_2);
    virtual ~SDCardThread();

    void StartThread();
    void OpenFile();
    void CloseFile();
    
private:
    Data& m_data;
    Mutex& m_dataMutex_1;
    Mutex& m_dataMutex_2;
    SDCard m_sd;
    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    //uint8_t* m_buffer;
    
    bool m_sdThread_running;
    bool m_sd_present;
    bool m_file_open;
    void run();
    void sendThreadFlag();
};

#endif /* SDCARD_THREAD_H_ */
