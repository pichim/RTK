#include "GNSSThread.h"
#include <chrono>
#include <cstdio>

GNSSThread::GNSSThread(Data& data) :
    m_GNSS(GNSS_TX, GNSS_RX),
    m_data(data),
    m_additional_led(SDCARD_PIN_ADDITIONAL_LED),
    m_thread(GNSS_THREAD_PRIORITY, GNSS_THREAD_SIZE)
{
    
}

GNSSThread::~GNSSThread()
{
    m_ticker.detach();
}

void GNSSThread::StartThread()
{
    m_thread.start(callback(this, &GNSSThread::run));
    m_ticker.attach(callback(this, &GNSSThread::sendThreadFlag), std::chrono::milliseconds{ static_cast<long int>( GNSS_THREAD_TS_MS ) });
}

void GNSSThread::run()
{   
    


    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);

        static Timer timer;
        static uint32_t msg_length;
        timer.start();

        m_GNSS.readGNSSdata();


      





#if GNSS_DO_PRINTF
        // printf's here
#endif
        
    }
}

void GNSSThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}