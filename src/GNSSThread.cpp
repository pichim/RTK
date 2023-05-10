#include "GNSSThread.h"
#include <chrono>
#include <cstdio>

GNSSThread::GNSSThread(Data& data) :
    m_GNSS(GNSS_TX, GNSS_RX, data),
    m_data(data),
    m_gnss_fix_led(GNSS_FIX_LED),
    m_rtk_fix_led(GNSS_RTK_FIX_LED),
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
    m_gnss_fix_led = 0;
    m_rtk_fix_led = 0;


    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);

        //static Timer timer;
        //timer.start();

        m_GNSS.readGNSSdata();

        if(m_data.rtk_fix){
            m_rtk_fix_led = 1;
        } else if(m_data.rtk_float){
            m_rtk_fix_led = !m_rtk_fix_led; //blink at whatever rate the system is running
        } else {
            m_rtk_fix_led = 0;
        }
        
        if(m_data.gnss_fix){
            m_gnss_fix_led = 1;
        } else {
            m_gnss_fix_led = 0;
        }


#if GNSS_DO_PRINTF
        // printf's here
        // printf("meanAcc = %f\n", m_data.meanAcc_SVIN);
        // printf("itow = %u\n", m_data.itow);
        // printf("msss = %u ms\n",m_data.msss);

#endif
        
    }
}

void GNSSThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}