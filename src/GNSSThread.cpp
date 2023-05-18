#include "GNSSThread.h"
#include <chrono>
#include <cstdio>

GNSSThread::GNSSThread(Data& data) :
    m_GNSS(GNSS_TX, GNSS_RX, data),
    m_data(data),
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

        //static Timer timer;
        //timer.start();

        m_GNSS.readGNSSdata();

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