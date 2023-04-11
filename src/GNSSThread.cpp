#include "GNSSThread.h"
#include <chrono>
#include <cstdio>

GNSSThread::GNSSThread(Data& data) :
    m_GNSS(GNSS_TX, GNSS_RX, data),
    m_data(data),
    m_status_led(D14),
    m_progress_led(D15),
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
    m_status_led = 0;
    m_progress_led = 0;


    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);

        static Timer timer;
        static uint32_t msg_length;
        timer.start();

        m_GNSS.readGNSSdata();
        
        if (m_data.base_svin_valid){
            m_status_led = 1;
        } else{
            m_status_led = 0;
        }


        if(m_data.meanAcc_SVIN > 15.0){
            m_progress_led = 0;
        } else if (m_data.meanAcc_SVIN > 5.0 && m_data.meanAcc_SVIN < 15.0){
            m_progress_led = !m_progress_led;
        } else if (m_data.meanAcc_SVIN > 1.0 && m_data.meanAcc_SVIN < 5.0) {
            m_progress_led = 1;
        }


#if GNSS_DO_PRINTF
        // printf's here
        //printf("meanAcc = %f\n", m_data.meanAcc_SVIN);
        printf("itow = %u\n", m_data.itow);
#endif
        
    }
}

void GNSSThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}