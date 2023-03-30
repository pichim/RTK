#include "SDCardThread.h"
#include <chrono>

SDCardThread::SDCardThread(Data& data) :
    m_data(data),
    m_additional_led(SDCARD_PIN_ADDITIONAL_LED),
    m_thread(SDCARD_THREAD_PRIORITY, SDCARD_THREAD_SIZE)
{
    if (!m_sd.init()) {
        //if this fails all operations will be ignored (in case you wanna use it without sd card)
        printf("SD init failed\n");
        m_additional_led = 0;
    } else {
        m_additional_led = 1;
    }
    m_data = data;
}

SDCardThread::~SDCardThread()
{
    m_ticker.detach();
}

void SDCardThread::StartThread()
{
    m_thread.start(callback(this, &SDCardThread::run));
    m_ticker.attach(callback(this, &SDCardThread::sendThreadFlag), std::chrono::milliseconds{ static_cast<long int>( SDCARD_THREAD_TS_MS ) });
}

void SDCardThread::CloseFile() {
    m_sd.close();
}

void SDCardThread::run()
{
    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);
#if SDCARD_DO_PRINTF
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                         m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                         m_data.mag(0) , m_data.mag(1) , m_data.mag(2) );
#endif
        char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
        m_sd.write2sd(rover_header);
    }
}

void SDCardThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}