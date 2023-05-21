#include "SDCardThread.h"
#include <chrono>
#include <cstdio>

SDCardThread::SDCardThread(Data& data, Mutex& mutex_1, Mutex& mutex_2) :
    m_data(data),
    m_thread(SDCARD_THREAD_PRIORITY, SDCARD_THREAD_SIZE),
    m_dataMutex_1(mutex_1),
    m_dataMutex_2(mutex_2)
{
    m_sdThread_running = false;
    m_sd_present = false;
    m_data = data;
    //m_buffer = (uint8_t*)malloc(SDCARD_BUFFER_SIZE);
    if(!m_sd.init()) {
            //if this fails all operations will be ignored (in case you wanna use it without sd card)
            printf("SD init failed\n");
            m_sd_present = false;
    } else {
        m_sd_present = true;
    }
}

SDCardThread::~SDCardThread()
{
    m_ticker.detach();
}

void SDCardThread::StartThread()
{
    if(m_sdThread_running) return;
    
    m_thread.start(callback(this, &SDCardThread::run));
    m_ticker.attach(callback(this, &SDCardThread::sendThreadFlag), std::chrono::milliseconds{ static_cast<long int>( SDCARD_THREAD_TS_MS ) });

    m_sdThread_running = true;
}
void SDCardThread::OpenFile() {
    static bool writing = false;

    if(writing) return;

    m_sd.mkfile();
    writing = true;
}
void SDCardThread::CloseFile() {
    m_sd.close();
    //free(m_buffer);
}

void SDCardThread::run()
{

    float buffer_f[9+1+12+7+3+3+5*3]; //=50 / 200 bytes
    double buffer_d[3];
    uint32_t buffer_u32[4+2]; //3 from param.h and 2 from time and index =6 / 24 bytes
    uint8_t buffer_u8[4]; //4 bytes
    bool buffer_b[8]; //8 bytes

    int i_f;
    int i_d;
    int i_u32;
    int i_u8;
    int i_b;

    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);
        
        
        i_f = 0;
        i_d = 0;
        i_u32 = 0;
        i_u8 = 0;
        i_b = 0;

        static Timer timer;
        timer.start();

        if(m_sd_present){
        
            buffer_u32[i_u32++] = m_data.t;
            buffer_u32[i_u32++]++;
            

#if SDCARD_DO_PRINTF
            printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                            m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                            m_data.mag(0) , m_data.mag(1) , m_data.mag(2) );
#endif
            m_dataMutex_1.lock();

            buffer_f[i_f++] = m_data.gyro(0); //1
            buffer_f[i_f++] = m_data.gyro(1);
            buffer_f[i_f++] = m_data.gyro(2);
            buffer_f[i_f++] = m_data.acc(0);
            buffer_f[i_f++] = m_data.acc(1);
            buffer_f[i_f++] = m_data.acc(2);
            buffer_f[i_f++] = m_data.mag(0);
            buffer_f[i_f++] = m_data.mag(1);
            buffer_f[i_f++] = m_data.mag(2);
            buffer_f[i_f++] = m_data.quat.w();
            buffer_f[i_f++] = m_data.quat.x();
            buffer_f[i_f++] = m_data.quat.y();
            buffer_f[i_f++] = m_data.quat.z();
            buffer_f[i_f++] = m_data.rpy(0);
            buffer_f[i_f++] = m_data.rpy(1);
            buffer_f[i_f++] = m_data.rpy(2); //16

            m_dataMutex_1.unlock();

            m_dataMutex_2.lock();

            buffer_u32[i_u32++] = m_data.itow;

            //UBX-NAV-SVIN (BASE)
            buffer_b[i_b++] = m_data.base_time_mode;
            buffer_b[i_b++] = m_data.base_svin_valid;

            buffer_f[i_f++] = m_data.meanAcc_SVIN; //17

            //UBX-NAV-COV
            buffer_b[i_b++] = m_data.posCOVvalid;
            buffer_b[i_b++] = m_data.velCOVvalid;

            buffer_f[i_f++] = m_data.posCovNN; //18
            buffer_f[i_f++] = m_data.posCovNE;
            buffer_f[i_f++] = m_data.posCovND;
            buffer_f[i_f++] = m_data.posCovEE;
            buffer_f[i_f++] = m_data.posCovED;
            buffer_f[i_f++] = m_data.posCovDD;
            buffer_f[i_f++] = m_data.velCovNN;
            buffer_f[i_f++] = m_data.velCovNE;
            buffer_f[i_f++] = m_data.velCovND;
            buffer_f[i_f++] = m_data.velCovEE;
            buffer_f[i_f++] = m_data.velCovED;
            buffer_f[i_f++] = m_data.velCovDD; //29

            //UBX-NAV-DOP
            buffer_f[i_f++] = m_data.gDOP; //30
            buffer_f[i_f++] = m_data.pDOP;
            buffer_f[i_f++] = m_data.tDOP;
            buffer_f[i_f++] = m_data.vDOP;
            buffer_f[i_f++] = m_data.hDOP;
            buffer_f[i_f++] = m_data.nDOP;
            buffer_f[i_f++] = m_data.eDOP; //36

            //UBX-NAV-STATUS
            buffer_u8[i_u8++] = m_data.gnss_fix;

            buffer_b[i_b++] = m_data.diffCorr_available;
            buffer_b[i_b++] = m_data.rtk_float;
            buffer_b[i_b++] = m_data.rtk_fix;

            buffer_u32[i_u32++] = m_data.ttff;
            buffer_u32[i_u32++] = m_data.msss;

            //UBX-NAV-RELPOSNED
            buffer_f[i_f++] = m_data.relPosNED(0); //37
            buffer_f[i_f++] = m_data.relPosNED(1);
            buffer_f[i_f++] = m_data.relPosNED(2);
            buffer_f[i_f++] = m_data.accNED(0);
            buffer_f[i_f++] = m_data.accNED(1);
            buffer_f[i_f++] = m_data.accNED(2); //42

            buffer_u32[i_u32++] = m_data.rtk_flags;

            //UBX-NAV-HPPOSLLH
            buffer_b[i_b++] = m_data.invalidLLH;

            buffer_d[i_d++] = m_data.hpLlh(0); //1
            buffer_d[i_d++] = m_data.hpLlh(1);
            buffer_d[i_d++] = m_data.hpLlh(2); //3
            buffer_f[i_f++] = m_data.hMSL; //43
            buffer_f[i_f++] = m_data.hAcc;
            buffer_f[i_f++] = m_data.vAcc; //45

            //UBX-NAV-PVT
            buffer_u8[i_u8++] = m_data.fix_type;
            buffer_u8[i_u8++] = m_data.numSV;

            buffer_f[i_f++] = m_data.llh(0); //46
            buffer_f[i_f++] = m_data.llh(1);
            buffer_f[i_f++] = m_data.llh(2);
            buffer_f[i_f++] = m_data.velNED(0);
            buffer_f[i_f++] = m_data.velNED(1);
            buffer_f[i_f++] = m_data.velNED(2);
            buffer_f[i_f++] = m_data.sAcc;
            buffer_f[i_f++] = m_data.gSpeed;
            buffer_f[i_f++] = m_data.headMotion; //54

            buffer_u8[i_u8++] = m_data.lastCorrectionAge;

            m_dataMutex_2.unlock();

            //char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
            //m_sd.write2sd(buffer_c,data_length);
            //printf("indexed: f = %i, d = %i, u32 = %i, u8 = %i, b = %i, Total Bytes = %i\n", i_f, i_d, i_u32, i_u8, i_b, (i_f*4+i_d*8+i_u32*4+i_u8+i_b));

            //m_sd.sdMutexLock();

            m_sd.write_f_2_sd(buffer_f, i_f);
            m_sd.write_d_2_sd(buffer_d, i_d);
            m_sd.write_u32_2_sd(buffer_u32, i_u32);
            m_sd.write_u8_2_sd(buffer_u8, i_u8);
            m_sd.write_bool_2_sd(buffer_b, i_b);

            //m_sd.sdMutexUnlock();
        }
    }
}

void SDCardThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}