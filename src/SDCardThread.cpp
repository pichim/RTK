#include "SDCardThread.h"
#include <chrono>
#include <cstdio>

SDCardThread::SDCardThread(Data& data) :
    m_data(data),
    m_thread(SDCARD_THREAD_PRIORITY, SDCARD_THREAD_SIZE),
    m_led(PB_15)
{
    m_sdThread_running = false;
    m_data = data;
    //m_buffer = (uint8_t*)malloc(SDCARD_BUFFER_SIZE);
    if(!m_sd.init()) {
            //if this fails all operations will be ignored (in case you wanna use it without sd card)
            printf("SD init failed\n");
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

void SDCardThread::CloseFile() {
    m_sd.close();
    //free(m_buffer);
}

void SDCardThread::run()
{

    static float buffer_f[9+1+12+7+3+3+5*3]; //=50 / 200 bytes
    static int i_f = 0;
    static uint32_t buffer_u32[4+2]; //3 from param.h and 2 from time and index =6 / 24 bytes
    static int i_u32 = 0;
    static uint8_t buffer_u8[4]; //4 bytes
    static int i_u8 = 0;
    static bool buffer_b[8]; //8 bytes
    static int i_b = 0;
    static Timer timer;

    while(true) {
        ThisThread::flags_wait_any(m_threadFlag);
        timer.start();
        

        i_f = 0;
        i_u32 = 0;
        i_u8 = 0;
        i_b = 0;
        
        
        buffer_u32[i_u32++] = std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
        buffer_u32[i_u32++]++;
        

#if SDCARD_DO_PRINTF
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n", m_data.gyro(0), m_data.gyro(1), m_data.gyro(2),
                                                                         m_data.acc(0) , m_data.acc(1) , m_data.acc(2) ,
                                                                         m_data.mag(0) , m_data.mag(1) , m_data.mag(2) );
#endif
        
        
        buffer_f[i_f++] = m_data.gyro(0);
        buffer_f[i_f++] = m_data.gyro(1);
        buffer_f[i_f++] = m_data.gyro(2);
        buffer_f[i_f++] = m_data.acc(0);
        buffer_f[i_f++] = m_data.acc(1);
        buffer_f[i_f++] = m_data.acc(2);
        buffer_f[i_f++] = m_data.mag(0);
        buffer_f[i_f++] = m_data.mag(1);
        buffer_f[i_f++] = m_data.mag(2);
        
        buffer_u32[i_u32++] = m_data.itow;

        //UBX-NAV-SVIN (BASE)
        buffer_b[i_b++] = m_data.base_time_mode;
        buffer_b[i_b++] = m_data.base_svin_valid;

        buffer_f[i_f++] = m_data.meanAcc_SVIN;

        //UBX-NAV-COV
        buffer_b[i_b++] = m_data.posCOVvalid;
        buffer_b[i_b++] = m_data.velCOVvalid;

        buffer_f[i_f++] = m_data.posCovNN;
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
        buffer_f[i_f++] = m_data.velCovDD;

        //UBX-NAV-DOP
        buffer_f[i_f++] = m_data.gDOP;
        buffer_f[i_f++] = m_data.pDOP;
        buffer_f[i_f++] = m_data.tDOP;
        buffer_f[i_f++] = m_data.vDOP;
        buffer_f[i_f++] = m_data.hDOP;
        buffer_f[i_f++] = m_data.nDOP;
        buffer_f[i_f++] = m_data.eDOP;

        //UBX-NAV-STATUS
        buffer_u8[i_u8++] = m_data.gnss_fix;

        buffer_b[i_b++] = m_data.diffCorr_available;
        buffer_b[i_b++] = m_data.rtk_float;
        buffer_b[i_b++] = m_data.rtk_fix;

        buffer_u32[i_u32++] = m_data.ttff;
        buffer_u32[i_u32++] = m_data.msss;

        //UBX-NAV-RELPOSNED
        buffer_f[i_f++] = m_data.relPosNED(0);
        buffer_f[i_f++] = m_data.relPosNED(1);
        buffer_f[i_f++] = m_data.relPosNED(2);
        buffer_f[i_f++] = m_data.accNED(0);
        buffer_f[i_f++] = m_data.accNED(1);
        buffer_f[i_f++] = m_data.accNED(2);

        buffer_u32[i_u32++] = m_data.rtk_flags;

        //UBX-NAV-HPPOSLLH
        buffer_b[i_b++] = m_data.invalidLLH;

        buffer_f[i_f++] = m_data.hpLlh(0);
        buffer_f[i_f++] = m_data.hpLlh(1);
        buffer_f[i_f++] = m_data.hpLlh(2);
        buffer_f[i_f++] = m_data.hMSL;
        buffer_f[i_f++] = m_data.hAcc;
        buffer_f[i_f++] = m_data.vAcc;

        //UBX-NAV-PVT
        buffer_u8[i_u8++] = m_data.fix_type;
        buffer_u8[i_u8++] = m_data.numSV;

        buffer_f[i_f++] = m_data.llh(0);
        buffer_f[i_f++] = m_data.llh(1);
        buffer_f[i_f++] = m_data.llh(2);
        buffer_f[i_f++] = m_data.velNED(0);
        buffer_f[i_f++] = m_data.velNED(1);
        buffer_f[i_f++] = m_data.velNED(2);
        buffer_f[i_f++] = m_data.sAcc;
        buffer_f[i_f++] = m_data.gSpeed;
        buffer_f[i_f++] = m_data.headMotion;

        buffer_u8[i_u8++] = m_data.lastCorrectionAge;

        //char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
        //m_sd.write2sd(buffer_c,data_length);
        //printf("indexed: f = %i, u32 = %i, u8 = %i, b = %i, Total Bytes = %i\n", i_f, i_u32, i_u8, i_b, (i_f*4+i_u32*4+i_u8+i_b));
	    m_sd.write_f_2_sd(buffer_f, i_f);
        m_sd.write_u32_2_sd(buffer_u32, i_u32);
        m_sd.write_u8_2_sd(buffer_u8, i_u8);
        m_sd.write_bool_2_sd(buffer_b, i_b);
    }
}

void SDCardThread::sendThreadFlag()
{
    m_thread.flags_set(m_threadFlag);
}