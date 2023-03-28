/*
main branch for RTK-GPS implenetation for Drone


*/

#include "mbed.h"
#include "SD_interface.h"



time_t date2sec(uint16_t yyyy_, uint8_t mm_, uint8_t dd_, uint8_t hh_, uint8_t min_, uint8_t ss_){
  
    struct tm t;
    time_t t_of_day;

    t.tm_year = yyyy_-1900;  // Year - 1900
    t.tm_mon = mm_-1;           // Month, where 0 = jan
    t.tm_mday = dd_;          // Day of the month
    t.tm_hour = hh_;
    t.tm_min = min_;
    t.tm_sec = ss_;
    t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);

    return t_of_day;
}

int main(){
    //
    set_time(date2sec(2023, 3, 25, 13, 55, 40));  // Set RTC time

    SDCARD sd;
    if(!sd.init()){
        printf("SD init failed\n"); //if this fails all operations will be ignored(in case you wanna use it without sd card)
    }
    // define a header to know what values go where
    char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
    sd.write2sd(rover_header);
    sd.close();

    while(1){
        ThisThread::sleep_for(1s);
    }
}
