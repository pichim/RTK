/**
 * notes:
 * - acc calibration should also be done static while system is level (offset x and y)
 * - all unnecesarry printf commands need to vanish or are to wrap inside a compiler command for debugging
 * - 
*/

#include <mbed.h>

#include "param.h"
#include "DebounceIn.h"
#include "SDCardThread.h"
#include "IMUThread.h"

DebounceIn user_button(PC_13);
void user_button_pressed_fcn();
bool do_close_sd_file = false;
DigitalOut user_led(LED1);

int main() {

    user_button.fall(&user_button_pressed_fcn);

    Data data;
    SDCardThread sdCardThread(data);
    IMUThread imuThread(data);

    sdCardThread.StartThread();
    imuThread.StartThread();

    while(true) {

        if (do_close_sd_file) {

            sdCardThread.CloseFile();
        }
        user_led = !user_led;
        
        thread_sleep_for(500);
    }
}

void user_button_pressed_fcn()
{
    if (!do_close_sd_file) do_close_sd_file = !do_close_sd_file;
}

/*
set_time(date2sec(2023, 3, 29, 14, 30, 0));  // Set RTC time
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
*/