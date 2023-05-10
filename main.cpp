/*
 * notes:
 * - acc calibration should also be done static while system is level (offset x and y)
 * 
 * 
*/
#include <mbed.h>

#include "Param.h"
#include "DebounceIn.h"
#include "SDCardThread.h"
#include "IMUThread.h"
#include "GNSSThread.h"

DebounceIn user_button(PC_13);
void user_button_pressed_fcn();
bool do_close_sd_file = false;
DigitalOut user_led(LED1);

int main(){

    user_button.fall(&user_button_pressed_fcn);

    Data data;
    printf("program Start\n");

    
    
    IMUThread imuThread(data);
    ThisThread::sleep_for(1s);
    SDCardThread sdCardThread(data);
    GNSSThread GNSSThread(data);

    GNSSThread.StartThread();
    imuThread.StartThread();
    sdCardThread.StartThread();

    ThisThread::sleep_for(1s);
    sdCardThread.OpenFile();

    while(true) {

        if(data.gnss_time_valid && data.gnss_fix) {
            //
        }
        
        if (do_close_sd_file) {

            sdCardThread.CloseFile();
        }
        user_led = !user_led;
        
        //printf("hpposllh = %3.9f, %3.9f, %4.4f\n",data.longitude_hp,data.latitude_hp,data.hpLlh(2));
        //printf("relposned = %4.4f, %4.4f, %4.4f\n", data.relPosNED(0), data.relPosNED(1), data.relPosNED(2));
        //printf("pvt = %3.7f, %3.7f, %4.4f, vel= %4.3f, %4.3f, %4.3f\n",data.llh(0),data.llh(1),data.llh(2),data.velNED(0),data.velNED(1),data.velNED(2));
        //printf("dop = %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\n", data.gDOP,data.pDOP,data.tDOP,data.vDOP,data.hDOP);
        printf("cov = %f, %f, %f, %f\n", data.posCovNN, data.posCovNE, data.posCovND, data.posCovEE);
        
        thread_sleep_for(500);
    }
}

void user_button_pressed_fcn()
{
    if (!do_close_sd_file) do_close_sd_file = !do_close_sd_file;
}
