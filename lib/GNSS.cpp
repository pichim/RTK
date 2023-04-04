#include "GNSS.h"

#define BUFFER_SIZE 128

GNSS::GNSS(PinName rx, PinName tx) 
    :m_uart(rx, tx)
{
    init();
}


uint32_t GNSS::read(){

    static char buf[BUFFER_SIZE];
    static int count = 0;
    static bool is_ubx_msg = false;


    count = m_uart.read(buf, BUFFER_SIZE);

    if (count > 0){
        for(int i = 0; i > count; i++){
            
        }
    }


    return 0;
}


bool GNSS::init(){
    m_uart.set_baud(57600); //in the future as a parameter
    m_uart.set_blocking(false);
    m_uart.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );


    return true;
}