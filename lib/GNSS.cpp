#include "GNSS.h"

#define BUFFER_SIZE 512


GNSS::GNSS(PinName rx, PinName tx, Data& data) :
    m_uart(rx, tx),
    m_data(data)
{
    init();

}

uint8_t GNSS::decode()
{
    int i = 0;

    while(m_msg[i].is_valid){

        switch(m_msg[i].id){

            case(0x3b):

                m_data.itow = ((uint32_t)m_msg[i].data[4]) | ((uint32_t)m_msg[i].data[5] << 8) | ((uint32_t)m_msg[i].data[6] << 16) | ((uint32_t)m_msg[i].data[7] << 24);       //itow
                m_data.base_svin_valid = m_msg[i].data[36];                                                                     //see if base has succesfully completed survey-in
                m_data.meanAcc_SVIN = (float)(((uint32_t)m_msg[i].data[28]) | ((uint32_t)m_msg[i].data[29] << 8) | ((uint32_t)m_msg[i].data[30] << 16) | ((uint32_t)m_msg[i].data[31] << 24)) / 10000.0;   // progress
                
            break;

            case(0x21):

                m_data.gnss_year = ((uint32_t)m_msg[i].data[12]) | ((uint32_t)m_msg[i].data[13] << 8);
                m_data.gnss_month = ((uint32_t)m_msg[i].data[14]);
                m_data.gnss_day = ((uint32_t)m_msg[i].data[15]);
                m_data.gnss_hour = ((uint32_t)m_msg[i].data[16]);
                m_data.gnss_minutes = ((uint32_t)m_msg[i].data[17]);
                m_data.gnss_seconds = ((uint32_t)m_msg[i].data[18]);
                m_data.gnss_time_valid = ((uint32_t)m_msg[i].data[19] >> 2) & 0x01;


                if(m_data.gnss_time_valid){
                    set_time(date2sec(m_data.gnss_year, m_data.gnss_month, m_data.gnss_day, m_data.gnss_hour, m_data.gnss_minutes, m_data.gnss_seconds));  // Set RTC time
                }

            break;
            //
            default:
#if GNSS_DO_PRINTF
            printf("msg not supported: 0x%x\n",m_msg[i].id);
#endif
        }
        m_msg[i].reset();
        i++;
    }
    
    m_msg_index = 0;

    return i;
}



uint8_t GNSS::readGNSSdata()
{

    static char buffer[BUFFER_SIZE];
    static int msg_length = 0;

    msg_length = m_uart.read(buffer, BUFFER_SIZE);
    if(msg_length <= 0) return 0;


    static uint16_t header_ref = 0xB562; //
    static uint16_t header = 0;
    static uint8_t ck_a = 0;
    static uint8_t ck_b = 0;
    int i = 0;
    int offset = 0;
    int32_t remaining_bytes = msg_length;
    bool loop = true;

    while(remaining_bytes){
        header = (uint16_t)buffer[offset + 0] << 8 | (uint16_t)buffer[offset + 1];
        //printf("header = 0x%04x\n", header);
        if(header == header_ref){ //its a valid message
            m_msg[m_msg_index].is_arriving = true;
            m_msg[m_msg_index].header = header;
            header = 0;
            m_msg[m_msg_index].class_ = buffer[offset + 2];
            m_msg[m_msg_index].id = buffer[offset + 3];
            m_msg[m_msg_index].length = (uint16_t)buffer[offset + 5] << 8 | (uint16_t)buffer[offset + 4];

            for(i = 0; i < m_msg[m_msg_index].length; i++){
                m_msg[m_msg_index].data[i] = buffer[offset + 6 + i];
            }

            m_msg[m_msg_index].ck_a = buffer[offset + 6 + i + 1];
            m_msg[m_msg_index].ck_b = buffer[offset + 6 + i + 2];

            /*
            if(m_msg[m_msg_index].id == 0x3b){
                m_msg[m_msg_index].checksum();
            }
            */
            m_msg[m_msg_index].is_arriving = false;

            offset = offset + 5 + i + 3;
            remaining_bytes -= offset;
            
            //printf("\n\nmsg_index = %i, id = 0x%x, l = %i\n",m_msg_index, m_msg[m_msg_index].id, m_msg[m_msg_index].length);

            // checksum(m_msg_index); doesnt currently work so its expected to be correct
            m_msg[m_msg_index].is_valid = true;


            m_msg_index++;
            if(m_msg_index >= 10){ //reached max messages
                m_msg_index = 0; // if more messages come the will be overwritten
            }

            

            if(remaining_bytes < 0){ // some bytes are missing
                m_msg[m_msg_index].is_valid = false;
                remaining_bytes = 0; // force loop to end
                break;
            }

        }else{offset++; remaining_bytes--;}


    }
    
    /*
    if(msg_length >= 0){
        for(int i = 0; i < msg_length; i++){
            printf(" %02x",buffer[i]);
        }
        printf("\nread %i bytes\n",msg_length);
    }
    */
    
    return decode();
}


bool GNSS::init()
{
    m_uart.set_baud(38400); //in the future as a parameter
    m_uart.set_blocking(false);
    m_uart.set_format(8,BufferedSerial::None,1);

    //m_uart.attach(callback(this,&GNSS::read_uart),SerialBase::RxIrq);

    m_msg_index = 0;
    return true;
}

bool GNSS::checksum(int i){
    uint8_t a = 0;
    uint8_t b = 0;

    a += m_msg[i].class_;
    b += a;

    a += m_msg[i].id;
    b += a;

    a += (uint8_t)(m_msg[i].length >> 8);
    b += a;

    a += (uint8_t)m_msg[i].length;
    b += a;

    for(int i = 0; i < m_msg[i].length; i++){
        a += m_msg[i].data[i];
        b += a;
    }
    /*
    printf("my ckecksum = 0x%02x , 0x%02x\n",a,b);
    printf("desired cs  = 0x%02x , 0x%02x\n",m_msg[i].ck_a,m_msg[i].ck_b);
    */
    if((m_msg[i].ck_a == a) && (m_msg[i].ck_b == b)){
        m_msg[i].is_valid = true;
        return true;
    }else{
        return false;
    }

    return false;
}


time_t GNSS::date2sec(uint16_t yyyy_, uint8_t mm_, uint8_t dd_, uint8_t hh_, uint8_t min_, uint8_t ss_){
  
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
