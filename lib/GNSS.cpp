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

#if GNSS_DO_PRINTF
        printf("msg id = 0x%02x \n",m_msg[i].id);

#endif

        switch(m_msg[i].id){

            case(0x3b): //UBX-NAV-SVIN
            { //please dont mind these curly braces, removing these leads to errors
                m_data.itow = ((uint32_t)m_msg[i].data[4]) | ((uint32_t)m_msg[i].data[5] << 8) | ((uint32_t)m_msg[i].data[6] << 16) | ((uint32_t)m_msg[i].data[7] << 24);       //itow
                m_data.base_svin_valid = m_msg[i].data[36];                                                                     //see if base has succesfully completed survey-in
                m_data.meanAcc_SVIN = (float)(((uint32_t)m_msg[i].data[28]) | ((uint32_t)m_msg[i].data[29] << 8) | ((uint32_t)m_msg[i].data[30] << 16) | ((uint32_t)m_msg[i].data[31] << 24)) / 10000.0;   // progress

            break;
            }
            case(0x21):
            {
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
            }
            case(0x03): //UBX-NAV-STATUS
            {
                m_data.itow =      (uint32_t)m_msg[i].data[0]
                                | ((uint32_t)m_msg[i].data[1] << 8)
                                | ((uint32_t)m_msg[i].data[2] << 16)
                                | ((uint32_t)m_msg[i].data[3] << 24);

                m_data.gnss_fix = m_msg[i].data[4];
                m_data.diffCorr_available = m_msg[i].data[6] & 0x01;
                m_data.rtk_float = m_msg[i].data[7] >> 6 & 0x01;
                m_data.rtk_fix = m_msg[i].data[7] >> 7 & 0x01;

                m_data.ttff =      (uint32_t)m_msg[i].data[8]
                                | ((uint32_t)m_msg[i].data[9] << 8)
                                | ((uint32_t)m_msg[i].data[10] << 16)
                                | ((uint32_t)m_msg[i].data[11] << 24);

                m_data.msss =      (uint32_t)m_msg[i].data[12]
                                | ((uint32_t)m_msg[i].data[13] << 8)
                                | ((uint32_t)m_msg[i].data[14] << 16)
                                | ((uint32_t)m_msg[i].data[15] << 24);

            break;
            }            
            case(0x14): //UBX-NAV-HPPOSLLH
            {
                m_data.invalidLLH = m_msg[i].data[3] & 0x01; //invalid LLH
                m_data.itow =      (uint32_t)m_msg[i].data[4]
                                | ((uint32_t)m_msg[i].data[5] << 8)
                                | ((uint32_t)m_msg[i].data[6] << 16)
                                | ((uint32_t)m_msg[i].data[7] << 24); //itow

                uint32_t temp1 =   (uint32_t)m_msg[i].data[8]
                                | ((uint32_t)m_msg[i].data[9] << 8)
                                | ((uint32_t)m_msg[i].data[10] << 16)
                                | ((uint32_t)m_msg[i].data[11] << 24); //lon

                uint32_t temp2 =   (uint32_t)m_msg[i].data[12]
                                | ((uint32_t)m_msg[i].data[13] << 8)
                                | ((uint32_t)m_msg[i].data[14] << 16)
                                | ((uint32_t)m_msg[i].data[15] << 24); //lat

                float lon = (float)temp1 / 10000000.0f;    // [deg]
                float lat = (float)temp2 / 10000000.0f;    // [deg]

                uint32_t temp3 =   (uint32_t)m_msg[i].data[16]
                                | ((uint32_t)m_msg[i].data[17] << 8)
                                | ((uint32_t)m_msg[i].data[18] << 16)
                                | ((uint32_t)m_msg[i].data[19] << 24); //height

                float height = (float)temp3 / 1000;     // [m]

                uint32_t temp4 =   (uint32_t)m_msg[i].data[20]
                                | ((uint32_t)m_msg[i].data[21] << 8)
                                | ((uint32_t)m_msg[i].data[22] << 16)
                                | ((uint32_t)m_msg[i].data[23] << 24); //hMSL

                float hMSL = (float)temp4 / 1000;     // [m]

                m_data.hpLlh <<     (lon + ((float)(int8_t)m_msg[i].data[24] / 1000000000.0f)),
                                    (lat + ((float)(int8_t)m_msg[i].data[25] / 1000000000.0f)),
                                    (height + ((float)(int8_t)m_msg[i].data[26]) / 10000.0f);

                m_data.hMSL = (hMSL + ((float)(int8_t)m_msg[i].data[27]) / 10000);

                m_data.hAcc = (float)((uint32_t)m_msg[i].data[28]
                            | ((uint32_t)m_msg[i].data[29] << 8)
                            | ((uint32_t)m_msg[i].data[30] << 16)
                            | ((uint32_t)m_msg[i].data[31] << 24)) / 10000.0f; //hAcc

                m_data.vAcc = (float)((uint32_t)m_msg[i].data[32]
                            | ((uint32_t)m_msg[i].data[33] << 8)
                            | ((uint32_t)m_msg[i].data[34] << 16)
                            | ((uint32_t)m_msg[i].data[35] << 24)) / 10000.0f; //vAcc

            break;
            }
            case(0x3c): //UBX-NAV-RELPOSNED
            {
            
                m_data.refstationid =    (uint16_t)m_msg[i].data[2]
                                      | ((uint16_t)m_msg[i].data[3] << 8); //ref ID

                m_data.itow =    (uint32_t)m_msg[i].data[4]
                              | ((uint32_t)m_msg[i].data[5] << 8)
                              | ((uint32_t)m_msg[i].data[6] << 16)
                              | ((uint32_t)m_msg[i].data[7] << 24); //itow

                float tempf[3];
                tempf[0] =    (float)((uint32_t)m_msg[i].data[8]
                            | ((uint32_t)m_msg[i].data[9] << 8)
                            | ((uint32_t)m_msg[i].data[10] << 16)
                            | ((uint32_t)m_msg[i].data[11] << 24)) / 100.0f; //relpos N

                tempf[1] =    (float)((uint32_t)m_msg[i].data[12]
                            | ((uint32_t)m_msg[i].data[13] << 8)
                            | ((uint32_t)m_msg[i].data[14] << 16)
                            | ((uint32_t)m_msg[i].data[15] << 24)) / 100.0f; //relpos E

                tempf[2] =    (float)((uint32_t)m_msg[i].data[16]
                            | ((uint32_t)m_msg[i].data[17] << 8)
                            | ((uint32_t)m_msg[i].data[18] << 16)
                            | ((uint32_t)m_msg[i].data[19] << 24)) / 100.0f; //relpos D

                m_data.relPosNED << (tempf[0] + (float)(int8_t)m_msg[i].data[32] / 10000.0f),
                                    (tempf[1] + (float)(int8_t)m_msg[i].data[33] / 10000.0f),
                                    (tempf[2] + (float)(int8_t)m_msg[i].data[34] / 10000.0f);

                tempf[0] =    (float)((uint32_t)m_msg[i].data[36]
                            | ((uint32_t)m_msg[i].data[37] << 8)
                            | ((uint32_t)m_msg[i].data[38] << 16)
                            | ((uint32_t)m_msg[i].data[39] << 24)) / 10000.0f; //acc N

                tempf[1] =    (float)((uint32_t)m_msg[i].data[40]
                            | ((uint32_t)m_msg[i].data[41] << 8)
                            | ((uint32_t)m_msg[i].data[42] << 16)
                            | ((uint32_t)m_msg[i].data[43] << 24)) / 10000.0f; //acc E

                tempf[2] =    (float)((uint32_t)m_msg[i].data[44]
                            | ((uint32_t)m_msg[i].data[45] << 8)
                            | ((uint32_t)m_msg[i].data[46] << 16)
                            | ((uint32_t)m_msg[i].data[47] << 24)) / 10000.0f; //acc D

                m_data.accNED << tempf[0], tempf[1], tempf[2];

                m_data.rtk_flags =  (uint32_t)m_msg[i].data[60]
                                 | ((uint32_t)m_msg[i].data[61] << 8)
                                 | ((uint32_t)m_msg[i].data[62] << 16)
                                 | ((uint32_t)m_msg[i].data[63] << 24);

            break;
            }

            case(0x07): //UBX-NAV-PVT
            {
                m_data.itow =    (uint32_t)m_msg[i].data[0]
                              | ((uint32_t)m_msg[i].data[1] << 8)
                              | ((uint32_t)m_msg[i].data[2] << 16)
                              | ((uint32_t)m_msg[i].data[3] << 24); //itow

                m_data.fix_type = m_msg[i].data[20];
                m_data.numSV = m_msg[i].data[23];

                float tempf[3];
                tempf[0] =    (float)((uint32_t)m_msg[i].data[24]
                            | ((uint32_t)m_msg[i].data[25] << 8)
                            | ((uint32_t)m_msg[i].data[26] << 16)
                            | ((uint32_t)m_msg[i].data[27] << 24)) / 10000000.0f; //lon

                tempf[1] =    (float)((uint32_t)m_msg[i].data[28]
                            | ((uint32_t)m_msg[i].data[29] << 8)
                            | ((uint32_t)m_msg[i].data[30] << 16)
                            | ((uint32_t)m_msg[i].data[31] << 24)) / 10000000.0f; // lat

                tempf[2] =    (float)((uint32_t)m_msg[i].data[32]
                            | ((uint32_t)m_msg[i].data[33] << 8)
                            | ((uint32_t)m_msg[i].data[34] << 16)
                            | ((uint32_t)m_msg[i].data[35] << 24)) / 1000.0f; // height

                m_data.llh << tempf[0], tempf[1], tempf[2];

                tempf[0] =    (float)((uint32_t)m_msg[i].data[48]
                            | ((uint32_t)m_msg[i].data[49] << 8)
                            | ((uint32_t)m_msg[i].data[50] << 16)
                            | ((uint32_t)m_msg[i].data[51] << 24)) / 1000.0f; //vel N

                tempf[1] =    (float)((uint32_t)m_msg[i].data[52]
                            | ((uint32_t)m_msg[i].data[53] << 8)
                            | ((uint32_t)m_msg[i].data[54] << 16)
                            | ((uint32_t)m_msg[i].data[55] << 24)) / 1000.0f; // vel E

                tempf[2] =    (float)((uint32_t)m_msg[i].data[56]
                            | ((uint32_t)m_msg[i].data[57] << 8)
                            | ((uint32_t)m_msg[i].data[58] << 16)
                            | ((uint32_t)m_msg[i].data[59] << 24)) / 1000.0f; // vel D

                m_data.velNED << tempf[0], tempf[1], tempf[2];

                m_data.gSpeed =    (float)((uint32_t)m_msg[i].data[60]
                                 | ((uint32_t)m_msg[i].data[61] << 8)
                                 | ((uint32_t)m_msg[i].data[62] << 16)
                                 | ((uint32_t)m_msg[i].data[63] << 24)) / 1000.0f; // ground Speed

                m_data.headMotion = (float)((uint32_t)m_msg[i].data[64]
                                 | ((uint32_t)m_msg[i].data[65] << 8)
                                 | ((uint32_t)m_msg[i].data[66] << 16)
                                 | ((uint32_t)m_msg[i].data[67] << 24)) / 100000.0f; // heading
                
                m_data.pDOP = (float)((uint32_t)m_msg[i].data[76]
                                 | ((uint32_t)m_msg[i].data[77] << 8)) / 100.0f; // Position DOP


                m_data.lastCorrectionAge = (m_msg[i].data[78] >> 1) & 0x0F;

            break;
            }
            case(0x36): //UBX-NAV-COV
            {
                m_data.itow =      (uint32_t)m_msg[i].data[0]
                                | ((uint32_t)m_msg[i].data[1] << 8)
                                | ((uint32_t)m_msg[i].data[2] << 16)
                                | ((uint32_t)m_msg[i].data[3] << 24);

                m_data.posCOVvalid = m_msg[i].data[5] & 0x01;
                m_data.velCOVvalid = m_msg[i].data[6] & 0x01;

                m_data.posCovNN =  (float)((uint32_t)m_msg[i].data[16]
                                        | ((uint32_t)m_msg[i].data[17] << 8)
                                        | ((uint32_t)m_msg[i].data[18] << 16)
                                        | ((uint32_t)m_msg[i].data[19] << 24));
                
                m_data.posCovNE =  (float)((uint32_t)m_msg[i].data[20]
                                        | ((uint32_t)m_msg[i].data[21] << 8)
                                        | ((uint32_t)m_msg[i].data[22] << 16)
                                        | ((uint32_t)m_msg[i].data[23] << 24));

                m_data.posCovND =  (float)((uint32_t)m_msg[i].data[24]
                                        | ((uint32_t)m_msg[i].data[25] << 8)
                                        | ((uint32_t)m_msg[i].data[26] << 16)
                                        | ((uint32_t)m_msg[i].data[27] << 24));
                
                m_data.posCovEE =  (float)((uint32_t)m_msg[i].data[28]
                                        | ((uint32_t)m_msg[i].data[29] << 8)
                                        | ((uint32_t)m_msg[i].data[30] << 16)
                                        | ((uint32_t)m_msg[i].data[31] << 24));
                
                m_data.posCovED =  (float)((uint32_t)m_msg[i].data[32]
                                        | ((uint32_t)m_msg[i].data[33] << 8)
                                        | ((uint32_t)m_msg[i].data[34] << 16)
                                        | ((uint32_t)m_msg[i].data[35] << 24));

                m_data.posCovDD =  (float)((uint32_t)m_msg[i].data[36]
                                        | ((uint32_t)m_msg[i].data[37] << 8)
                                        | ((uint32_t)m_msg[i].data[38] << 16)
                                        | ((uint32_t)m_msg[i].data[39] << 24));
                                        
                m_data.velCovNN =  (float)((uint32_t)m_msg[i].data[40]
                                        | ((uint32_t)m_msg[i].data[41] << 8)
                                        | ((uint32_t)m_msg[i].data[42] << 16)
                                        | ((uint32_t)m_msg[i].data[43] << 24));
                
                m_data.velCovNE =  (float)((uint32_t)m_msg[i].data[44]
                                        | ((uint32_t)m_msg[i].data[45] << 8)
                                        | ((uint32_t)m_msg[i].data[46] << 16)
                                        | ((uint32_t)m_msg[i].data[47] << 24));

                m_data.velCovND =  (float)((uint32_t)m_msg[i].data[48]
                                        | ((uint32_t)m_msg[i].data[49] << 8)
                                        | ((uint32_t)m_msg[i].data[50] << 16)
                                        | ((uint32_t)m_msg[i].data[51] << 24));
                
                m_data.velCovEE =  (float)((uint32_t)m_msg[i].data[52]
                                        | ((uint32_t)m_msg[i].data[53] << 8)
                                        | ((uint32_t)m_msg[i].data[54] << 16)
                                        | ((uint32_t)m_msg[i].data[55] << 24));
                
                m_data.velCovED =  (float)((uint32_t)m_msg[i].data[56]
                                        | ((uint32_t)m_msg[i].data[57] << 8)
                                        | ((uint32_t)m_msg[i].data[58] << 16)
                                        | ((uint32_t)m_msg[i].data[59] << 24));

                m_data.velCovDD =  (float)((uint32_t)m_msg[i].data[60]
                                        | ((uint32_t)m_msg[i].data[61] << 8)
                                        | ((uint32_t)m_msg[i].data[62] << 16)
                                        | ((uint32_t)m_msg[i].data[63] << 24));

            break;
            }
            case(0x04): //UBX-NAV-DOP
            {
                m_data.itow =      (uint32_t)m_msg[i].data[0]
                                | ((uint32_t)m_msg[i].data[1] << 8)
                                | ((uint32_t)m_msg[i].data[2] << 16)
                                | ((uint32_t)m_msg[i].data[3] << 24);

                m_data.gDOP =  (float)((uint32_t)m_msg[i].data[4]
                                    | ((uint32_t)m_msg[i].data[5] << 8)) / 100.0f; // Geometric DOP
                
                m_data.pDOP =  (float)((uint32_t)m_msg[i].data[6]
                                    | ((uint32_t)m_msg[i].data[7] << 8)) / 100.0f; // Position DOP
                
                m_data.tDOP =  (float)((uint32_t)m_msg[i].data[8]
                                    | ((uint32_t)m_msg[i].data[9] << 8)) / 100.0f; // Time DOP
                
                m_data.vDOP =  (float)((uint32_t)m_msg[i].data[10]
                                    | ((uint32_t)m_msg[i].data[11] << 8)) / 100.0f; // vertical DOP

                m_data.hDOP =  (float)((uint32_t)m_msg[i].data[12]
                                    | ((uint32_t)m_msg[i].data[13] << 8)) / 100.0f; // horizontal DOP
                
                m_data.nDOP =  (float)((uint32_t)m_msg[i].data[14]
                                    | ((uint32_t)m_msg[i].data[15] << 8)) / 100.0f; // Northing DOP
                
                m_data.eDOP =  (float)((uint32_t)m_msg[i].data[16]
                                    | ((uint32_t)m_msg[i].data[17] << 8)) / 100.0f; // Easting DOP

            break;
            }   
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

#if GNSS_DO_PRINTF
    printf("l = %i\n", msg_length);
#endif

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
            remaining_bytes -= (8 - m_msg[m_msg_index].length);
            
            //printf("\n\nmsg_index = %i, id = 0x%x, l = %i\n",m_msg_index, m_msg[m_msg_index].id, m_msg[m_msg_index].length);
            //checksum(m_msg_index); //doesnt currently work so its expected to be correct
            m_msg[m_msg_index].is_valid = true;

            m_msg_index++;
            if(m_msg_index >= 10){ //reached max messages
                m_msg_index = 0; // if more messages come the will be overwritten
            }

            

            if(remaining_bytes < 0){ // some bytes are missing

#if GNSS_DO_PRINTF
                //printf("Something wrong with: %i, remaining bytes = %i\n", m_msg_index, remaining_bytes);
#endif

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
#if GNSS_DO_PRINTF
        //printf("number of msg = %u \n",m_msg_index);

#endif
    
    return decode();
}


bool GNSS::init()
{
    m_uart.set_baud(GNSS_UART_BAUD); //in the future as a parameter
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
