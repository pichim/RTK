#include "GNSS.h"

#define BUFFER_SIZE 128




GNSS::GNSS(PinName rx, PinName tx) 
    :m_uart(rx, tx)
{
    init();

}

uint32_t GNSS::decode(char* buf, int l){

    // loop through all objects
        //check if object is valid
        //decode this object

    // reset m_msg_index to 0

    //return the number of valid objects

    return 0;
}



uint8_t GNSS::readGNSSdata(){

    static char buffer[512];
    static int msg_length = 0;

    msg_length = m_uart.read(buffer, sizeof(buffer));
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
                m_msg[m_msg_index].data[i] = buffer[offset + 5 + i];
            }

            m_msg[m_msg_index].ck_a = buffer[offset + 5 + i + 1];
            m_msg[m_msg_index].ck_b = buffer[offset + 5 + i + 2];

            /*
            if(m_msg[m_msg_index].id == 0x3b){
                m_msg[m_msg_index].checksum();
            }
            */
            

            m_msg[m_msg_index].is_arriving = false;

            offset = offset + 5 + i + 3;
            remaining_bytes -= offset;
            
            //printf("\n\nmsg_index = %i, id = 0x%x, l = %i\n",m_msg_index, m_msg[m_msg_index].id, m_msg[m_msg_index].length);

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
    
    return decode(buffer, msg_length);
}


bool GNSS::init(){
    m_uart.set_baud(38400); //in the future as a parameter
    m_uart.set_blocking(false);
    m_uart.set_format(8,BufferedSerial::None,1);

    //m_uart.attach(callback(this,&GNSS::read_uart),SerialBase::RxIrq);

    m_msg_index = 0;
    return true;
}

/*
void GNSS::read_uart(){
    char c;
    static char state = MSG_IDLE;
    static uint16_t header = 0;
    static int ii = 0;

    m_uart.read(&c,1);

    switch(state){
        case(MSG_HEADER):

            header = header << 8;
            header |= c;

            if(header == 0xb562){
                header = 0;
                m_msg[m_msg_index].header=header;
                m_msg[m_msg_index].is_arriving=true;
                state = MSG_CLASS;
            }
            return;

        case(MSG_CLASS):

            m_msg[m_msg_index].class_ = c;
            state = MSG_ID;
            return;

        case(MSG_ID):

            m_msg[m_msg_index].id = c;
            state = MSG_LENGTH;
            return;

        case(MSG_LENGTH):

            m_msg[m_msg_index].length = m_msg[m_msg_index].length << 8;
            m_msg[m_msg_index].length |= c;
            if(ii){
                ii = 0;
                state = MSG_DATA;
                return;
            }
            ii++;
            return;

        case(MSG_DATA):

            m_msg[m_msg_index].data[ii] = c;
            ii++;

            if(ii == m_msg[m_msg_index].length){
                ii = 0;
                state = MSG_CK_A;
                return;
            }
            return;

        case(MSG_CK_A):

            m_msg[m_msg_index].ck_a = c;
            state = MSG_CK_B;
            return;

        case(MSG_CK_B):

            m_msg[m_msg_index].ck_a = c;
            state = MSG_HEADER;
            return;

        default:

        return;
    }



}
*/