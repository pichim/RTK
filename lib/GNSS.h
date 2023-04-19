#ifndef GNSS_H_
#define GNSS_H_


#include <mbed.h>
#include "Param.h"
/*
enum msg_state_t{
    MSG_IDLE,
    MSG_HEADER,
    MSG_CLASS,
    MSG_ID,
    MSG_LENGTH,
    MSG_DATA,
    MSG_CK_A,
    MSG_CK_B
};
*/


class UBXDATA
{
    public:
        UBXDATA(){};

        uint16_t header;
        uint8_t class_;
        uint8_t id;
        uint16_t length;
        char data[256];
        uint8_t ck_a;
        uint8_t ck_b;
        bool is_arriving;
        bool is_valid;

        void reset(){
            header = 0;
            class_ = 0;
            id = 0;
            length = 0;
            ck_a = 0;
            ck_b = 0;
            is_arriving = 0;
            is_valid = 0;

            for(int i = 0; i < 256; i++){
                data[i] = 0;
            }

        }
};

class GNSS {

    public:
        GNSS(PinName, PinName, Data& data);

        uint8_t decode();
        uint8_t readGNSSdata();


    private:

        uint8_t m_msg_index;
        UBXDATA m_msg[10];
        Data& m_data;
        BufferedSerial m_uart;

        bool init();
        bool checksum(int i);
        time_t date2sec(uint16_t yyyy_, uint8_t mm_, uint8_t dd_, uint8_t hh_, uint8_t min_, uint8_t ss_);

        //void read_uart();
        



};



#endif /* GNSS_H_ */
