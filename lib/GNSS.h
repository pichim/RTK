#ifndef GNSS_H_
#define GNSS_H_


#include <mbed.h>
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


class UBXDATA{
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

        void checksum(){
            uint8_t a = 0;
            uint8_t b = 0;
            a += class_;
            b += a;

            a += id;
            b += a;

            a += (uint8_t)(length >> 8);
            b += a;

            a += (uint8_t)length;
            b += a;

            for(int i = 0; i < length; i++){
                a += data[i];
                b += a;
            }
            /*
            printf("my ckecksum = 0x%02x , 0x%02x\n",a,b);
            printf("desired cs  = 0x%02x , 0x%02x\n",ck_a,ck_b);

            */
            if((ck_a == a) && (ck_b == b)){
                is_valid = true;
            }

        }

};

class GNSS {

    public:
        GNSS(PinName, PinName);

        uint32_t decode(char* buf, int l);
        uint8_t readGNSSdata();


    private:

        uint8_t m_msg_index;
        UBXDATA m_msg[10];



        bool init();

        //void read_uart();


        typedef struct ubxNavPVT_s {
            uint32_t iTOW;    //  0  -   ms   GPS time of week of the navigation epoch
            uint16_t year;    //  4  -    -   Year (UTC)
            uint8_t month;    //  6  -    -   Month, range 1..12 (UTC)
            uint8_t day;      //  7  -    -   Day of month, range 1..31 (UTC)
            uint8_t hour;     //  8  -    -   Hour of day, range 0..23 (UTC)
            uint8_t min;      //  9  -    -   Minute of hour, range 0..59 (UTC)
            uint8_t sec;      // 10  -    -   Seconds of minute, range 0..60 (UTC)
            uint8_t fixType;  // 20  -    -   GNSSﬁx Type
            uint8_t numSV;    // 23  -    -   Number of satellites used in Nav Solution
            int32_t lon;      // 24 1e-7 deg  Longitude
            int32_t lat;      // 28 1e-7 deg  Latitude
            int32_t height;   // 32  -   mm   Height above ellipsoid
            uint32_t hAcc;    // 40  -   mm   Horizontal accuracy estimate
            uint32_t vAcc;    // 44  -   mm   Vertical accuracy estimate
            int32_t velN;     // 48  -   mm/s NED north velocity
            int32_t velE;     // 52  -   mm/s NED east velocity
            int32_t velD;     // 56  -   mm/s NED down velocity
            int32_t gSpeed;   // 60  -   mm/s Ground Speed (2-D)
            int32_t headMot;  // 64 1e-5 deg  Heading of motion (2-D)
            uint32_t sAcc;    // 68 -    mm/s Speed accuracy estimate
            uint32_t headAcc; // 72 1e-5 deg  Heading accuracy estimate (both motion and vehicle)
            //int16_t magDec;   // 88 1e-2 deg  Magnetic declination.
            //uint16_t magAcc;  // 90 1e-2 deg  Magnetic declination accuracy
        } ubxNavPVT_t;

        BufferedSerial m_uart;



};



#endif /* GNSS_H_ */
