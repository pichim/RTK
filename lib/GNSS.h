#ifndef GNSS_H_
#define GNSS_H_


#include <mbed.h>



class GNSS {

    public:
        GNSS(PinName, PinName);

        uint32_t read();


    private:
        bool init();


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
