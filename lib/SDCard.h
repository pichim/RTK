#ifndef SDCARD_H_
#define SDCARD_H_

#include <mbed.h>
#include <SDBlockDevice.h>
#include <FATFileSystem.h>

#define MOUNT_PATH "sd"

class SDCard{
    public:
    SDCard();

    bool init();
    bool write2sd(char* data);
    bool writeln();
    bool close();

    private:
    bool init_success;
    char path[30];
    FATFileSystem fs;
    SDBlockDevice _sd;
    FILE* fp;
};

#endif /* SDCARD_H_ */