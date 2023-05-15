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
    bool write_str_2_sd(char* data);
    bool write_ln();
    bool write_f_2_sd(float* data, int l);
    bool write_u32_2_sd(uint32_t* data, int l);
    bool write_u8_2_sd(uint8_t* data, int l);
    bool write_bool_2_sd(bool* data, int l);
    bool close();

    private:
    bool m_init_success;
    char m_path[30];
    FATFileSystem m_fs;
    SDBlockDevice m_sd;
    FILE* m_fp;

    PlatformMutex _mutex;
    
    bool write_sd(void* data, int s, int l);
};

#endif /* SDCARD_H_ */