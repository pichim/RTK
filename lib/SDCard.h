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
    bool mkfile();
    void sdMutexLock();
    void sdMutexUnlock();
    bool write_str_2_sd(char* data);
    bool write_ln();
    bool write_f_2_sd(float* data, size_t l);
    bool write_u32_2_sd(uint32_t* data, size_t l);
    bool write_u8_2_sd(uint8_t* data, size_t l);
    bool write_bool_2_sd(bool* data, size_t l);
    bool close();

    private:
    bool write_2_sd(void* data, size_t s, size_t l);
    bool m_init_success;
    bool m_file_valid;
    char m_path[30];
    FATFileSystem m_fs;
    SDBlockDevice m_sd;
    //Mutex m_sdMutex;
    FILE* m_fp;
};

#endif /* SDCARD_H_ */