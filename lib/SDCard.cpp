#include "SDCard.h"

SDCard::SDCard() : m_fs("sd"), m_sd(PC_12, PC_11, PC_10, PD_2){
    
    m_init_success = 0;//nothing special here
    m_file_valid = 0;
}


bool SDCard::init() {
    if (0 != m_sd.init()) {
        printf("Init failed \n");
        return 0;
    }
    
    if (0 != m_sd.frequency(5000000)) {
        printf("Error setting frequency \n");
    }
    
    fflush(stdout);

    if(m_fs.mount(&m_sd) != 0){
        printf("mount failed\n");
        return 0;
    }
    
    fflush(stdout);
    mkdir("/sd/data", 0777);
    m_init_success = true;

    return 1;
}


bool SDCard::mkfile() {
    if(m_file_valid) return 0;
    if(!m_init_success) return 0;

    int i = 0;
    
    while(1){
        i++;
        sprintf(m_path, "/sd/data/%03i.bin", i);
        m_fp = fopen(m_path, "r");
        if(m_fp == NULL){
            m_fp = fopen(m_path, "w");
            //header for .csv files
            //fprintf(m_fp,"sep=;\nRTK_GPS_DATA;%s\n",m_path);

            printf("working in file: %s\n",m_path);
            break;
        } else {
            fclose(m_fp);
        }
        if(i >= 999){
            printf("maximum files reached\n");
            fclose(m_fp);
            return 0;
        }
    }
    //fclose(fp);
    m_file_valid = true;

    return 1;
}

void SDCard::sdMutexLock(){
    //m_sdMutex.lock();
}

void SDCard::sdMutexUnlock(){
    //m_sdMutex.unlock();
}

bool SDCard::write_str_2_sd(char *data) {
    if(!m_file_valid) return 0;

    //FILE* fp = fopen(path, "a");
    if(m_fp == NULL){
        return 0;
    }
    fprintf(m_fp, "%s", data);
    //fclose(fp);

    return 1;
}


bool SDCard::write_ln() {
    char tmp[] = "\n";
    if(!write_str_2_sd(tmp)){
        return 0;
    }
    return 1;
}


bool SDCard::write_f_2_sd(float* data, size_t l) {
    
    return write_2_sd(data, sizeof(float), l);
}

bool SDCard::write_d_2_sd(double* data, size_t l) {
    
    return write_2_sd(data, sizeof(double), l);
}


bool SDCard::write_u32_2_sd(uint32_t* data, size_t l) {
    
    return write_2_sd(data, sizeof(uint32_t), l);
}

bool SDCard::write_u8_2_sd(uint8_t* data, size_t l) {

    return write_2_sd(data, sizeof(uint8_t), l);
}

//TODO: compress 8 bits into one byte
bool SDCard::write_bool_2_sd(bool* data, size_t l)
{
    uint8_t buf_data[l];
    memcpy(buf_data, data, l);
    int n = l / 8;
    int r = l % 8;
    if (r > 0){
        n++;
    }
    int j = 0;
    uint8_t buf[n];

    while(n){
        for(int i = 0; i < 8; i++){
            buf[j] |= (buf_data[j*8+i] << i);
        }
        n--;
        j++;
    }
    printf("%02x\n",buf[1]);
    //return write_2_sd(data, sizeof(bool), l);
    return write_2_sd(buf, sizeof(bool), j);
}

bool SDCard::close() {

    //m_sdMutex.lock();

    if(m_file_valid){
        fclose(m_fp);
        m_file_valid = 0;
        printf("closing file\n");
    } else{
        return 0;
    }
    //m_sdMutex.unlock();

    return 1;
}

bool SDCard::unmount()
{
    if(m_init_success){
        m_sd.deinit(); //must be here else expect to have crashes
        m_init_success = 0;
        printf("unmounting SD card\n");
    } else{
        return 0;
    }
    return 1;
}


bool SDCard::write_2_sd(void* data, size_t s, size_t l) {
    if(!m_file_valid) return 0;

    size_t n_written = fwrite(data, s, l, m_fp);

    if (n_written != l){
        printf("SDCard Write failed\n");
        return 0;
    }

    return 1;
}