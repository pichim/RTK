#include "SDCard.h"

SDCard::SDCard() : m_fs("sd"), m_sd(PC_12, PC_11, PC_10, PD_2){
    
    m_init_success = 0;//nothing special here
}


bool SDCard::init() {
    if (0 != m_sd.init()) {
        //printf("Init failed \n");
        return 0;
    }
    if (0 != m_sd.frequency(12000000)) {
        //printf("Error setting frequency \n");
    }

    fflush(stdout);

    if(m_fs.mount(&m_sd) != 0){
        //printf("mount failed\n");
        return 0;
    }
    fflush(stdout);
    mkdir("/sd/data", 0777);
    
    int i = 0;
    
    while(1){
        i++;
        sprintf(m_path, "/sd/data/%03i.bin", i);
        m_fp = fopen(m_path, "r");
        if(m_fp == NULL){
            m_fp = fopen(m_path, "w");
            fprintf(m_fp,"sep=;\nRTK_GPS_DATA;%s\n",m_path);

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
    m_init_success = true;
    return 1;
}


bool SDCard::write_str_2_sd(char *data){
    if(!m_init_success) return 0;

    //FILE* fp = fopen(path, "a");
    if(m_fp == NULL){
        return 0;
    }
    fprintf(m_fp, "%s", data);
    //fclose(fp);

    return 1;
}


bool SDCard::write_ln(){
    if(!m_init_success) return 0;

    char tmp[] = "\n";
    if(!write_str_2_sd(tmp)){
        return 0;
    }
    return 1;
}


bool SDCard::write_f_2_sd(float* data, int l){
    if(!m_init_success) return 0;

    size_t n_written = fwrite(data, sizeof(float), l, m_fp);

    if (n_written != l){
        printf("SDCard Write failed\n");
    }

    return 1;
}


bool SDCard::write_u32_2_sd(uint32_t* data, int l) {
    if(!m_init_success) return 0;

    size_t n_written = fwrite(data, sizeof(uint32_t), l, m_fp);

    if (n_written != l){
        printf("SDCard Write failed\n");
    }

    return 1;
}

bool SDCard::write_u8_2_sd(uint8_t* data, int l){
    if(!m_init_success) return 0;

    size_t n_written = fwrite(data, sizeof(uint8_t), l, m_fp);

    if (n_written != l){
        printf("SDCard Write failed\n");
    }

    return 1;
}

//TODO: compress individual bits into one byte
bool SDCard::write_bool_2_sd(bool* data, int l){
    if(!m_init_success) return 0;

    size_t n_written = fwrite(data, sizeof(bool), l, m_fp);

    if (n_written != l){
        printf("SDCard Write failed\n");
    }

    return 1;
}


bool SDCard::close(){
    if(!m_init_success) return 0; //must be here else expect to have crashes

    fclose(m_fp);
    if(0 != m_sd.deinit()){
        return 0;
    }
    m_init_success = 0;
    return 1;
}