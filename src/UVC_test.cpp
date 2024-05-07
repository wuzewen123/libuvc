#include <./libuvc/libuvc.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <fstream>
#include <./md5.h>
#include <csignal>
#include <cstdlib>
#include <iostream>
#define WIDTH 240
#define HEIGHT 360
std::vector<uint16_t> amplitude_data;
std::vector<uint16_t> phase_data;
std::vector<uint8_t> data;
typedef std::ifstream InputFileStream;
uvc_device_handle_t *_msg_devh;
int curFreq = 0;
bool stop_requested = false;
#define MAX_FRAME_WITDH 240
#define MAX_FRAME_HEIGHT 180
#define UVC_PT_CAMERA_CTRL 0x01
#define UVC_PT_CALIBRATE_CTRL 0x02
#define UVC_PT_DEPTHEYE_CTRL 0x03
#define UVC_VC_DEPTHEYE_UNIT_ID 13  // Pointcloud.ai ctrl
#define UVC_VC_POINTCLOUD_UNIT_ID 8 // Pointcloud.ai
#define UVC_VC_IMX556REG_UNIT_ID 9  // Pointcloud.ai
#define SYSTEM_FREQ_CLOCK 120
#define UVC_CTRL_BUFF_LENGTH 32 // 60
#define DEPTHEYE_BASE_ID 0x100
#define DEPTHEYE_OUTPUT_MODE_ID DEPTHEYE_BASE_ID + 1
#define DEPTHEYE_MEASURE_MODE_ID DEPTHEYE_BASE_ID + 2
#define DEPTHEYE_FRAME_RATE_ID DEPTHEYE_BASE_ID + 3
#define DEPTHEYE_INTERGRAL_TIME_SCALE DEPTHEYE_BASE_ID + 12
#define DEPTHEYE_INTERGRAL_TIME DEPTHEYE_BASE_ID + 13
#define DEPTHEYE_FIRMWARE_VERSION DEPTHEYE_BASE_ID + 17
#define DEPTHEYE_GET_TSENSOR DEPTHEYE_BASE_ID + 22
#define DEPTHEYE_GET_TILLUM DEPTHEYE_BASE_ID + 23
#define DEPTHEYE_SET_PWM DEPTHEYE_BASE_ID + 24
#define DEPTHEYE_SET_AE_ENABLE DEPTHEYE_BASE_ID + 25
#define DEPTHEYE_GET_OVEREXPOSED_PIXELS DEPTHEYE_BASE_ID + 26
#define DEPTHEYE_SET_EXPLORE_STATU DEPTHEYE_BASE_ID + 27

#define TSENSOR_OFFSET 1
#define TILLUM_OFFSET 2
#define PHASE_LENGTH 240 * 180
// #define pointcloud

void *user_ptr = 0;
uvc_device_handle_t *devh;

void stop(uvc_device_handle_t *devh)
{
    uvc_stop_streaming(devh);
}

#ifdef pointcloud

bool setDeptheyeCMD(uvc_device_handle_t *devh, uint16_t address, uint8_t value, uint8_t length)
{
    uint8_t ctl_cmd = UVC_PT_DEPTHEYE_CTRL;
    uint8_t unit = UVC_VC_POINTCLOUD_UNIT_ID;

    int resSetMode = 0;

    uint8_t data[6];
    memcpy(data, &address, 2);
    memcpy(data + 2, &value, 1);
    resSetMode = uvc_set_ctrl(devh, unit, ctl_cmd, data, length + 2);
    if (resSetMode != length + 2)
        return false;
    return true;
}

bool getDeptheyeCMD(uvc_device_handle_t *devh, uint8_t address, uint8_t *value, uint8_t length)
{
    uint8_t unit = UVC_VC_DEPTHEYE_UNIT_ID;
    int resSetMode = uvc_get_ctrl(devh, unit, address, value, length, UVC_GET_CUR);
    if (resSetMode != length)
        return false;

    return true;
}

bool getVersion(uvc_device_handle_t *devh)
{
    uint8_t data[4] = {0};
    uint8_t len = 4;
    uint8_t ctl_cmd = UVC_PT_DEPTHEYE_CTRL;
    uint8_t unit = UVC_VC_DEPTHEYE_UNIT_ID;
    getDeptheyeCMD(devh, DEPTHEYE_FIRMWARE_VERSION, data, 4);
    // if(resSetMode != len)
    //     return false;
    uint8_t major = data[2];
    uint8_t minor = data[1];
    uint8_t patch = data[0];
    uint32_t ret = 0;
    ret = major;
    ret <<= 8;
    ret |= minor;
    ret <<= 8;
    ret |= patch;
    printf("INFO: Sigmastar Firmware Version: v%d.%d.%d(0x%08x)\n", major, minor, patch, ret);
    return true;
}

// bool uvcUploadFile(uvc_device_handle_t *_msg_devh, const std::string &readFile, const std::string &saveName, bool savefile)
// {
//     uint8_t ctl_cmd = UVC_PT_CAMERA_CTRL;
//     uint8_t unit = UVC_VC_IMX556REG_UNIT_ID; // UVC_VC_POINTCLOUD_UNIT_ID;
//     uint8_t *ptr = 0;
//     int length = 0;
//     uint8_t temp_file[256] = {0};
//     uint file_length = (uint)saveName.length();
//     uint compensation = 0;
//     uint upload_cnt = 0;
//     uint last_upload_cnt = 0;
//     uint md5_length = 16;

//     // get md5
//     uint8_t *md5_data = new uint8_t[16];
//     ifstream fin;
//     fin.open(readFile.c_str(), std::ios::binary);
//     if (!fin.good())
//     {
//         std::cout << "Error: file \"" << readFile << "\" not existed!" << std::endl;
//         delete[] md5_data;
//         return false;
//     }
//     MD5 md5;
//     md5.reset();
//     md5.update(fin);
//     md5.toString();
//     md5.get_md5(md5_data);
//     for (int i = 0; i < 16; i++)
//     {
//         printf("%x", md5_data[i]);
//     }
//     std::cout << std::endl;
//     fin.close();

//     if (!savefile)
//     {
//         for (int i = 0; i < 256; i++)
//         {
//             temp_file[i] = i;
//         }
//         memcpy(temp_file, &file_length, 4);
//         memcpy(temp_file + 4, saveName.c_str(), file_length);
//         ptr = temp_file;
//         length = 256;
//     }
//     else
//     {
//         InputFileStream f(readFile.c_str(), std::ios::binary | std::ios::ate);

//         if (!f.good())
//         {
//             std::cout << "Error: file not existed!" << readFile << std::endl;
//             return false;
//         }
//         size_t size = f.tellg();

//         compensation = UVC_CTRL_BUFF_LENGTH - (size + 4 + md5_length + file_length) % 32;
//         ptr = new uint8_t[size + 4 + file_length + md5_length + compensation];
//         memset(ptr, '\0', size + 4 + file_length + md5_length + compensation);
//         memcpy(ptr, &file_length, 4);
//         memcpy(ptr + 4, saveName.c_str(), file_length);
//         memcpy(ptr + 4 + file_length, md5_data, md5_length);
//         f.seekg(0, std::ios::beg);
//         f.clear();
//         if (!f.good())
//             return false;
//         f.read((char *)ptr + 4 + file_length + md5_length, size);
//         length = size + 4 + file_length + md5_length;

//         /*compensation = UVC_CTRL_BUFF_LENGTH - (size + 4 + file_length) % 32;
//         ptr = new uint8_t[size + 4 + file_length + compensation];
//         memset(ptr, '\0', size + 4 + file_length + compensation);
//         memcpy(ptr, &file_length, 4);
//         memcpy(ptr + 4, saveName.c_str(), file_length);
//         f.seekg(0, std::ios::beg);
//         f.clear();
//         if (!f.good())
//             return -1;
//         f.read((char *)ptr + 4 + file_length, size);
//         length = size + 4 + file_length;*/
//     }

//     int ret = uvc_set_xu_fw(_msg_devh, unit, ctl_cmd, ptr, length);
//     if (ret < 0)
//     {
//         return ret;
//     }

//     if (savefile)
//     {
//         delete[] ptr;
//     }

//     return true;
// }

int getIntegralTime(uvc_device_handle_t *devh)
{
    uint value = 0;
    uint integrationDutyCycle = 0;

    uint8_t data[4] = {0, 0, 0, 0};
    uint8_t len = 4;

    getDeptheyeCMD(devh, DEPTHEYE_INTERGRAL_TIME, data, len);
    integrationDutyCycle = (uint) * (uint *)data;
    printf("integrationDutyCycle :%d\n", integrationDutyCycle);

    uint intg_time = (integrationDutyCycle / SYSTEM_FREQ_CLOCK / 10);
    // if (intg_time > 100) intg_time = 100;
    // if (intg_time < 0) intg_time = 0;
    value = intg_time;
    // logger(LOG_DEBUG) << "get intgTime[0,100]: " << value << std::endl;
    printf("get intgTime[0,100]: %d\n", value);

    return value;
}

int getOverexposedPixels(uvc_device_handle_t *devh)
{
    uint value = 0;

    uint8_t data[4] = {0, 0, 0, 0};
    uint8_t len = 4;

    getDeptheyeCMD(devh, DEPTHEYE_GET_OVEREXPOSED_PIXELS, data, len);
    value = (uint) * (uint *)data;

    return value;
}

#endif

void callback(uvc_frame_t *frame, void *user_ptr)
{
    // uvc_frame_t *bgr;
    // uvc_error_t ret;
    // enum uvc_frame_format *frame_format = (enum uvc_frame_format *)user_ptr;
    // /* FILE *fp;
    //  * static int jpeg_count = 0;
    //  * static const char *H264_FILE = "iOSDevLog.h264";
    //  * static const char *MJPEG_FILE = ".jpeg";
    //  * char filename[16]; */

    // /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
    // bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    // if (!bgr)
    // {
    //     printf("unable to allocate bgr frame!\n");
    //     return;
    // }

    // printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %p\n",
    //        frame->frame_format, frame->width, frame->height, frame->data_bytes, user_ptr);
    printf("frame_size: %d \n", frame->data_bytes);
    // return;
    // printf("kkkkkkkkkkkkkkkk~~~~~~~~~~~~~\n");
    // uint8_t* in = (uint8_t *)frame->data;
    // uint8_t flag = (*in & 0xc0);
    // *in = (*in & 0x0f);
    // if (flag == 0xc0)
    // {
    //     curFreq = 77;
    //     // printf("curFreq: %d ", (int)curFreq);
    //     return;
    // }
    // else if (flag == 0x80)
    // {
    //     curFreq = 11;
    //     // printf("curFreq: %d ", (int)curFreq);
    // }
    // else
    // {
    //     printf("callbackTOFData:Error Can't get dealias freq! flag:0x%x\n", flag);
    // }

    // int tillumOri = *(uint8_t*)(in + TILLUM_OFFSET + 3) << 24 | *(uint8_t*)(in + TILLUM_OFFSET + 2) << 16 | *(uint8_t*)(in + TILLUM_OFFSET + 1) << 8 | *(uint8_t*)(in + TILLUM_OFFSET);
    // int tsensorOri = *((int8_t*)in + TSENSOR_OFFSET);
    // uint8_t *pyuv = (uint8_t *)frame->data;
    // int idx = 0;
    // size_t len = PHASE_LENGTH;
    // uint16_t* depth = (uint16_t *)pyuv;
    // uint16_t* amp = (uint16_t *)pyuv+len;
    // amplitude_data.resize(PHASE_LENGTH);
    // phase_data.resize(PHASE_LENGTH);
    // printf("center_pha: %d , center_amp: %d ,tillum:%d, tsensor: %d \n",depth[320*120+160],amp[320*120+160], tillumOri,tsensorOri);

    // int gop = getOverexposedPixels(devh);
    // printf("cur get Overexposed Pixels:%d\n",gop);

// #define rawimage
// #ifdef rawimage
//     static int count4 = 0;
//     if (count4 % 1 == 0)
//     {
//         char file_name[100] = {0};
//         snprintf(file_name, 100, "./data/_gray_%d.raw", count4);
//         FILE *gray_file = fopen(file_name, "wb");
//         if (gray_file)
//         {
//             fwrite(amp, PHASE_LENGTH, 2, gray_file);
//             fclose(gray_file);
//             printf("vertical record file: %s\n", file_name);
//         }
//         // else
//         // {
//         //     printf("[%s] open raw file: %s failed\n",__func__,file_name);
//         // }
//         snprintf(file_name, 100, "./data/_depth_%d.raw", count4);
//         FILE *depth_file = fopen(file_name, "wb");
//         if (depth_file)
//         {
//             fwrite(depth, PHASE_LENGTH, 2, depth_file);
//             fclose(depth_file);
//             printf("vertical record file: %s\n", file_name);
//         }
//         // else
//         // {
//         //     printf("[%s] open raw file: %s failed\n",__func__,file_name);
//         // }
//     }
//     ++count4;
// #endif
}

bool start(uvc_device_handle_t *devh, uvc_stream_ctrl_t *ctrl, void *user_ptr)
{
    uvc_error_t res;
    res = uvc_start_streaming(devh, ctrl, callback, user_ptr, (uint8_t)5);
    if (res < 0)
    {
        uvc_perror(res, "start_streaming");
        return false;
    }
    return true;
}

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "Received Ctrl+C signal. Exiting..." << std::endl;
        stop_requested = true;
    }
}

int main()
{
    signal(SIGINT, signal_handler);
    uvc_context_t *ctx;
    uvc_error_t res;
    uvc_stream_ctrl_t ctrl;
    uvc_device_handle_t *devh;
    res = uvc_init(&ctx, NULL); // 初始化uvc
    if (res < 0)
    {
        uvc_perror(res, "uvc_init");
        return res;
    }
    uvc_device_t **devs;
    res = uvc_get_device_list(ctx, &devs); // 获取uvc列表
    
    if (res < 0)
    {
        uvc_perror(res, "uvc_get_device_list");
        return res;
    }
    uvc_device_t *dev = devs[0];
    if (devs == NULL || devs[0] == NULL)
    {
        fprintf(stderr, "No devices found.\n");
        uvc_free_device_list(devs, 1); // Clean up device list before returning
        uvc_exit(ctx);                 // Clean up context before returning
        return -1;                     // Or any appropriate error code
    }
  
#ifdef pointcloud
    int interface_idx = 3;
    int camera_num = 0;
    res = uvc_get_stream_num(dev,&camera_num); // 获取数据流
    if(interface_idx > camera_num)
        interface_idx = 1;
    res = uvc_open2(dev, &devh,interface_idx-1); // 打开uvc设备
#else
    res = uvc_open(dev, &devh);
    if (res < 0)
    {
        uvc_perror(res, "uvc_open failed.");
        return res;
    }
#endif

    // #endif
    printf("open devices success~\n");
    res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 320, 480, 30); // 配置数据流参数
    if (res < 0)
    {
        uvc_perror(res, "uvc_get_stream_ctrl_format_size");
        return -1;
    }
    uvc_print_stream_ctrl(&ctrl, stderr);
    // res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);
    res = uvc_start_streaming(devh, &ctrl, callback, user_ptr, (uint8_t)0);
    if (res < 0)
    {
        uvc_perror(res, "start_streaming");
        return -1;
    }
    // if(!start(devh, &ctrl,user_ptr)) // 开始 接收数据流
    // {
    //     printf("start camera fail!\n");
    //     return -1;
    // }

    // while (!stop_requested) // ctrl+c exit output data
    // {
    //     usleep(1000);
    //     // run loop
    // }
    // uvc_stop_streaming(devh);
exit:
    stop(devh);
    puts("Done streaming.");

    uvc_close(devh);
    puts("Device closed");

    uvc_unref_device(dev);
    uvc_exit(ctx);
    puts("UVC exited");
    return 0;
}
