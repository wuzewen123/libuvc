/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2010-2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <stdio.h>
#include <opencv2/highgui/highgui_c.h>

#include "libuvc/libuvc.h"

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
#define DEPTHEYE_FRAME_RATE_ID DEPTHEYE_BASE_ID + 3
#define DEPTHEYE_INTERGRAL_TIME DEPTHEYE_BASE_ID + 13
#define DEPTHEYE_FIRMWARE_VERSION DEPTHEYE_BASE_ID + 17
#define DEPTHEYE_GET_TSENSOR DEPTHEYE_BASE_ID + 22
#define DEPTHEYE_GET_TILLUM DEPTHEYE_BASE_ID + 23
#define DEPTHEYE_AE_ENABLE DEPTHEYE_BASE_ID + 24
#define DEPTHEYE_FILTER_ENABLE DEPTHEYE_BASE_ID + 25

void cb(uvc_frame_t *frame, void *ptr) {
  printf("callback! length = %u, ptr = %d\n", frame->data_bytes, (int) ptr);
}
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

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_error_t res;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;

  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device");
  } else {
    puts("Device found");

    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open");
    } else {
      puts("Device opened");

      // uvc_print_diag(devh, stderr);

      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 240, 360, 30
      );
      res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);
      
      if(setDeptheyeCMD(devh,DEPTHEYE_INTERGRAL_TIME,100,1)){
          printf("set intergratime sucess~~~~~~~~~\n");
      } else {
          printf("set intergratime fail~~~~~~~~~\n");
      }

      if(!setDeptheyeCMD(devh,DEPTHEYE_FRAME_RATE_ID,15,1))
      {
          printf("setFps fail~~~~~~~~~~~~\n");
      } else {
          printf("setFps sucess~~~~~~~~~~~~\n");
      } // 
          

      if(!setDeptheyeCMD(devh,DEPTHEYE_AE_ENABLE,1,1))
      {
          printf("setAE value fail~~~~~~~~~\n");
      } else {
          printf("setAE value sucess~~~~~~~~~~~~\n");
      } //  // set fan
      
      if(!setDeptheyeCMD(devh,DEPTHEYE_FILTER_ENABLE,1,1))
      {
          printf("set filter value fail~~~~~~~~~~~~~~~~~\n");
      } else {
          printf("set filter value sucess~~~~~~~~~~~~\n");
      } //  // 


      sleep(10000);
      uvc_stop_streaming(devh);
      puts("Done streaming.");


      uvc_close(devh);
      puts("Device closed");
    }

    uvc_unref_device(dev);
  }

  uvc_exit(ctx);
  puts("UVC exited");

  return 0;
}

