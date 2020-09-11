#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// -- define v4l2 ---------------
#include <assert.h>
#include <fcntl.h>
#include <libusb-1.0/libusb.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/ioctl.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "colormap.h"
#include "font5x7.h"

#define VIDEO_DEVICE0 "/dev/video1"  // gray scale thermal image
#define FRAME_WIDTH0 160
#define FRAME_HEIGHT0 120

#define VIDEO_DEVICE1 "/dev/video2"  // color visible image
#define FRAME_WIDTH1 640
#define FRAME_HEIGHT1 480

#define VIDEO_DEVICE2 "/dev/video3"  // colorized thermal image
#define FRAME_WIDTH2 160
#define FRAME_HEIGHT2 128

#define FRAME_FORMAT0 V4L2_PIX_FMT_GREY
#define FRAME_FORMAT1 V4L2_PIX_FMT_MJPEG
#define FRAME_FORMAT2 V4L2_PIX_FMT_RGB24

constexpr std::array<unsigned char, 768> kColormapRainbow = {
    0x01, 0x03, 0x4a, 0x00, 0x03, 0x4a, 0x00, 0x03, 0x4b, 0x00, 0x03, 0x4b,
    0x00, 0x03, 0x4c, 0x00, 0x03, 0x4c, 0x00, 0x03, 0x4d, 0x00, 0x03, 0x4f,
    0x00, 0x03, 0x52, 0x00, 0x05, 0x55, 0x00, 0x07, 0x58, 0x00, 0x0a, 0x5b,
    0x00, 0x0e, 0x5e, 0x00, 0x13, 0x62, 0x00, 0x16, 0x64, 0x00, 0x19, 0x67,
    0x00, 0x1c, 0x6a, 0x00, 0x20, 0x6d, 0x00, 0x23, 0x70, 0x00, 0x26, 0x74,
    0x00, 0x28, 0x77, 0x00, 0x2a, 0x7b, 0x00, 0x2d, 0x80, 0x00, 0x31, 0x85,
    0x00, 0x32, 0x86, 0x00, 0x33, 0x88, 0x00, 0x34, 0x89, 0x00, 0x35, 0x8b,
    0x00, 0x36, 0x8e, 0x00, 0x37, 0x90, 0x00, 0x38, 0x91, 0x00, 0x3a, 0x95,
    0x00, 0x3d, 0x9a, 0x00, 0x3f, 0x9c, 0x00, 0x41, 0x9f, 0x00, 0x42, 0xa1,
    0x00, 0x44, 0xa4, 0x00, 0x45, 0xa7, 0x00, 0x47, 0xaa, 0x00, 0x49, 0xae,
    0x00, 0x4b, 0xb3, 0x00, 0x4c, 0xb5, 0x00, 0x4e, 0xb8, 0x00, 0x4f, 0xbb,
    0x00, 0x50, 0xbc, 0x00, 0x51, 0xbe, 0x00, 0x54, 0xc2, 0x00, 0x57, 0xc6,
    0x00, 0x58, 0xc8, 0x00, 0x5a, 0xcb, 0x00, 0x5c, 0xcd, 0x00, 0x5e, 0xcf,
    0x00, 0x5e, 0xd0, 0x00, 0x5f, 0xd1, 0x00, 0x60, 0xd2, 0x00, 0x61, 0xd3,
    0x00, 0x63, 0xd6, 0x00, 0x66, 0xd9, 0x00, 0x67, 0xda, 0x00, 0x68, 0xdb,
    0x00, 0x69, 0xdc, 0x00, 0x6b, 0xdd, 0x00, 0x6d, 0xdf, 0x00, 0x6f, 0xdf,
    0x00, 0x71, 0xdf, 0x00, 0x73, 0xde, 0x00, 0x75, 0xdd, 0x00, 0x76, 0xdc,
    0x01, 0x78, 0xdb, 0x01, 0x7a, 0xd9, 0x02, 0x7c, 0xd8, 0x02, 0x7e, 0xd6,
    0x03, 0x81, 0xd4, 0x03, 0x83, 0xcf, 0x04, 0x84, 0xcd, 0x04, 0x85, 0xca,
    0x04, 0x86, 0xc5, 0x05, 0x88, 0xc0, 0x06, 0x8a, 0xb9, 0x07, 0x8d, 0xb2,
    0x08, 0x8e, 0xac, 0x0a, 0x90, 0xa6, 0x0a, 0x90, 0xa2, 0x0b, 0x91, 0x9e,
    0x0c, 0x92, 0x99, 0x0d, 0x93, 0x95, 0x0f, 0x95, 0x8c, 0x11, 0x97, 0x84,
    0x16, 0x99, 0x78, 0x19, 0x9a, 0x73, 0x1c, 0x9c, 0x6d, 0x22, 0x9e, 0x65,
    0x28, 0xa0, 0x5e, 0x2d, 0xa2, 0x56, 0x33, 0xa4, 0x4f, 0x3b, 0xa7, 0x45,
    0x43, 0xab, 0x3c, 0x48, 0xad, 0x36, 0x4e, 0xaf, 0x30, 0x53, 0xb1, 0x2b,
    0x59, 0xb3, 0x27, 0x5d, 0xb5, 0x23, 0x62, 0xb7, 0x1f, 0x69, 0xb9, 0x1a,
    0x6d, 0xbb, 0x17, 0x71, 0xbc, 0x15, 0x76, 0xbd, 0x13, 0x7b, 0xbf, 0x11,
    0x80, 0xc1, 0x0e, 0x86, 0xc3, 0x0c, 0x8a, 0xc4, 0x0a, 0x8e, 0xc5, 0x08,
    0x92, 0xc6, 0x06, 0x97, 0xc8, 0x05, 0x9b, 0xc9, 0x04, 0xa0, 0xcb, 0x03,
    0xa4, 0xcc, 0x02, 0xa9, 0xcd, 0x02, 0xad, 0xce, 0x01, 0xaf, 0xcf, 0x01,
    0xb2, 0xcf, 0x01, 0xb8, 0xd0, 0x00, 0xbe, 0xd2, 0x00, 0xc1, 0xd3, 0x00,
    0xc4, 0xd4, 0x00, 0xc7, 0xd4, 0x00, 0xca, 0xd5, 0x01, 0xcf, 0xd6, 0x02,
    0xd4, 0xd7, 0x03, 0xd7, 0xd6, 0x03, 0xda, 0xd6, 0x03, 0xdc, 0xd5, 0x03,
    0xde, 0xd5, 0x04, 0xe0, 0xd4, 0x04, 0xe1, 0xd4, 0x05, 0xe2, 0xd4, 0x05,
    0xe5, 0xd3, 0x05, 0xe8, 0xd3, 0x06, 0xe8, 0xd3, 0x06, 0xe9, 0xd3, 0x06,
    0xea, 0xd2, 0x06, 0xeb, 0xd2, 0x07, 0xec, 0xd1, 0x07, 0xed, 0xd0, 0x08,
    0xef, 0xce, 0x08, 0xf1, 0xcc, 0x09, 0xf2, 0xcb, 0x09, 0xf4, 0xca, 0x0a,
    0xf4, 0xc9, 0x0a, 0xf5, 0xc8, 0x0a, 0xf5, 0xc7, 0x0b, 0xf6, 0xc6, 0x0b,
    0xf7, 0xc5, 0x0c, 0xf8, 0xc2, 0x0d, 0xf9, 0xbf, 0x0e, 0xfa, 0xbd, 0x0e,
    0xfb, 0xbb, 0x0f, 0xfb, 0xb9, 0x10, 0xfc, 0xb7, 0x11, 0xfc, 0xb2, 0x12,
    0xfd, 0xae, 0x13, 0xfd, 0xab, 0x13, 0xfe, 0xa8, 0x14, 0xfe, 0xa5, 0x15,
    0xfe, 0xa4, 0x15, 0xff, 0xa3, 0x16, 0xff, 0xa1, 0x16, 0xff, 0x9f, 0x17,
    0xff, 0x9d, 0x17, 0xff, 0x9b, 0x18, 0xff, 0x95, 0x19, 0xff, 0x8f, 0x1b,
    0xff, 0x8b, 0x1c, 0xff, 0x87, 0x1e, 0xff, 0x83, 0x1f, 0xff, 0x7f, 0x20,
    0xff, 0x76, 0x22, 0xff, 0x6e, 0x24, 0xff, 0x68, 0x25, 0xff, 0x65, 0x26,
    0xff, 0x63, 0x27, 0xff, 0x5d, 0x28, 0xff, 0x58, 0x2a, 0xfe, 0x52, 0x2b,
    0xfe, 0x4d, 0x2d, 0xfe, 0x45, 0x2f, 0xfe, 0x3e, 0x31, 0xfd, 0x39, 0x32,
    0xfd, 0x35, 0x34, 0xfc, 0x31, 0x35, 0xfc, 0x2d, 0x37, 0xfb, 0x27, 0x39,
    0xfb, 0x21, 0x3b, 0xfb, 0x20, 0x3c, 0xfb, 0x1f, 0x3c, 0xfb, 0x1e, 0x3d,
    0xfb, 0x1d, 0x3d, 0xfb, 0x1c, 0x3e, 0xfa, 0x1b, 0x3f, 0xfa, 0x1b, 0x41,
    0xf9, 0x1a, 0x42, 0xf9, 0x1a, 0x44, 0xf8, 0x19, 0x46, 0xf8, 0x18, 0x49,
    0xf7, 0x18, 0x4b, 0xf7, 0x19, 0x4d, 0xf7, 0x19, 0x4f, 0xf7, 0x1a, 0x51,
    0xf7, 0x20, 0x53, 0xf7, 0x23, 0x55, 0xf7, 0x26, 0x56, 0xf7, 0x2a, 0x58,
    0xf7, 0x2e, 0x5a, 0xf7, 0x32, 0x5c, 0xf8, 0x37, 0x5e, 0xf8, 0x3b, 0x60,
    0xf8, 0x40, 0x62, 0xf8, 0x48, 0x65, 0xf9, 0x51, 0x68, 0xf9, 0x57, 0x6a,
    0xfa, 0x5d, 0x6c, 0xfa, 0x5f, 0x6d, 0xfa, 0x62, 0x6e, 0xfa, 0x64, 0x6f,
    0xfb, 0x65, 0x70, 0xfb, 0x66, 0x71, 0xfb, 0x6d, 0x75, 0xfc, 0x74, 0x79,
    0xfc, 0x79, 0x7b, 0xfd, 0x7e, 0x7e, 0xfd, 0x82, 0x80, 0xfe, 0x87, 0x83,
    0xfe, 0x8b, 0x85, 0xfe, 0x90, 0x88, 0xfe, 0x97, 0x8c, 0xff, 0x9e, 0x90,
    0xff, 0xa3, 0x92, 0xff, 0xa8, 0x95, 0xff, 0xad, 0x98, 0xff, 0xb0, 0x99,
    0xff, 0xb2, 0x9b, 0xff, 0xb8, 0xa0, 0xff, 0xbf, 0xa5, 0xff, 0xc3, 0xa8,
    0xff, 0xc7, 0xac, 0xff, 0xcb, 0xaf, 0xff, 0xcf, 0xb3, 0xff, 0xd3, 0xb6,
    0xff, 0xd8, 0xb9, 0xff, 0xda, 0xbe, 0xff, 0xdc, 0xc4, 0xff, 0xde, 0xc8,
    0xff, 0xe1, 0xca, 0xff, 0xe3, 0xcc, 0xff, 0xe6, 0xce, 0xff, 0xe9, 0xd0};

ABSL_FLAG(std::string, colormap, "", "Pallete.");

// -- define Flir calibration values ---------------
// exiftool -plan* FLIROne-2015-11-30-17-26-48+0100.jpg

constexpr double kPlanckR1 = 16528.178;
constexpr double kPlanckB = 1427.5;
constexpr double kPlanckF = 1.0;
constexpr double kPlanckO = -1307.0;
constexpr double kPlanckR2 = 0.012258549;

// Reflected Apparent Temperature [°C]
constexpr double kTempReflected = 20.0;
// Emissivity of object
constexpr double kEmissivity = 0.95;

struct v4l2_capability vid_caps0;
struct v4l2_capability vid_caps1;
struct v4l2_capability vid_caps2;

struct v4l2_format vid_format0;
struct v4l2_format vid_format1;
struct v4l2_format vid_format2;

size_t framesize0;
size_t linewidth0;

size_t framesize1;
size_t linewidth1;

size_t framesize2;
size_t linewidth2;

const char *video_device0 = VIDEO_DEVICE0;
const char *video_device1 = VIDEO_DEVICE1;
const char *video_device2 = VIDEO_DEVICE2;

int fdwr0 = 0;
int fdwr1 = 0;
int fdwr2 = 0;

// -- end define v4l2 ---------------

#define VENDOR_ID 0x09cb
#define PRODUCT_ID 0x1996

static struct libusb_device_handle *devh = nullptr;
int filecount = 0;
struct timeval t1, t2;
long long fps_t;

int FFC = 0;  // detect FFC

// -- buffer for EP 0x85 chunks ---------------
#define BUF85SIZE 1048576  // size got from android app
int buf85pointer = 0;
unsigned char buf85[BUF85SIZE];

void print_format(struct v4l2_format *vid_format) {
  printf("     vid_format->type                =%d\n", vid_format->type);
  printf("     vid_format->fmt.pix.width       =%d\n",
         vid_format->fmt.pix.width);
  printf("     vid_format->fmt.pix.height      =%d\n",
         vid_format->fmt.pix.height);
  printf("     vid_format->fmt.pix.pixelformat =%d\n",
         vid_format->fmt.pix.pixelformat);
  printf("     vid_format->fmt.pix.sizeimage   =%u\n",
         vid_format->fmt.pix.sizeimage);
  printf("     vid_format->fmt.pix.field       =%d\n",
         vid_format->fmt.pix.field);
  printf("     vid_format->fmt.pix.bytesperline=%d\n",
         vid_format->fmt.pix.bytesperline);
  printf("     vid_format->fmt.pix.colorspace  =%d\n",
         vid_format->fmt.pix.colorspace);
}

void font_write(unsigned char *fb, int x, int y, const char *string) {
  int rx, ry;
  while (*string) {
    for (ry = 0; ry < 5; ++ry) {
      for (rx = 0; rx < 7; ++rx) {
        int v =
            (kFont5x7Basic[((*string) & 0x7F) - kCharOffset][ry] >> (rx)) & 1;
        //	fb[(y+ry) * 160 + (x + rx)] = v ? 0 : 0xFF; // black / white
        //	fb[(y+rx) * 160 + (x + ry)] = v ? 0 : 0xFF; // black / white

        fb[(y + rx) * 160 + (x + ry)] =
            v ? 0 : fb[(y + rx) * 160 + (x + ry)];  // transparent
      }
    }
    string++;
    x += 6;
  }
}

double raw2temperature(uint8_t raw) {
  // calc amount of radiance of reflected objects ( Emissivity < 1 )
  const double raw_refl =
      kPlanckR1 / (kPlanckR2 * (std::exp(kPlanckB / (kTempReflected + 273.15)) -
                                kPlanckF)) -
      kPlanckO;
  // get displayed object temp max/min
  double raw_obj = (4.0 * raw - (1.0 - kEmissivity) * raw_refl) / kEmissivity;
  // calc object temperature
  return kPlanckB / std::log(kPlanckR1 / (kPlanckR2 * (raw_obj + kPlanckO)) +
                             kPlanckF) -
         273.15;
}

void startv4l2() {
  int ret_code = 0;
  /*
  //open video_device0
       printf("using output device: %s\n", video_device0);

       fdwr0 = open(video_device0, O_RDWR);
       assert(fdwr0 >= 0);

       ret_code = ioctl(fdwr0, VIDIOC_QUERYCAP, &vid_caps0);
       assert(ret_code != -1);

       memset(&vid_format0, 0, sizeof(vid_format0));

       ret_code = ioctl(fdwr0, VIDIOC_G_FMT, &vid_format0);

       linewidth0=FRAME_WIDTH0;
       framesize0=FRAME_WIDTH0*FRAME_HEIGHT0*1; // 8 Bit

       vid_format0.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
       vid_format0.fmt.pix.width = FRAME_WIDTH0;
       vid_format0.fmt.pix.height = FRAME_HEIGHT0;
       vid_format0.fmt.pix.pixelformat = FRAME_FORMAT0;
       vid_format0.fmt.pix.sizeimage = framesize0;
       vid_format0.fmt.pix.field = V4L2_FIELD_NONE;
       vid_format0.fmt.pix.bytesperline = linewidth0;
       vid_format0.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

       // set data format
       ret_code = ioctl(fdwr0, VIDIOC_S_FMT, &vid_format0);
       assert(ret_code != -1);

       print_format(&vid_format0);
  */
  // open video_device1
  std::cout << "Using output device" << video_device1 << std::endl;

  fdwr1 = open(video_device1, O_RDWR);
  assert(fdwr1 >= 0);

  ret_code = ioctl(fdwr1, VIDIOC_QUERYCAP, &vid_caps1);
  assert(ret_code != -1);

  memset(&vid_format1, 0, sizeof(vid_format1));

  ret_code = ioctl(fdwr1, VIDIOC_G_FMT, &vid_format1);

  linewidth1 = FRAME_WIDTH1;
  framesize1 = FRAME_WIDTH1 * FRAME_HEIGHT1 * 1;  // 8 Bit ??

  vid_format1.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  vid_format1.fmt.pix.width = FRAME_WIDTH1;
  vid_format1.fmt.pix.height = FRAME_HEIGHT1;
  vid_format1.fmt.pix.pixelformat = FRAME_FORMAT1;
  vid_format1.fmt.pix.sizeimage = framesize1;
  vid_format1.fmt.pix.field = V4L2_FIELD_NONE;
  vid_format1.fmt.pix.bytesperline = linewidth1;
  vid_format1.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

  // set data format
  ret_code = ioctl(fdwr1, VIDIOC_S_FMT, &vid_format1);
  assert(ret_code != -1);

  print_format(&vid_format1);

  // open video_device2
  printf("using output device: %s\n", video_device2);

  fdwr2 = open(video_device2, O_RDWR);
  assert(fdwr2 >= 0);

  ret_code = ioctl(fdwr2, VIDIOC_QUERYCAP, &vid_caps2);
  assert(ret_code != -1);

  memset(&vid_format2, 0, sizeof(vid_format2));

  ret_code = ioctl(fdwr2, VIDIOC_G_FMT, &vid_format2);

  linewidth2 = FRAME_WIDTH2;
  framesize2 = FRAME_WIDTH2 * FRAME_HEIGHT2 * 3;  // 8x8x8 Bit

  vid_format2.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  vid_format2.fmt.pix.width = FRAME_WIDTH2;
  vid_format2.fmt.pix.height = FRAME_HEIGHT2;
  vid_format2.fmt.pix.pixelformat = FRAME_FORMAT2;
  vid_format2.fmt.pix.sizeimage = framesize2;
  vid_format2.fmt.pix.field = V4L2_FIELD_NONE;
  vid_format2.fmt.pix.bytesperline = linewidth2;
  vid_format2.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

  // set data format
  ret_code = ioctl(fdwr2, VIDIOC_S_FMT, &vid_format2);
  assert(ret_code != -1);

  print_format(&vid_format2);
}

// unused
void closev4l2() {
  //     close(fdwr0);
  close(fdwr1);
  close(fdwr2);
}

void vframe(char ep[], char EP_error[], int r, int actual_length,
            unsigned char buf[], const unsigned char *colormap) {
  // error handler
  time_t now1;
  now1 = time(nullptr);
  if (r < 0) {
    if (strcmp(EP_error, libusb_error_name(r)) != 0) {
      strcpy(EP_error, libusb_error_name(r));
      fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n",
              ctime(&now1), ep, r, libusb_error_name(r));
      sleep(1);
    }
    return;
  }

  // reset buffer if the new chunk begins with magic bytes or the buffer size
  // limit is exceeded
  unsigned char magicbyte[4] = {0xEF, 0xBE, 0x00, 0x00};

  if ((std::memcmp(buf, magicbyte, 4) == 0) ||
      ((buf85pointer + actual_length) >= BUF85SIZE)) {
    // printf(">>>>>>>>>>>begin of new frame<<<<<<<<<<<<<\n");
    buf85pointer = 0;
  }

  // printf("actual_length %d !!!!!\n", actual_length);

  memmove(buf85 + buf85pointer, buf, actual_length);
  buf85pointer = buf85pointer + actual_length;

  if ((std::memcmp(buf85, magicbyte, 4) != 0)) {
    // reset buff pointer
    buf85pointer = 0;
    printf("Reset buffer because of bad Magic Byte!\n");
    return;
  }

  // a quick and dirty job for gcc
  int FrameSize =
      buf85[8] + (buf85[9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
  int ThermalSize =
      buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
  int JpgSize =
      buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
  // uint32_t StatusSize =
  //     buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

  // printf("FrameSize= %d (+28=%d), ThermalSize %d, JPG %d, StatusSize %d,
  // Pointer %d\n",FrameSize,FrameSize+28, ThermalSize,
  // JpgSize,StatusSize,buf85pointer);

  if ((FrameSize + 28) > (buf85pointer)) {
    // wait for next chunk
    return;
  }

  int v;
  // get a full frame, first print the status
  t1 = t2;
  gettimeofday(&t2, nullptr);
  // fps as moving average over last 20 frames
  //  fps_t = (19*fps_t+10000000/(((t2.tv_sec * 1000000) + t2.tv_usec) -
  //  ((t1.tv_sec * 1000000) + t1.tv_usec)))/20;

  filecount++;
  //  printf("#%08i %lld/10 fps:",filecount,fps_t);
  //  for (i = 0; i <  StatusSize; i++) {
  //                    v=28+ThermalSize+JpgSize+i;
  //                    if(buf85[v]>31) {printf("%c", buf85[v]);}
  //            }
  //  printf("\n");

  buf85pointer = 0;

  unsigned short pix[160 * 120];  // original Flir 16 Bit RAW
  int x, y;

  // 8 Bit gray buffer really needs only 160 x 120
  auto fb_proc = std::vector<uint8_t>(160 * 128);
  std::fill(fb_proc.begin(), fb_proc.end(), 128);

  // 8x8x8  Bit RGB buffer
  auto fb_proc2 = std::vector<uint8_t>(160 * 128 * 3);

  int min = 0x10000, max = 0;
  float rms = 0;

  // Make a unsigned short array from what comes from the thermal frame
  // find the max, min and RMS (not used yet) values of the array
  int maxx, maxy;
  for (y = 0; y < 120; ++y) {
    for (x = 0; x < 160; ++x) {
      if (x < 80)
        v = buf85[2 * (y * 164 + x) + 32] + 256 * buf85[2 * (y * 164 + x) + 33];
      else
        v = buf85[2 * (y * 164 + x) + 32 + 4] +
            256 * buf85[2 * (y * 164 + x) + 33 + 4];
      pix[y * 160 + x] = v;  // unsigned char!!

      if (v < min) min = v;
      if (v > max) {
        max = v;
        maxx = x;
        maxy = y;
      }
      rms += v * v;
    }
  }

  // RMS used later
  //  rms /= 160 * 120;
  //  rms = sqrtf(rms);

  // scale the data in the array
  int delta = max - min;
  if (!delta) delta = 1;  // if max = min we have divide by zero
  int scale = 0x10000 / delta;

  for (y = 0; y < 120; ++y)  // 120
  {
    for (x = 0; x < 160; ++x) {  // 160
      int v = (pix[y * 160 + x] - min) * scale >> 8;

      // fb_proc is the gray scale frame buffer
      fb_proc[y * 160 + x] = v;  // unsigned char!!
    }
  }

  char st1[100];
  char st2[100];
  struct tm *loctime;
  // Convert it to local time and Print it out in a nice format.
  loctime = localtime(&now1);
  strftime(st1, 60, "%H:%M:%S", loctime);

  // calc medium of 2x2 center pixels
  int med = (pix[59 * 160 + 79] + pix[59 * 160 + 80] + pix[60 * 160 + 79] +
             pix[60 * 160 + 80]) /
            4;
  sprintf(st2, " %.1f/%.1f/%.1f'C", raw2temperature(min), raw2temperature(med),
          raw2temperature(max));
  strcat(st1, st2);

#define MAX 26  // max chars in line  160/6=26,6
  std::strncpy(st2, st1, MAX);
  // write zero to string !!
  st2[MAX - 1] = '\0';
  font_write(fb_proc.data(), 1, 120, st2);

  // show crosshairs, remove if required
  font_write(fb_proc.data(), 80 - 2, 60 - 3, "+");

  maxx -= 4;
  maxy -= 4;

  if (maxx < 0) maxx = 0;
  if (maxy < 0) maxy = 0;
  if (maxx > 150) maxx = 150;
  if (maxy > 110) maxy = 110;

  font_write(fb_proc.data(), 160 - 6, maxy, "<");
  font_write(fb_proc.data(), maxx, 120 - 8, "|");

  for (y = 0; y < 128; ++y) {
    for (x = 0; x < 160; ++x) {
      // fb_proc is the gray scale frame buffer
      v = fb_proc[y * 160 + x];  // unsigned char!!

      // fb_proc2 is an 24bit RGB buffer

      fb_proc2[3 * y * 160 + x * 3] = colormap[3 * v];  // unsigned char!!
      fb_proc2[(3 * y * 160 + x * 3) + 1] =
          colormap[3 * v + 1];  // unsigned char!!
      fb_proc2[(3 * y * 160 + x * 3) + 2] =
          colormap[3 * v + 2];  // unsigned char!!
    }
  }

  // write video to v4l2loopback(s)
  //   write(fdwr0, fb_proc, framesize0);  // gray scale Thermal Image
  write(fdwr1, &buf85[28 + ThermalSize], JpgSize);  // jpg Visual Image

  if (std::memcmp(&buf85[28 + ThermalSize + JpgSize + 17], "FFC", 3) == 0) {
    FFC = 1;  // drop all FFC frames
  } else {
    if (FFC == 1) {
      FFC = 0;  // drop first frame after FFC
    } else {
      write(fdwr2, fb_proc2.data(), framesize2);  // colorized RGB Thermal Image
    }
  }
}

static int find_lvr_flirusb(void) {
  devh = libusb_open_device_with_vid_pid(nullptr, VENDOR_ID, PRODUCT_ID);
  return devh ? 0 : -EIO;
}

void print_bulk_result(char ep[], char EP_error[], int r, int actual_length,
                       unsigned char buf[]) {
  time_t now1;
  int i;

  now1 = time(nullptr);
  if (r < 0) {
    if (strcmp(EP_error, libusb_error_name(r)) != 0) {
      strcpy(EP_error, libusb_error_name(r));
      fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n",
              ctime(&now1), ep, r, libusb_error_name(r));
      sleep(1);
    }
    // return 1;
  } else {
    printf("\n: %s bulk read EP %s, actual length %d\nHEX:\n", ctime(&now1), ep,
           actual_length);
    // write frame to file
    /*
              char filename[100];
              sprintf(filename, "EP%s#%05i.bin",ep,filecount);
              filecount++;
              FILE *file = fopen(filename, "wb");
              fwrite(buf, 1, actual_length, file);
              fclose(file);
    */
    // hex print of first byte
    for (i = 0; i < (((200) < (actual_length)) ? (200) : (actual_length));
         i++) {
      printf(" %02x", buf[i]);
    }

    printf("\nSTRING:\n");
    for (i = 0; i < (((200) < (actual_length)) ? (200) : (actual_length));
         i++) {
      if (buf[i] > 31) {
        printf("%c", buf[i]);
      }
    }
    printf("\n");
  }
}

int EPloop(const unsigned char *colormap) {
  int r = 1;
  auto out = [&] {
    // close the device
    libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(nullptr);
    return r >= 0 ? r : -r;
  };

  r = libusb_init(nullptr);
  if (r < 0) {
    fprintf(stderr, "failed to initialise libusb\n");
    exit(1);
  }

  r = find_lvr_flirusb();
  if (r < 0) {
    fprintf(stderr, "Could not find/open device\n");
    return out();
  }
  printf("Successfully find the Flir One G2 device\n");

  r = libusb_set_configuration(devh, 3);
  if (r < 0) {
    fprintf(stderr, "libusb_set_configuration error %d\n", r);
    return out();
  }
  printf("Successfully set usb configuration 3\n");

  // Claiming of interfaces is a purely logical operation;
  // it does not cause any requests to be sent over the bus.
  r = libusb_claim_interface(devh, 0);
  if (r < 0) {
    fprintf(stderr, "libusb_claim_interface 0 error %d\n", r);
    return out();
  }
  r = libusb_claim_interface(devh, 1);
  if (r < 0) {
    fprintf(stderr, "libusb_claim_interface 1 error %d\n", r);
    return out();
  }
  r = libusb_claim_interface(devh, 2);
  if (r < 0) {
    fprintf(stderr, "libusb_claim_interface 2 error %d\n", r);
    return out();
  }
  printf("Successfully claimed interface 0,1,2\n");

  unsigned char buf[1048576];
  int actual_length;

  time_t now;
  // save last error status to avoid clutter the log
  // char EP81_error[50] = "";
  // char EP83_error[50] = "";
  char EP85_error[50] = "";
  unsigned char data[2] = {0, 0};  // only a bad dummy

  // don't forget: $ sudo modprobe v4l2loopback video_nr=0,1
  startv4l2();

  int state = 1;

  while (1) {
    switch (state) {
      case 1: {
        /* Flir config
        01 0b 01 00 01 00 00 00 c4 d5
        0 bmRequestType = 01
        1 bRequest = 0b
        2 wValue 0001 type (H) index (L)    stop=0/start=1 (Alternate Setting)
        4 wIndex 01                         interface 1/2
        5 wLength 00
        6 Data 00 00

        libusb_control_transfer (*dev_handle, bmRequestType, bRequest, wValue,
        wIndex, *data, wLength, timeout)
        */

        printf("stop interface 2 FRAME\n");
        r = libusb_control_transfer(devh, 1, 0x0b, 0, 2, data, 0, 100);
        if (r < 0) {
          fprintf(stderr, "Control Out error %d\n", r);
          return r;
        }

        printf("stop interface 1 FILEIO\n");
        r = libusb_control_transfer(devh, 1, 0x0b, 0, 1, data, 0, 100);
        if (r < 0) {
          fprintf(stderr, "Control Out error %d\n", r);
          return r;
        }

        printf("\nstart interface 1 FILEIO\n");
        r = libusb_control_transfer(devh, 1, 0x0b, 1, 1, data, 0, 100);
        if (r < 0) {
          fprintf(stderr, "Control Out error %d\n", r);
          return r;
        }
        now = time(0);  // Get the system time
        printf("\n:xx %s", ctime(&now));
        state = 3;  // jump over wait stait 2. Not really using any data from
        // CameraFiles.zip
      } break;
      case 2: {
        printf("\nask for CameraFiles.zip on EP 0x83:\n");
        now = time(0);  // Get the system time
        printf("\n: %s", ctime(&now));

        int transferred = 0;
        char my_string[128];

        //--------- write string:
        //{"type":"openFile","data":{"mode":"r","path":"CameraFiles.zip"}}
        int length = 16;
        unsigned char my_string2[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00,
                                        0x00, 0x00, 0x41, 0x00, 0x00, 0x00,
                                        0xF8, 0xB3, 0xF7, 0x00};
        printf("\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
        int i;
        for (i = 0; i < length; i++) {
          printf(" %02x", my_string2[i]);
        }
        printf(" ]\n");

        r = libusb_bulk_transfer(devh, 2, my_string2, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          printf("\nWrite successful!");
        } else
          printf("\nError in write! res = %d and transferred = %d\n", r,
                 transferred);

        strcpy(my_string,
               "{\"type\":\"openFile\",\"data\":{\"mode\":\"r\",\"path\":"
               "\"CameraFiles.zip\"}}");

        length = strlen(my_string) + 1;
        printf("\nEP 0x02 to be sent: %s", my_string);

        // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’
        // [-fpermissive]
        unsigned char *my_string1 = (unsigned char *)my_string;
        // my_string1 = (unsigned char*)my_string;

        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          printf("\nWrite successful!");
          printf("\nSent %d bytes with string: %s\n", transferred, my_string);
        } else
          printf("\nError in write! res = %d and transferred = %d\n", r,
                 transferred);

        //--------- write string:
        //{"type":"readFile","data":{"streamIdentifier":10}}
        length = 16;
        unsigned char my_string3[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00,
                                        0x00, 0x00, 0x33, 0x00, 0x00, 0x00,
                                        0xef, 0xdb, 0xc1, 0xc1};
        printf("\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
        for (i = 0; i < length; i++) {
          printf(" %02x", my_string3[i]);
        }
        printf(" ]\n");

        r = libusb_bulk_transfer(devh, 2, my_string3, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          printf("\nWrite successful!");
        } else
          printf("\nError in write! res = %d and transferred = %d\n", r,
                 transferred);

        // strcpy(  my_string,
        // "{\"type\":\"setOption\",\"data\":{\"option\":\"autoFFC\",\"value\":true}}");
        strcpy(my_string,
               "{\"type\":\"readFile\",\"data\":{\"streamIdentifier\":10}}");
        length = strlen(my_string) + 1;
        printf("\nEP 0x02 to be sent %i Bytes: %s", length, my_string);

        // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’
        // [-fpermissive]
        my_string1 = (unsigned char *)my_string;

        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          printf("\nWrite successful!");
          printf("\nSent %d bytes with string: %s\n", transferred, my_string);
        } else
          printf("\nError in write! res = %d and transferred = %d\n", r,
                 transferred);

        // go to next state
        now = time(0);  // Get the system time
        printf("\n: %s", ctime(&now));
        // sleep(1);
        state = 3;
      } break;
      case 3: {
        printf("\nAsk for video stream, start EP 0x85:\n");

        r = libusb_control_transfer(devh, 1, 0x0b, 1, 2, data, 2, 200);
        if (r < 0) {
          fprintf(stderr, "Control Out error %d\n", r);
          return r;
        };

        state = 4;
      } break;
      case 4: {
        // endless loop
        // poll Frame Endpoints 0x85
        // don't change timeout=100ms !!
        r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length,
                                 100);
        if (actual_length > 0) {
          char foo[] = "0x85";
          vframe(foo, EP85_error, r, actual_length, buf, colormap);
        }
      } break;
    }

    // poll Endpoints 0x81, 0x83
    r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
    /*
            if (actual_length > 0 && actual_length <= 101)
            {



            char k[5];
            if (strncmp (&buf[32],"VoltageUpdate",13)==0)
            {
            printf("xx %d\n",actual_length);


            char *token, *string, *tofree, *string2;
    //	char l;
            strcpy(string,buf);
    //       string = buf;
    //	 assert(string != nullptr);
            printf("yy\n");

            for (i = 32; i <  (((200)<(actual_length))?(200):(actual_length));
    i++)
                    {
                        if(string[i]>31)
                        {
                        printf("%c", string[i]);
    //		    printf("%d ", i);
    //		    string2[i-32] = string[i];
                        }
                    }

               while ((token = strsep(&string, ":")) != nullptr)
                {
                printf("zz\n");
                printf("%s\n", token);
                }

    //           free(tofree);
    //        for (i = 32; i <  (((200)<(actual_length))?(200):(actual_length));
    i++) {
    //                    if(buf[i]>31) {printf("%c", buf[i]);}
    //            }


            }
            }


    */

    r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
    if (strcmp(libusb_error_name(r), "LIBUSB_ERROR_NO_DEVICE") == 0) {
      fprintf(stderr, "EP 0x83 LIBUSB_ERROR_NO_DEVICE -> reset USB\n");
      return out();
    }
    //        print_bulk_result("0x83",EP83_error, r, actual_length, buf);
  }

  // never reached ;-)
  return libusb_release_interface(devh, 0);
}

int main(int argc, char **argv) {
  absl::ParseCommandLine(argc, argv);
  std::vector<char> colormap(768);

  const std::string pallete_file = absl::GetFlag(FLAGS_colormap);
  ABSL_RAW_CHECK(!pallete_file.empty(), "Missing required --colormap");

  while (1) {
    EPloop(kColormapRainbow.data());
  }
  return EXIT_SUCCESS;
}
