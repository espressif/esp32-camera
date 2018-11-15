// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stddef.h>
#include <string.h>
#include "img_converters.h"
#include "rom/tjpgd.h"
#include "esp_spiram.h"
#include "soc/efuse_reg.h"
#include "esp_heap_caps.h"
#include "yuv.h"
#include "sdkconfig.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char* TAG = "to_bmp";
#endif

static const int BMP_HEADER_LEN = 54;

typedef enum {
    JPEG_DIV_NONE,
    JPEG_DIV_2,
    JPEG_DIV_4,
    JPEG_DIV_8,
    JPEG_DIV_MAX
} jpeg_div_t;

typedef struct {
    uint32_t filesize;
    uint32_t reserved;
    uint32_t fileoffset_to_pixelarray;
    uint32_t dibheadersize;
    int32_t width;
    int32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
} bmp_header_t;

typedef struct {
    jpeg_div_t scale;
    const void * src;
    size_t len;
    size_t index;
    uint16_t width;
    uint16_t height;
    uint8_t * dst;
    size_t dstlen;
} jpg_frame_decoder_t;

const char * jpgd_errors[] = {
    "Succeeded",
    "Interrupted by output function",
    "Device error or wrong termination of input stream",
    "Insufficient memory pool for the image",
    "Insufficient stream input buffer",
    "Parameter error",
    "Data format error",
    "Right format but not supported",
    "Not supported JPEG standard"
};

static void *_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static uint32_t jpg_write_bmp(JDEC *decoder, void *bitmap, JRECT *rect)
{
    jpg_frame_decoder_t * jpeg = (jpg_frame_decoder_t *)decoder->device;
    size_t jw = jpeg->width*3;
    size_t t = rect->top * jw;
    size_t b = (rect->bottom * jw) + 1;
    size_t l = rect->left * 3;
    size_t w = (rect->right + 1 - rect->left) * 3;
    uint8_t *data = (uint8_t *)bitmap;
    uint8_t *out = jpeg->dst;
    uint8_t *o = out;
    size_t iy, ix;
    for(iy=t; iy<b; iy+=jw) {
        o = out+iy+l;
        for(ix=0; ix<w; ix+= 3) {
            o[ix] = data[ix+2];
            o[ix+1] = data[ix+1];
            o[ix+2] = data[ix];
        }
        data+=w;
    }
    return 1;
}

static uint32_t jpg_read_frame(JDEC *decoder, uint8_t *buf, uint32_t len)
{
    jpg_frame_decoder_t * jpeg = (jpg_frame_decoder_t *)decoder->device;
    if(buf) {
        memcpy(buf, (const uint8_t *)jpeg->src + jpeg->index, len);
    }
    jpeg->index += len;
    return len;
}

static uint8_t jpg_work_buffer[3100];

bool jpg2rgb888(const uint8_t *src, size_t src_len, uint8_t * out, jpeg_div_t scale)
{
    JDEC decoder;
    jpg_frame_decoder_t jpeg;
    jpeg.src = src;
    jpeg.len = src_len;
    jpeg.index = 0;
    jpeg.width = 0;
    jpeg.height = 0;
    jpeg.dst = out;
    jpeg.dstlen = 0;
    jpeg.scale = scale;

    JRESULT jres = jd_prepare(&decoder, jpg_read_frame, jpg_work_buffer, 3100, &jpeg);
    if(jres != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed! %s", jpgd_errors[jres]);
        return false;
    }
    jpeg.width = decoder.width / (1 << (uint8_t)(jpeg.scale));
    jpeg.height = decoder.height / (1 << (uint8_t)(jpeg.scale));
    jpeg.dstlen = jpeg.width*jpeg.height*3;

    jres = jd_decomp(&decoder, jpg_write_bmp, (uint8_t)jpeg.scale);
    if(jres != JDR_OK) {
        ESP_LOGE(TAG, "jd_decomp failed! %s", jpgd_errors[jres]);
        return false;
    }
    return true;
}

bool fmt2rgb888(const uint8_t *src_buf, size_t src_len, pixformat_t format, uint8_t * rgb_buf)
{
    int pix_count = 0;
    if(format == PIXFORMAT_JPEG) {
        return jpg2rgb888(src_buf, src_len, rgb_buf, JPEG_DIV_NONE);
    } else if(format == PIXFORMAT_RGB888) {
        memcpy(rgb_buf, src_buf, src_len);
    } else if(format == PIXFORMAT_RGB565) {
        int i;
        uint8_t hb, lb;
        pix_count = src_len / 2;
        for(i=0; i<pix_count; i++) {
            hb = *src_buf++;
            lb = *src_buf++;
            *rgb_buf++ = (lb & 0x1F) << 3;
            *rgb_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
            *rgb_buf++ = hb & 0xF8;
        }
    } else if(format == PIXFORMAT_GRAYSCALE) {
        int i;
        uint8_t b;
        pix_count = src_len;
        for(i=0; i<pix_count; i++) {
            b = *src_buf++;
            *rgb_buf++ = b;
            *rgb_buf++ = b;
            *rgb_buf++ = b;
        }
    } else if(format == PIXFORMAT_YUV422) {
        int i, maxi = pix_count / 2;
        uint8_t y0, y1, u, v;
        uint8_t r, g, b;
        pix_count = src_len / 2;
        for(i=0; i<maxi; i++) {
            y0 = *src_buf++;
            u = *src_buf++;
            y1 = *src_buf++;
            v = *src_buf++;

            yuv2rgb(y0, u, v, &r, &g, &b);
            *rgb_buf++ = b;
            *rgb_buf++ = g;
            *rgb_buf++ = r;

            yuv2rgb(y1, u, v, &r, &g, &b);
            *rgb_buf++ = b;
            *rgb_buf++ = g;
            *rgb_buf++ = r;
        }
    }
    return true;
}

bool jpg2bmp(const uint8_t *src, size_t src_len, uint8_t ** out, size_t * out_len)
{
    JDEC decoder;
    uint8_t * out_buf = NULL;
    jpg_frame_decoder_t jpeg;

    jpeg.src = src;
    jpeg.len = src_len;
    jpeg.index = 0;
    jpeg.width = 0;
    jpeg.height = 0;
    jpeg.dst = NULL;
    jpeg.dstlen = 0;
    jpeg.scale = JPEG_DIV_NONE;

    *out = jpeg.dst;
    *out_len = jpeg.dstlen;

    JRESULT jres = jd_prepare(&decoder, jpg_read_frame, jpg_work_buffer, 3100, &jpeg);
    if(jres != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed! %s", jpgd_errors[jres]);
        return false;
    }
    jpeg.width = decoder.width / (1 << (uint8_t)(jpeg.scale));
    jpeg.height = decoder.height / (1 << (uint8_t)(jpeg.scale));
    size_t output_size = jpeg.width*jpeg.height*3;

    //setup output buffer
    jpeg.dstlen = output_size+BMP_HEADER_LEN;
    out_buf = (uint8_t *)_malloc(jpeg.dstlen);
    if(!out_buf) {
        ESP_LOGE(TAG, "_malloc failed! %u", jpeg.dstlen);
        return false;
    }
    jpeg.dst = out_buf+BMP_HEADER_LEN;

    out_buf[0] = 'B';
    out_buf[1] = 'M';
    bmp_header_t * bitmap  = (bmp_header_t*)&out_buf[2];
    bitmap->reserved = 0;
    bitmap->filesize = jpeg.dstlen;
    bitmap->fileoffset_to_pixelarray = BMP_HEADER_LEN;
    bitmap->dibheadersize = 40;
    bitmap->width = jpeg.width;
    bitmap->height = -jpeg.height;//set negative for top to bottom
    bitmap->planes = 1;
    bitmap->bitsperpixel = 24;
    bitmap->compression = 0;
    bitmap->imagesize = output_size;
    bitmap->ypixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->xpixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->numcolorspallette = 0;
    bitmap->mostimpcolor = 0;

    jres = jd_decomp(&decoder, jpg_write_bmp, (uint8_t)jpeg.scale);

    if(jres != JDR_OK) {
        ESP_LOGE(TAG, "jd_decomp failed! %s", jpgd_errors[jres]);
        free(out_buf);
        return false;
    }

    *out = out_buf;
    *out_len = jpeg.dstlen;
    return true;
}

bool fmt2bmp(uint8_t *src, size_t src_len, uint16_t width, uint16_t height, pixformat_t format, uint8_t ** out, size_t * out_len)
{
    if(format == PIXFORMAT_JPEG) {
        return jpg2bmp(src, src_len, out, out_len);
    }

    *out = NULL;
    *out_len = 0;

    int pix_count = width*height;
    size_t out_size = (pix_count * 3) + BMP_HEADER_LEN;
    uint8_t * out_buf = (uint8_t *)_malloc(out_size);
    if(!out_buf) {
        ESP_LOGE(TAG, "_malloc failed! %u", out_size);
        return false;
    }

    out_buf[0] = 'B';
    out_buf[1] = 'M';
    bmp_header_t * bitmap  = (bmp_header_t*)&out_buf[2];
    bitmap->reserved = 0;
    bitmap->filesize = out_size;
    bitmap->fileoffset_to_pixelarray = BMP_HEADER_LEN;
    bitmap->dibheadersize = 40;
    bitmap->width = width;
    bitmap->height = -height;//set negative for top to bottom
    bitmap->planes = 1;
    bitmap->bitsperpixel = 24;
    bitmap->compression = 0;
    bitmap->imagesize = pix_count * 3;
    bitmap->ypixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->xpixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->numcolorspallette = 0;
    bitmap->mostimpcolor = 0;

    uint8_t * rgb_buf = out_buf + BMP_HEADER_LEN;
    uint8_t * src_buf = src;


    //convert data to RGB888
    if(format == PIXFORMAT_RGB888) {
        memcpy(rgb_buf, src_buf, pix_count*3);
    } else if(format == PIXFORMAT_RGB565) {
        int i;
        uint8_t hb, lb;
        for(i=0; i<pix_count; i++) {
            hb = *src_buf++;
            lb = *src_buf++;
            *rgb_buf++ = (lb & 0x1F) << 3;
            *rgb_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
            *rgb_buf++ = hb & 0xF8;
        }
    } else if(format == PIXFORMAT_GRAYSCALE) {
        int i;
        uint8_t b;
        for(i=0; i<pix_count; i++) {
            b = *src_buf++;
            *rgb_buf++ = b;
            *rgb_buf++ = b;
            *rgb_buf++ = b;
        }
    } else if(format == PIXFORMAT_YUV422) {
        int i, maxi = pix_count / 2;
        uint8_t y0, y1, u, v;
        uint8_t r, g, b;
        for(i=0; i<maxi; i++) {
            y0 = *src_buf++;
            u = *src_buf++;
            y1 = *src_buf++;
            v = *src_buf++;

            yuv2rgb(y0, u, v, &r, &g, &b);
            *rgb_buf++ = b;
            *rgb_buf++ = g;
            *rgb_buf++ = r;

            yuv2rgb(y1, u, v, &r, &g, &b);
            *rgb_buf++ = b;
            *rgb_buf++ = g;
            *rgb_buf++ = r;
        }
    }
    *out = out_buf;
    *out_len = out_size;
    return true;
}

bool frame2bmp(camera_fb_t * fb, uint8_t ** out, size_t * out_len)
{
    return fmt2bmp(fb->buf, fb->len, fb->width, fb->height, fb->format, out, out_len);
}
