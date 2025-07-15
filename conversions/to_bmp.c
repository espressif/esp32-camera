// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
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
#include <stdlib.h>
#include "img_converters.h"
#include "soc/efuse_reg.h"
#include "esp_heap_caps.h"
#include "yuv.h"
#include "sdkconfig.h"
#include "jpeg_decoder.h"

#include "esp_system.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char* TAG = "to_bmp";
#endif

static const int BMP_HEADER_LEN = 54;
static uint8_t work[3100]; // 3.1kB for JPEG decoder, static for legacy reasons

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

static void *_malloc(size_t size)
{
    // check if SPIRAM is enabled and allocate on SPIRAM if allocatable
#if (CONFIG_SPIRAM_SUPPORT && (CONFIG_SPIRAM_USE_CAPS_ALLOC || CONFIG_SPIRAM_USE_MALLOC))
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
    // try allocating in internal memory
    return malloc(size);
}

static bool jpg2rgb888(const uint8_t *src, size_t src_len, uint8_t * out, esp_jpeg_image_scale_t scale)
{
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)src,
        .indata_size = src_len,
        .outbuf = out,
        .outbuf_size = UINT32_MAX, // @todo: this is very bold assumption, keeping this like this for now, not to break existing code
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = scale,
        .flags.swap_color_bytes = 0,
        .advanced.working_buffer = work,
        .advanced.working_buffer_size = sizeof(work),
    };
    esp_jpeg_image_output_t output_img = {};

    if(esp_jpeg_decode(&jpeg_cfg, &output_img) != ESP_OK){
        return false;
    }
    return true;
}

bool jpg2rgb565(const uint8_t *src, size_t src_len, uint8_t * out, esp_jpeg_image_scale_t scale)
{
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)src,
        .indata_size = src_len,
        .outbuf = out,
        .outbuf_size = UINT32_MAX, // @todo: this is very bold assumption, keeping this like this for now, not to break existing code
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
        .flags.swap_color_bytes = 0,
        .advanced.working_buffer = work,
        .advanced.working_buffer_size = sizeof(work),
    };

    esp_jpeg_image_output_t output_img = {};

    if(esp_jpeg_decode(&jpeg_cfg, &output_img) != ESP_OK){
        return false;
    }
    return true;
}

bool jpg2bmp(const uint8_t *src, size_t src_len, uint8_t ** out, size_t * out_len)
{
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)src,
        .indata_size = src_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags.swap_color_bytes = 0,
        .advanced.working_buffer = work,
        .advanced.working_buffer_size = sizeof(work),
    };

    bool ret = false;
    uint8_t *output = NULL;
    esp_jpeg_image_output_t output_img = {};
    if (esp_jpeg_get_image_info(&jpeg_cfg, &output_img) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get image info");
        goto fail;
    }

    // @todo here we allocate memory and we assume that the user will free it
    // this is not the best way to do it, but we need to keep the API
    // compatible with the previous version
    const size_t output_size = output_img.output_len + BMP_HEADER_LEN;
    output = _malloc(output_size);
    if (!output) {
        ESP_LOGE(TAG, "Failed to allocate output buffer");
        goto fail;
    }

    // Start writing decoded data after the BMP header
    jpeg_cfg.outbuf = output + BMP_HEADER_LEN;
    jpeg_cfg.outbuf_size = output_img.output_len;
    if(esp_jpeg_decode(&jpeg_cfg, &output_img) != ESP_OK){
        ESP_LOGE(TAG, "JPEG decode failed");
        goto fail;
    }

    output[0] = 'B';
    output[1] = 'M';
    bmp_header_t * bitmap  = (bmp_header_t*)&output[2];
    bitmap->reserved = 0;
    bitmap->filesize = output_size;
    bitmap->fileoffset_to_pixelarray = BMP_HEADER_LEN;
    bitmap->dibheadersize = 40;
    bitmap->width  =  output_img.width;
    bitmap->height = -output_img.height; //set negative for top to bottom
    bitmap->planes = 1;
    bitmap->bitsperpixel = 24;
    bitmap->compression = 0;
    bitmap->imagesize = output_img.output_len;
    bitmap->ypixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->xpixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->numcolorspallette = 0;
    bitmap->mostimpcolor = 0;

    *out = output;
    *out_len = output_size;
    ret = true;

fail:
    if (!ret && output) {
        free(output);
    }
    return ret;
}

bool fmt2rgb888(const uint8_t *src_buf, size_t src_len, pixformat_t format, uint8_t * rgb_buf)
{
    int pix_count = 0;
    if(format == PIXFORMAT_JPEG) {
        return jpg2rgb888(src_buf, src_len, rgb_buf, JPEG_IMAGE_SCALE_0);
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
        pix_count = src_len / 2;
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

    // With BMP, 8-bit greyscale requires a palette.
    // For a 640x480 image though, that's a savings
    // over going RGB-24.
    int bpp = (format == PIXFORMAT_GRAYSCALE) ? 1 : 3;
    int palette_size = (format == PIXFORMAT_GRAYSCALE) ? 4 * 256 : 0;
    size_t out_size = (pix_count * bpp) + BMP_HEADER_LEN + palette_size;
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
    bitmap->fileoffset_to_pixelarray = BMP_HEADER_LEN + palette_size;
    bitmap->dibheadersize = 40;
    bitmap->width = width;
    bitmap->height = -height;//set negative for top to bottom
    bitmap->planes = 1;
    bitmap->bitsperpixel = bpp * 8;
    bitmap->compression = 0;
    bitmap->imagesize = pix_count * bpp;
    bitmap->ypixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->xpixelpermeter = 0x0B13 ; //2835 , 72 DPI
    bitmap->numcolorspallette = 0;
    bitmap->mostimpcolor = 0;

    uint8_t * palette_buf = out_buf + BMP_HEADER_LEN;
    uint8_t * pix_buf = palette_buf + palette_size;
    uint8_t * src_buf = src;

    if (palette_size > 0) {
        // Grayscale palette
        for (int i = 0; i < 256; ++i) {
            for (int j = 0; j < 3; ++j) {
                *palette_buf = i;
                palette_buf++;
            }
            // Reserved / alpha channel.
            *palette_buf = 0;
            palette_buf++;
        }
    }

    //convert data to RGB888
    if(format == PIXFORMAT_RGB888) {
        memcpy(pix_buf, src_buf, pix_count*3);
    } else if(format == PIXFORMAT_RGB565) {
        int i;
        uint8_t hb, lb;
        for(i=0; i<pix_count; i++) {
            hb = *src_buf++;
            lb = *src_buf++;
            *pix_buf++ = (lb & 0x1F) << 3;
            *pix_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
            *pix_buf++ = hb & 0xF8;
        }
    } else if(format == PIXFORMAT_GRAYSCALE) {
        memcpy(pix_buf, src_buf, pix_count);
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
            *pix_buf++ = b;
            *pix_buf++ = g;
            *pix_buf++ = r;

            yuv2rgb(y1, u, v, &r, &g, &b);
            *pix_buf++ = b;
            *pix_buf++ = g;
            *pix_buf++ = r;
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
