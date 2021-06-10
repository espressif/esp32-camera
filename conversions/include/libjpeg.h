// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _IMG_LIBJPEG_H_
#define _IMG_LIBJPEG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	COLOR_TYPE_UNKNOWN,
    COLOR_TYPE_RGB565,
	COLOR_TYPE_RGB888,
	COLOR_TYPE_YUV422,
} color_space_t;


/**
 * @brief JPEG compression
 * 
 * @param in_buffer pointer to image data
 * The standard input image format is a rectangular array of pixels, 
 * for example, R,G,B,R,G,B,R,G,B,... for 24-bit RGB color.
 * 
 * @attention If the encoded data size exceeds the outsize, a larger buffer will be reallocated internally.
 * and the `outsize` will not be released, because the internal does not know whether the `outsize` is allocated. 
 * Therefore, after calling this function, you should check whether the paramter `outbuffer` is the same as before calling this function.
 * If it is different, it means that it has been reallocated internally. At this point, YOU NEED to FREE the new buffer
 * 
 * @param src_type color space of source
 * @param image_width width of image
 * @param image_height height of image
 * @param quality The input 'quality' factor should be 0 (terrible) to 100 (very good).
 * @param outbuffer pointer to output
 * @param outsize size of outbuffer
 * @return 1 on success, 0 on error.
 */
_Bool libjpeg_encode(const uint8_t *in_buffer, color_space_t src_type, int image_width, int image_height, int quality, uint8_t **outbuffer, uint32_t *outsize);

/**
 * @brief JPEG decompression
 * 
 * @param jpeg_data pointer to jpeg image data
 * @param jpeg_size jpeg image size
 * @param dst_type Destination color space
 * @param outbuffer pointer to output, The buffer must be large enough to store all RGB data corresponding to rgb565/rgb888
 * @param width width of image, if you don't care, set to NULL
 * @param height height of image, if you don't care, set to NULL
 * @return 1 on success, 0 on error. 
 */
_Bool libjpeg_decode(const uint8_t *jpeg_data, uint32_t jpeg_size, color_space_t dst_type, uint8_t *outbuffer, uint32_t *width, uint32_t *height);

#ifdef __cplusplus
}
#endif

#endif /* _IMG_LIBJPEG_H_ */
