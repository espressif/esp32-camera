// Copyright 2022-2023 Espressif Systems (Shanghai) PTE LTD
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

#pragma once

#include "esp_err.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RGBPALEGREEN 0x7EF5
#define RGBBLACK     0x000
#define RGBWHITE     0xFFFF
#define RGBLTGRAY    0xCCCC

#define YUVPALEGREEN 0x60FF
#define YUVBLACK     0x8000
#define YUVWHITE     0x80FF
#define YUVLTGRAY    0x80C0

/**
 * @brief color option.
 */
typedef enum {
    PALEGREEN,
    BLACK,
    WHITE,
    LTGRAY,
} image_timestamp_color_t;

/**
 * @brief Configuration for image timestamp
 */
typedef struct {
    pixformat_t image_format;/*!< The image format. Only support RGB565/YUV422 format for now*/
    size_t image_width;
    size_t image_hight;
    uint16_t timestamp_left_location;/*!< The timestamp bitmap left location in the final image. Range is [1~image_width] */
    uint16_t timestamp_top_location;/*!< The timestamp bitmap top location in the final image. Range is [1~image_hight]*/
    image_timestamp_color_t txt_color;/*!< The txt color. */
    image_timestamp_color_t bkg_color; /*!< The background color. */
} image_timestamp_config_t;

/**
 * @brief rgb565 image timestamp default setting.
 */
#define RGB565_IMAGE_TMSTAMP_CONFIG_DEFAULT() {.timestamp_left_location = 1, \
                                               .timestamp_top_location = 1, \
                                               .txt_color = BLACK, \
                                               .bkg_color = PALEGREEN, \
                                               .image_format = PIXFORMAT_RGB565}

/**
 * @brief yuv422 image timestamp default setting.
 */
#define YUV422_IMAGE_TMSTAMP_CONFIG_DEFAULT() {.timestamp_left_location = 1, \
                                               .timestamp_top_location = 1, \
                                               .txt_color = BLACK, \
                                               .bkg_color = LTGRAY, \
                                               .image_format = PIXFORMAT_YUV422}

/**
 * @brief   Init image timestamp engine(used to add timestamp to an image in RGB565\YUV422 format).
 * @param config   pointer to image timestamp engine initialized configuration structure; can point to a temporary variable.
 * @return      
 *    - ESP_OK: succeed
 *    - ESP_ERR_NO_MEM: out of memory
 *    - others: refer to error code esp_err.h
 */
esp_err_t esp_image_timestamp_init(image_timestamp_config_t *config);

/**
 * @brief   Get configuration of the image timestamp engine.
 * @param config   image timestamp engine configuration.
 * @return      
 *    - ESP_OK: succeed
 *    - ESP_ERR_INVALID_STATE: is not initialized by esp_image_timestamp_init
 */
esp_err_t esp_image_timestamp_get_config(image_timestamp_config_t *config);

/**
 * @brief   Add a timestamp at the specified coordinates of the image.
 * @param img   pointer to image buffer.
 * @param left  the timestamp bitmap left location in the final image. Range is [1~image_width].
 * @param top   The timestamp bitmap top location in the final image. Range is [1~image_hight].
 * @return
 *    - ESP_OK: succeed.
 *    - ESP_ERR_INVALID_STATE: is not initialized by esp_image_timestamp_init.
 */
esp_err_t esp_set_image_timestamp_with_left_top(uint8_t *img, uint16_t left, uint16_t top);

/**
 * @brief   Add a timestamp to the image, the location of the timestamp in the image is specified by esp_image_timestamp_init.
 * @param img   pointer to image buffer.
 * @return
 *    - ESP_OK: succeed.
 *    - ESP_ERR_INVALID_STATE: is not initialized by esp_image_timestamp_init.
 */
esp_err_t esp_set_image_timestamp(uint8_t *img);

/**
 * @brief   Deinit image timestamp engine. Free all resource allocated in esp_image_timestamp_init.
 * @return
 *    - ESP_OK: succeed.
 */
esp_err_t esp_image_timestamp_deinit(void);

/**
 * @brief   Get the timestamp string used by the previous image.
 * @attention 1. This API should not be called if the image timestamp engine is removed
 * @return      timestamp string.
 */
const char *esp_get_image_timestamp(void);

#ifdef __cplusplus
}
#endif
