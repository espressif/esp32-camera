// Copyright 2020-2022 Espressif Systems (Shanghai) PTE LTD
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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_http_server.h"
#include "img_converters.h"
#include "sdkconfig.h"
#include "esp_log.h"

static httpd_handle_t pic_httpd = NULL;

static const char *TAG = "pic_s";

/* Handler to download a file kept on the server */
static esp_err_t pic_get_handler(httpd_req_t *req)
{
    camera_fb_t *frame = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_camera_fb_return(esp_camera_fb_get());
    frame = esp_camera_fb_get();

    if (frame) {
        if (frame->format == PIXFORMAT_JPEG) {
            _jpg_buf = frame->buf;
            _jpg_buf_len = frame->len;
        } else if (!frame2jpg(frame, 60, &_jpg_buf, &_jpg_buf_len)) {
            ESP_LOGE(TAG, "JPEG compression failed");
            res = ESP_FAIL;
        }
    } else {
        res = ESP_FAIL;
    }

    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (frame->format != PIXFORMAT_JPEG) {
        free(_jpg_buf);
        _jpg_buf = NULL;
    }

    esp_camera_fb_return(frame);

    ESP_LOGI(TAG, "pic len %d", _jpg_buf_len);

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "exit pic server");
        return ESP_FAIL;
    }

    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

esp_err_t start_pic_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 5120;

    httpd_uri_t pic_uri = {
        .uri = "/pic",
        .method = HTTP_GET,
        .handler = pic_get_handler,
        .user_ctx = NULL
    };

    ESP_LOGI(TAG, "Starting pic server on port: '%d'", config.server_port);
    if (httpd_start(&pic_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(pic_httpd, &pic_uri);
        return ESP_OK;
    }
    return ESP_FAIL;
}