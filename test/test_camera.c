
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "esp_log.h"

#include "esp_camera.h"

#define BOARD_ESP32CAM_AITHINKER 0
#define BOARD_WROVER_KIT 1

// WROVER-KIT PIN Map
#if BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

// ESP32Cam (AiThinker) PIN Map
#if BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

static const char *TAG = "test camera";

typedef void (*decode_func_t)(uint8_t *jpegbuffer, uint32_t size, uint8_t *outbuffer);

static esp_err_t init_camera(uint32_t xclk_freq_hz, pixformat_t pixel_format, uint8_t fb_count)
{
    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sscb_sda = CAM_PIN_SIOD,
        .pin_sscb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = xclk_freq_hz,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = pixel_format, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_UXGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = 12, //0-63 lower number means higher quality
        .fb_count = fb_count,       //if more than one, i2s runs in continuous mode. Use only with JPEG
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    //initialize the camera
    return esp_camera_init(&camera_config);
}

static float camera_test_fps(uint16_t times, uint32_t *size)
{
    uint32_t s = 0;
    uint32_t num = 0;
    uint64_t total_time = esp_timer_get_time();
    for (size_t i = 0; i < times; i++) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (NULL == pic) {
            ESP_LOGW(TAG, "fb get failed");
            continue;
        } else {
            s += pic->len;
            num++;
        }
        esp_camera_fb_return(pic);
    }
    total_time = esp_timer_get_time() - total_time;
    float fps = 0.0f;
    *size = 0;
    if (num) {
        fps = num * 1000000.0f / total_time ;
        *size = s / num;
    }
    return fps;
}

static void camera_test(framesize_t max_size, uint32_t pic_num)
{
    float fps[max_size];
    uint32_t size[max_size];

    for (size_t i = 0; i < max_size; i++) {
        fps[i] = 0.0f;
        ESP_LOGI(TAG, "Testing %d x %d", resolution[i].width, resolution[i].height);
        sensor_t *sensor = esp_camera_sensor_get();
        int ret = sensor->set_framesize(sensor, i);
        if (0 != ret) {
            ESP_LOGE(TAG, "set_framesize %d error", i);
            continue;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
        fps[i] = camera_test_fps(pic_num, &size[i]);
    }

    printf("FPS Result\n");
    printf("resolution  ,  size ,    fps  \n");
    for (size_t i = 0; i < max_size; i++) {
        printf("%4d x %4d , %5d, %5.2f  \n", resolution[i].width, resolution[i].height, size[i], fps[i]);
    }
}

static camera_sensor_info_t *get_camera_info_from_pid(uint8_t pid)
{
    for (size_t i = 0; i < CAMERA_MODEL_MAX; i++) {
        if (pid == camera_sensor[i].pid) {
            return (camera_sensor_info_t *)&camera_sensor[i];
        }
    }
    return NULL;
}


static void print_rgb565_img(uint8_t *img, int width, int height)
{
    uint16_t *p = (uint16_t *)img;
    const char temp2char[17] = "@MNHQ&#UJ*x7^i;.";
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            uint32_t c = p[j * width + i];
            uint8_t r = c >> 11;
            uint8_t g = (c >> 6) & 0x1f;
            uint8_t b = c & 0x1f;
            c = (r + g + b) / 3;
            c >>= 1;
            printf("%c", temp2char[15 - c]);
        }
        printf("\n");
    }
}

static void print_rgb888_img(uint8_t *img, int width, int height)
{
    uint8_t *p = (uint8_t *)img;
    const char temp2char[17] = "@MNHQ&#UJ*x7^i;.";
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            uint8_t *c = p + 3 * (j * width + i);
            uint8_t r = *c++;
            uint8_t g = *c++;
            uint8_t b = *c;
            uint32_t v = (r + g + b) / 3;
            v >>= 4;
            printf("%c", temp2char[15 - v]);
        }
        printf("\n");
    }
}

static void tjpgd_decode_rgb565(uint8_t *mjpegbuffer, uint32_t size, uint8_t *outbuffer)
{
    jpg2rgb565(mjpegbuffer, size, outbuffer, JPG_SCALE_NONE);
}

static void tjpgd_decode_rgb888(uint8_t *mjpegbuffer, uint32_t size, uint8_t *outbuffer)
{
    fmt2rgb888(mjpegbuffer, size, PIXFORMAT_JPEG, outbuffer);
}

typedef enum {
    DECODE_RGB565,
    DECODE_RGB888,
} decode_type_t;

static const decode_func_t g_decode_func[2][2] = {
    {tjpgd_decode_rgb565,},
    {tjpgd_decode_rgb888,},
};


static float jpg_decode_test(uint8_t decoder_index, decode_type_t type, const uint8_t *jpg, uint32_t length, uint32_t img_w, uint32_t img_h, uint32_t times)
{
    uint8_t *jpg_buf = malloc(length);
    if (NULL == jpg_buf) {
        ESP_LOGE(TAG, "malloc for jpg buffer failed");
        return 0;
    }
    memcpy(jpg_buf, jpg, length);

    uint8_t *rgb_buf = heap_caps_malloc(img_w * img_h * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (NULL == rgb_buf) {
        free(jpg_buf);
        ESP_LOGE(TAG, "malloc for rgb buffer failed");
        return 0;
    }
    decode_func_t decode = g_decode_func[type][decoder_index];
    decode(jpg_buf, length, rgb_buf);
    if (DECODE_RGB565 == type) {
        ESP_LOGI(TAG, "jpeg decode to rgb565");
        print_rgb565_img(rgb_buf, img_w, img_h);
    } else {
        ESP_LOGI(TAG, "jpeg decode to rgb888");
        print_rgb888_img(rgb_buf, img_w, img_h);
    }

    uint64_t t_decode[times];
    for (size_t i = 0; i < times; i++) {
        uint64_t t1 = esp_timer_get_time();
        decode(jpg_buf, length, rgb_buf);
        t_decode[i] = esp_timer_get_time() - t1;
    }

    printf("resolution  ,  t \n");
    uint64_t t_total = 0;
    for (size_t i = 0; i < times; i++) {
        t_total += t_decode[i];
        float t = t_decode[i] / 1000.0f;
        printf("%4d x %4d ,  %5.2f ms \n", img_w, img_h, t);
    }

    float fps = times / (t_total / 1000000.0f);
    printf("Decode FPS Result\n");
    printf("resolution  , fps \n");
    printf("%4d x %4d , %5.2f  \n", img_w, img_h, fps);

    free(jpg_buf);
    heap_caps_free(rgb_buf);
    return fps;
}

static void img_jpeg_decode_test(uint16_t pic_index, uint16_t lib_index)
{
    extern const uint8_t img1_start[] asm("_binary_testimg_jpeg_start");
    extern const uint8_t img1_end[]   asm("_binary_testimg_jpeg_end");
    extern const uint8_t img2_start[] asm("_binary_test_inside_jpeg_start");
    extern const uint8_t img2_end[]   asm("_binary_test_inside_jpeg_end");
    extern const uint8_t img3_start[] asm("_binary_test_outside_jpeg_start");
    extern const uint8_t img3_end[]   asm("_binary_test_outside_jpeg_end");

    struct img_t {
        const uint8_t *buf;
        uint32_t length;
        uint16_t w, h;
    };
    struct img_t imgs[3] = {
        {
            .buf = img1_start,
            .length = img1_end - img1_start,
            .w = 227,
            .h = 149,
        },
        {
            .buf = img2_start,
            .length = img2_end - img2_start,
            .w = 320,
            .h = 240,
        },
        {
            .buf = img3_start,
            .length = img3_end - img3_start,
            .w = 480,
            .h = 320,
        },
    };

    ESP_LOGI(TAG, "pic_index:%d", pic_index);
    ESP_LOGI(TAG, "lib_index:%d", lib_index);
    jpg_decode_test(lib_index, DECODE_RGB565, imgs[pic_index].buf, imgs[pic_index].length, imgs[pic_index].w, imgs[pic_index].h, 16);
}


TEST_CASE("Camera driver init, deinit test", "[camera]")
{
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_JPEG, 2));
    TEST_ESP_OK(esp_camera_deinit());
}

TEST_CASE("Camera driver take picture test", "[camera]")
{
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_JPEG, 2));

    ESP_LOGI(TAG, "Taking picture...");
    camera_fb_t *pic = esp_camera_fb_get();
    if (pic) {
        ESP_LOGI(TAG, "picrute: %d x %d, size: %u", pic->width, pic->height, pic->len);
        esp_camera_fb_return(pic);
    }

    TEST_ESP_OK(esp_camera_deinit());
    TEST_ASSERT_NOT_NULL(pic);
}

TEST_CASE("Camera driver jpeg fps test", "[camera]")
{
    uint64_t t1 = esp_timer_get_time();
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_JPEG, 2));
    uint64_t t2 = esp_timer_get_time();
    ESP_LOGI(TAG, "Camera init time %llu ms", (t2 - t1) / 1000);

    sensor_t *s = esp_camera_sensor_get();
    camera_sensor_info_t *info = get_camera_info_from_pid(s->id.PID);
    framesize_t max_size = info->max_size;
    int pic_num = 16;

    ESP_LOGI(TAG, "max_framesize:%d", max_size);
    ESP_LOGI(TAG, "pic_number:%d", pic_num);

    camera_test(max_size, pic_num);
    esp_camera_deinit();
}

TEST_CASE("Camera driver rgb565 fps test", "[camera]")
{
    uint64_t t1 = esp_timer_get_time();
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_RGB565, 2));
    uint64_t t2 = esp_timer_get_time();
    ESP_LOGI(TAG, "Camera init time %llu ms", (t2 - t1) / 1000);

    sensor_t *s = esp_camera_sensor_get();
    camera_sensor_info_t *info = get_camera_info_from_pid(s->id.PID);
    framesize_t max_size = info->max_size;
    int pic_num = 16;

    ESP_LOGI(TAG, "max_framesize:%d", max_size);
    ESP_LOGI(TAG, "pic_number:%d", pic_num);

    camera_test(max_size, pic_num);
    esp_camera_deinit();
}

TEST_CASE("Conversions image 227x149 jpeg decode test", "[camera]")
{
    img_jpeg_decode_test(0, 0);
}

TEST_CASE("Conversions image 320x240 jpeg decode test", "[camera]")
{
    img_jpeg_decode_test(1, 0);
}

TEST_CASE("Conversions image 480x320 jpeg decode test", "[camera]")
{
    img_jpeg_decode_test(2, 0);
}
