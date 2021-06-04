// SPDX-License-Identifier: GPL-2.0
/*
 * GC2145 CMOS Image Sensor driver
 *
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 fix gc2145 exposure issues.
 * V0.0X01.0X04 add enum_frame_interval function.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "gc2145.h"
#include "gc2145_regs.h"
#include "gc2145_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "gc2145";
#endif

#define GC2145_PIXEL_RATE		(120 * 1000 * 1000)

#define GC2145_EXPOSURE_CONTROL
#define EINVAL 1;



//#define REG_DEBUG_ON

static int read_reg(uint8_t slv_addr, const uint16_t reg){
    int ret = SCCB_Read(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value){
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }
    if ((uint8_t)old_value != value) {
        ESP_LOGI(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write(slv_addr, reg, value);//maybe not?
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "WRITE REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask){
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if(ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint16_t (*regs)[2])
{
    int i = 0, ret = 0;
    while (!ret && regs[i][0] != REGLIST_TAIL) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

// static int write_reg16(uint8_t slv_addr, const uint16_t reg, uint16_t value)
// {
//     if (write_reg(slv_addr, reg, value >> 8) || write_reg(slv_addr, reg + 1, value)) {
//         return -1;
//     }
//     return 0;
// }

// static int write_addr_reg(uint8_t slv_addr, const uint16_t reg, uint16_t x_value, uint16_t y_value)
// {
//     if (write_reg16(slv_addr, reg, x_value) || write_reg16(slv_addr, reg + 2, y_value)) {
//         return -1;
//     }
//     return 0;
// }


static int reset(sensor_t *sensor)
{
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, RESET_RELATED, 0xe0);
    if(ret){
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
	// ret = write_regs(sensor->slv_addr, gc2145_dvp_init_regs);
	ret = write_regs(sensor->slv_addr, g_gc2145_init_reg_table);
	if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    const uint16_t (*regs)[2];

	ESP_LOGI(TAG, "set_pixformat");
	// write_regs(sensor->slv_addr, gc2145_dvp_svga_20fps);
	write_regs(sensor->slv_addr, g_gc2145_qvga_reg_table);

    write_reg(sensor->slv_addr, 0xf2, 0x0f);
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    set_reg_bits(sensor->slv_addr, 0x86, 0, 0x01, 1);  //vsync polarity
    set_reg_bits(sensor->slv_addr, 0x84, 0, 0x1f, 6);  //RGB565
    set_reg_bits(sensor->slv_addr, 0xf7, 1, 0x01, 1); // PLL_mode1:div2en
    set_reg_bits(sensor->slv_addr, 0xf7, 7, 0x01, 1); // PLL_mode1:dvp mode
    set_reg_bits(sensor->slv_addr, 0xf8, 0, 0x3f, 8); //PLL_mode2 :divx4
    set_reg_bits(sensor->slv_addr, 0xfa, 4, 0x0f, 2); //vlk div mode :divide_by

    // write_reg(sensor->slv_addr, 0x05, 0); // buf_CISCTL_capt_hb[11:8]
    // write_reg(sensor->slv_addr, 0x06, 0x5a); // buf_CISCTL_capt_hb[7:0]

    write_reg(sensor->slv_addr, 0x07, 0); // buf_CISCTL_capt_vb[12:8]
    write_reg(sensor->slv_addr, 0x08, 100); // buf_CISCTL_capt_vb[7:0]
    
    
    ESP_LOGI(TAG,"REG list look ======================");
    for (size_t i = 0xf0; i <= 0xfe; i++)
    {
	    ESP_LOGI(TAG, "reg[0x%02x] = 0x%02x", i, read_reg(sensor->slv_addr, i));
    }
    ESP_LOGI(TAG,"\npage 0 ===");
    write_reg(sensor->slv_addr, 0xfe, 0x00); // page 0
    for (size_t i = 0x03; i <= 0x24; i++)
    {
	    ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(sensor->slv_addr, i));
    }
    for (size_t i = 0x80; i <= 0xa2; i++)
    {
	    ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(sensor->slv_addr, i));
    }
    ESP_LOGI(TAG,"\npage 3 ===");
    write_reg(sensor->slv_addr, 0xfe, 0x03); // page 3
    for (size_t i = 0x01; i <= 0x43; i++)
    {
	    ESP_LOGI(TAG, "p3 reg[0x%02x] = 0x%02x", i, read_reg(sensor->slv_addr, i));
    }

    
    // switch (pixformat) {
    // case PIXFORMAT_YUV422:
    //     regs = sensor_fmt_yuv422;
    //     break;

    // case PIXFORMAT_GRAYSCALE:
    //     regs = sensor_fmt_grayscale;
    //     break;

    // case PIXFORMAT_RGB565:
    // case PIXFORMAT_RGB888:
    //     regs = sensor_fmt_rgb565;
    //     break;

    // case PIXFORMAT_JPEG:
    //     regs = sensor_fmt_jpeg;
    //     break;

    // case PIXFORMAT_RAW:
    //     regs = sensor_fmt_raw;
    //     break;

    // default:
    //     ESP_LOGE(TAG, "Unsupported pixformat: %u", pixformat);
    //     return -1;
    // }

    // ret = write_regs(sensor->slv_addr, regs);
    if(ret == 0) {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat to: %u", pixformat);
    }
    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;
	if(framesize > FRAMESIZE_UXGA){
        ESP_LOGW(TAG, "Invalid framesize: %u", framesize);
        framesize = FRAMESIZE_UXGA;
    }
    framesize_t old_framesize = sensor->status.framesize;
    sensor->status.framesize = framesize;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;

    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= write_reg(sensor->slv_addr, 0x95, h>>8); //out_win_height[10:8]
    ret |= write_reg(sensor->slv_addr, 0x96, h&0xff); //out_win_height[7:0]
    ret |= write_reg(sensor->slv_addr, 0x97, w>>8); //out_win_width[10:8]
    ret |= write_reg(sensor->slv_addr, 0x98, w&0xff); //out_win_width[7:0]
 
    if (ret == 0) {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    return ret;

}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, 0x17, 0, 0x01, enable!=0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, 0x17, 1, 0x01, enable!=0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

static int set_quality(sensor_t *sensor, int qs)
{
    int ret = 0;
    // ret = write_reg(sensor->slv_addr, COMPRESSION_CTRL07, qs & 0x3f);
    if (ret == 0) {
        sensor->status.quality = qs;
        ESP_LOGD(TAG, "Set quality to: %d", qs);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    // ret = write_reg_bits(sensor->slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable);
    if (ret == 0) {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
    }
    return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// struct gc2145_framesize {
// 	uint16_t width;
// 	uint16_t height;
// 	struct v4l2_fract max_fps;
// 	uint16_t max_exp_lines;
// 	const struct sensor_register *regs;
// };

// struct gc2145_pll_ctrl {
// 	uint8_t ctrl1;
// 	uint8_t ctrl2;
// 	uint8_t ctrl3;
// };

// struct gc2145_pixfmt {
// 	uint32_t code;
// 	/* Output format Register Value (REG_FORMAT_CTRL00) */
// 	struct sensor_register *format_ctrl_regs;
// };

// struct pll_ctrl_reg {
// 	unsigned int div;
// 	unsigned char reg;
// };

// static const char * const gc2145_supply_names[] = {
// 	"dovdd",	/* Digital I/O power */
// 	"avdd",		/* Analog power */
// 	"dvdd",		/* Digital core power */
// };

// #define GC2145_NUM_SUPPLIES ARRAY_SIZE(gc2145_supply_names)

// struct gc2145 {
// 	struct v4l2_subdev sd;
// 	struct media_pad pad;
// 	struct v4l2_mbus_framefmt format;
// 	unsigned int fps;
// 	unsigned int xvclk_frequency;
// 	struct clk *xvclk;
// 	struct gpio_desc *power_gpio;
// 	struct gpio_desc *pwdn_gpio;
// 	struct gpio_desc *reset_gpio;
// 	struct regulator_bulk_data supplies[GC2145_NUM_SUPPLIES];
// 	struct mutex lock;
// 	struct i2c_client *client;
// 	struct v4l2_ctrl_handler ctrls;
// 	struct v4l2_ctrl *link_frequency;
// 	struct v4l2_fwnode_endpoint bus_cfg;
// 	const struct gc2145_framesize *frame_size;
// 	const struct gc2145_framesize *framesize_cfg;
// 	unsigned int cfg_num;
// 	int streaming;
// 	bool			power_on;
// 	uint32_t module_index;
// 	const char *module_facing;
// 	const char *module_name;
// 	const char *len_name;
// };


// static const struct gc2145_framesize gc2145_dvp_framesizes[] = {
// 	{ /* SVGA */
// 		.width		= 800,
// 		.height		= 600,
// 		.max_fps = {
// 			.numerator = 10000,
// 			.denominator = 160000,
// 		},
// 		.regs		= gc2145_dvp_svga_20fps,
// 	}, { /* SVGA */
// 		.width		= 800,
// 		.height		= 600,
// 		.max_fps = {
// 			.numerator = 10000,
// 			.denominator = 300000,
// 		},
// 		.regs		= gc2145_dvp_svga_30fps,
// 	}, { /* FULL */
// 		.width		= 1600,
// 		.height		= 1200,
// 		.max_fps = {
// 			.numerator = 10000,
// 			.denominator = 160000,
// 		},
// 		.regs		= gc2145_dvp_full,
// 	}
// };


// static const s64 link_freq_menu_items[] = {
// 	240000000
// };




// static void gc2145_set_streaming(struct gc2145 *gc2145, int on)
// {
// 	struct i2c_client *client = gc2145->client;
// 	int ret = 0;
// 	u8 val;

// 	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

// 	if (gc2145->bus_cfg.bus_type == V4L2_MBUS_CSI2) {
// 		val = on ? 0x94 : 0x84;
// 		ret = gc2145_write(client, 0xfe, 0x03);
// 		ret |= gc2145_write(client, 0x10, val);
// 	} else {
// 		val = on ? 0x0f : 0;
// 		ret = gc2145_write(client, 0xf2, val);
// 	}
// 	if (ret)
// 		dev_err(&client->dev, "gc2145 soft standby failed\n");
// }



// static void __gc2145_try_frame_size_fps(struct gc2145 *gc2145,
// 					struct v4l2_mbus_framefmt *mf,
// 					const struct gc2145_framesize **size,
// 					unsigned int fps)
// {
// 	const struct gc2145_framesize *fsize = &gc2145->framesize_cfg[0];
// 	const struct gc2145_framesize *match = NULL;
// 	unsigned int i = gc2145->cfg_num;
// 	unsigned int min_err = UINT_MAX;

// 	while (i--) {
// 		unsigned int err = abs(fsize->width - mf->width)
// 				+ abs(fsize->height - mf->height);
// 		if (err < min_err && fsize->regs[0].addr) {
// 			min_err = err;
// 			match = fsize;
// 		}
// 		fsize++;
// 	}

// 	if (!match) {
// 		match = &gc2145->framesize_cfg[0];
// 	} else {
// 		fsize = &gc2145->framesize_cfg[0];
// 		for (i = 0; i < gc2145->cfg_num; i++) {
// 			if (fsize->width == match->width &&
// 			    fsize->height == match->height &&
// 			    fps >= DIV_ROUND_CLOSEST(fsize->max_fps.denominator,
// 				fsize->max_fps.numerator))
// 				match = fsize;

// 			fsize++;
// 		}
// 	}

// 	mf->width  = match->width;
// 	mf->height = match->height;

// 	if (size)
// 		*size = match;
// }

// #ifdef GC2145_EXPOSURE_CONTROL
// /*
//  * the function is called before sensor register setting in VIDIOC_S_FMT
//  */
// /* Row times = Hb + Sh_delay + win_width + 4*/

// static int gc2145_aec_ctrl(struct v4l2_subdev *sd,
// 			  struct v4l2_mbus_framefmt *mf)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	int ret = 0;
// 	u8 value;
// 	static unsigned int capture_fps = 10, capture_lines = 1266;
// 	static unsigned int preview_fps = 20, preview_lines = 1266;
// 	static unsigned int lines_10ms = 1;
// 	static unsigned int shutter_h = 0x04, shutter_l = 0xe2;
// 	static unsigned int cap = 0, shutter = 0x04e2;

// 	dev_info(&client->dev, "%s enter\n", __func__);
// 	if ((mf->width == 800 && mf->height == 600) && cap == 1) {
// 		cap = 0;
// 		ret = gc2145_write(client, 0xfe, 0x00);
// 		ret |= gc2145_write(client, 0xb6, 0x00);
// 		ret |= gc2145_write(client, 0x03, shutter_h);
// 		ret |= gc2145_write(client, 0x04, shutter_l);
// 		ret |= gc2145_write(client, 0x82, 0xfa);
// 		ret |= gc2145_write(client, 0xb6, 0x01);
// 		if (ret)
// 			dev_err(&client->dev, "gc2145 reconfig failed!\n");
// 	}
// 	if (mf->width == 1600 && mf->height == 1200) {
// 		cap = 1;
// 		ret = gc2145_write(client, 0xfe, 0x00);
// 		ret |= gc2145_write(client, 0xb6, 0x00);
// 		ret |= gc2145_write(client, 0x82, 0xf8);

// 		/*shutter calculate*/
// 		ret |= gc2145_read(client, 0x03, &value);
// 		shutter_h = value;
// 		shutter = (value << 8);
// 		ret |= gc2145_read(client, 0x04, &value);
// 		shutter_l = value;
// 		shutter |= (value & 0xff);
// 		dev_info(&client->dev, "%s(%d) 800x600 shutter read(0x%04x)!\n",
// 					__func__, __LINE__, shutter);
// 		shutter = shutter * capture_lines / preview_lines;
// 		shutter = shutter * capture_fps / preview_fps;
// 		lines_10ms = capture_fps * capture_lines / 100;
// 		if (shutter > lines_10ms) {
// 			shutter = shutter + lines_10ms / 2;
// 			shutter /= lines_10ms;
// 			shutter *= lines_10ms;
// 		}
// 		if (shutter < 1)
// 			shutter = 0x276;
// 		dev_info(&client->dev, "%s(%d)lines_10ms(%d),cal(0x%08x)!\n",
// 			  __func__, __LINE__, lines_10ms, shutter);

// 		ret |= gc2145_write(client, 0x03, ((shutter >> 8) & 0x1f));
// 		ret |= gc2145_write(client, 0x04, (shutter & 0xff));
// 		if (ret)
// 			dev_err(&client->dev, "full exp reconfig failed!\n");
// 	}
// 	return ret;
// }
// #endif

// static int gc2145_set_fmt(struct v4l2_subdev *sd,
// 			  struct v4l2_subdev_pad_config *cfg,
// 			  struct v4l2_subdev_format *fmt)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	int index = ARRAY_SIZE(gc2145_formats);
// 	struct v4l2_mbus_framefmt *mf = &fmt->format;
// 	const struct gc2145_framesize *size = NULL;
// 	struct gc2145 *gc2145 = to_gc2145(sd);

// 	dev_info(&client->dev, "%s enter\n", __func__);

// 	__gc2145_try_frame_size_fps(gc2145, mf, &size, gc2145->fps);

// 	while (--index >= 0)
// 		if (gc2145_formats[index].code == mf->code)
// 			break;

// 	if (index < 0)
// 		return -EINVAL;

// 	mf->colorspace = V4L2_COLORSPACE_SRGB;
// 	mf->code = gc2145_formats[index].code;
// 	mf->field = V4L2_FIELD_NONE;

// 	mutex_lock(&gc2145->lock);

// 	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
// #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
// 		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
// 		*mf = fmt->format;
// #else
// 		return -ENOTTY;
// #endif
// 	} else {
// 		if (gc2145->streaming) {
// 			mutex_unlock(&gc2145->lock);
// 			return -EBUSY;
// 		}

// 		gc2145->frame_size = size;
// 		gc2145->format = fmt->format;

// 	}

// #ifdef GC2145_EXPOSURE_CONTROL
// 	if (gc2145->power_on)
// 		gc2145_aec_ctrl(sd, mf);
// #endif
// 	mutex_unlock(&gc2145->lock);
// 	return 0;
// }

// static int gc2145_s_stream(struct v4l2_subdev *sd, int on)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	struct gc2145 *gc2145 = to_gc2145(sd);
// 	int ret = 0;

// 	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
// 				gc2145->frame_size->width,
// 				gc2145->frame_size->height,
// 		DIV_ROUND_CLOSEST(gc2145->frame_size->max_fps.denominator,
// 				  gc2145->frame_size->max_fps.numerator));

// 	mutex_lock(&gc2145->lock);

// 	on = !!on;

// 	if (gc2145->streaming == on)
// 		goto unlock;

// 	if (!on) {
// 		/* Stop Streaming Sequence */
// 		gc2145_set_streaming(gc2145, on);
// 		gc2145->streaming = on;
// 		goto unlock;
// 	}
// 	if (ret)
// 		dev_err(&client->dev, "init error\n");

// 	ret = gc2145_write_array(client, gc2145->frame_size->regs);
// 	if (ret)
// 		goto unlock;

// 	gc2145_set_streaming(gc2145, on);
// 	gc2145->streaming = on;

// unlock:
// 	mutex_unlock(&gc2145->lock);
// 	return ret;
// }

// static int gc2145_set_test_pattern(struct gc2145 *gc2145, int value)
// {
// 	return 0;
// }

// static int gc2145_s_ctrl(struct v4l2_ctrl *ctrl)
// {
// 	struct gc2145 *gc2145 =
// 			container_of(ctrl->handler, struct gc2145, ctrls);

// 	switch (ctrl->id) {
// 	case V4L2_CID_TEST_PATTERN:
// 		return gc2145_set_test_pattern(gc2145, ctrl->val);
// 	}

// 	return 0;
// }

// static int gc2145_s_frame_interval(struct v4l2_subdev *sd,
// 				   struct v4l2_subdev_frame_interval *fi)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	struct gc2145 *gc2145 = to_gc2145(sd);
// 	const struct gc2145_framesize *size = NULL;
// 	struct v4l2_mbus_framefmt mf;
// 	unsigned int fps;
// 	int ret = 0;

// 	dev_dbg(&client->dev, "Setting %d/%d frame interval\n",
// 		fi->interval.numerator, fi->interval.denominator);

// 	mutex_lock(&gc2145->lock);

// 	if (gc2145->format.width == 1600)
// 		goto unlock;

// 	fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
// 				fi->interval.numerator);
// 	mf = gc2145->format;
// 	__gc2145_try_frame_size_fps(gc2145, &mf, &size, fps);

// 	if (gc2145->frame_size != size) {
// 		dev_info(&client->dev, "%s match wxh@FPS is %dx%d@%d\n",
// 			__func__, size->width, size->height,
// 			DIV_ROUND_CLOSEST(size->max_fps.denominator,
// 				size->max_fps.numerator));
// 		ret |= gc2145_write_array(client, size->regs);
// 		if (ret)
// 			goto unlock;
// 		gc2145->frame_size = size;
// 		gc2145->fps = fps;
// 	}
// unlock:
// 	mutex_unlock(&gc2145->lock);

// 	return ret;
// }



static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0, ret2 = 0;
    if(mask > 0xFF){
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if(ret > 0){
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0, ret2 = 0;
    if(mask > 0xFF){
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if(ret < 0){
        return ret;
    }
    value = (ret & ~mask) | (value & mask);

    if(mask > 0xFF){

    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int init_status(sensor_t *sensor)
{
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    // sensor->status.sharpness = (read_reg(sensor->slv_addr, 0x5303) / 8) - 3;
    // sensor->status.denoise = get_denoise(sensor);
    // sensor->status.ae_level = 0;
    // sensor->status.gainceiling = read_reg16(sensor->slv_addr, 0x3A18) & 0x3FF;
    // sensor->status.awb = check_reg_mask(sensor->slv_addr, ISP_CONTROL_01, 0x01);
    // sensor->status.dcw = !check_reg_mask(sensor->slv_addr, 0x5183, 0x80);
    // sensor->status.agc = !check_reg_mask(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN);
    // sensor->status.aec = !check_reg_mask(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN);
    // sensor->status.hmirror = check_reg_mask(sensor->slv_addr, TIMING_TC_REG21, TIMING_TC_REG21_HMIRROR);
    // sensor->status.vflip = check_reg_mask(sensor->slv_addr, TIMING_TC_REG20, TIMING_TC_REG20_VFLIP);
    // sensor->status.colorbar = check_reg_mask(sensor->slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR);
    // sensor->status.bpc = check_reg_mask(sensor->slv_addr, 0x5000, 0x04);
    // sensor->status.wpc = check_reg_mask(sensor->slv_addr, 0x5000, 0x02);
    // sensor->status.raw_gma = check_reg_mask(sensor->slv_addr, 0x5000, 0x20);
    // sensor->status.lenc = check_reg_mask(sensor->slv_addr, 0x5000, 0x80);
    // sensor->status.quality = read_reg(sensor->slv_addr, COMPRESSION_CTRL07) & 0x3f;
    // sensor->status.special_effect = 0;
    // sensor->status.wb_mode = 0;
    // sensor->status.awb_gain = check_reg_mask(sensor->slv_addr, 0x3406, 0x01);
    // sensor->status.agc_gain = get_agc_gain(sensor);
    // sensor->status.aec_value = get_aec_value(sensor);
    // sensor->status.aec2 = check_reg_mask(sensor->slv_addr, 0x3a00, 0x04);
    return 0;
}


int gc2145_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    // sensor->set_contrast  = set_contrast;
    // sensor->set_brightness= set_brightness;
    // sensor->set_saturation= set_saturation;

    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;

    // sensor->set_gainceiling = set_gainceiling_sensor;
    // sensor->set_gain_ctrl = set_agc_sensor;
    // sensor->set_exposure_ctrl = set_aec_sensor;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    // sensor->set_whitebal = set_awb_dsp;
    // sensor->set_aec2 = set_aec2;
    // sensor->set_aec_value = set_aec_value;
    // sensor->set_special_effect = set_special_effect;
    // sensor->set_wb_mode = set_wb_mode;
    // sensor->set_ae_level = set_ae_level;

    // sensor->set_dcw = set_dcw_dsp;
    // sensor->set_bpc = set_bpc_dsp;
    // sensor->set_wpc = set_wpc_dsp;
    // sensor->set_awb_gain = set_awb_gain_dsp;
    // sensor->set_agc_gain = set_agc_gain;

    // sensor->set_raw_gma = set_raw_gma_dsp;
    // sensor->set_lenc = set_lenc_dsp;

    // //not supported
    // sensor->set_sharpness = set_sharpness;
    // sensor->set_denoise = set_denoise;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    // sensor->set_res_raw = set_res_raw;
    // sensor->set_pll = _set_pll;
    // sensor->set_xclk = set_xclk;

	

    ESP_LOGD(TAG, "GC2145 Attached");
    return 0;
}
