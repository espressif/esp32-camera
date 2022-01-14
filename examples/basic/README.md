| Supported Targets | ESP32 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- |

# Camera Basic Example

This example demonstrates how to initialize the camera sensor and then obtain the data of the sensor.

## How to use example

### Hardware Required

* A development board with camera module (e.g., ESP-EYE, ESP32-S2-Kaluga-1, ESP32-S3-EYE, etc.)
* A USB cable for power supply and programming

### Configure the project

```
idf.py menuconfig
```
* Open the project configuration menu (`idf.py menuconfig -> Camera Pin Configuration`) to configure camera pins.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


## Example Output
```
I (325) camera: Detected OV2640 camera
I (325) camera: Camera PID=0x26 VER=0x42 MIDL=0x7f MIDH=0xa2
I (504) s2 ll_cam: node_size: 3840, nodes_per_line: 1, lines_per_node: 6
I (504) s2 ll_cam: dma_half_buffer_min:  3840, dma_half_buffer: 15360, lines_per_half_buffer: 24, dma_buffer_size: 30720
I (512) cam_hal: buffer_size: 30720, half_buffer_size: 15360, node_buffer_size: 3840, node_cnt: 8, total_cnt: 10
I (522) cam_hal: Allocating 153600 Byte frame buffer in PSRAM
I (529) cam_hal: Allocating 153600 Byte frame buffer in PSRAM
I (535) cam_hal: cam config ok
I (539) ov2640: Set PLL: clk_2x: 1, clk_div: 3, pclk_auto: 1, pclk_div: 8
I (714) example:take_picture: Taking picture...
I (855) example:take_picture: Picture taken! Its size was: 153600 bytes
```

## Troubleshooting
* If the log shows "gpio: gpio_intr_disable(176): GPIO number error", then you probably need to check the configuration of camera pins, which you can find in the project configuration menu (`idf.py menuconfig`): Component config -> Camera Pin Config.
