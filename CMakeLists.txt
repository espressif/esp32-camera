# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

# set conversion sources
set(srcs
  conversions/yuv.c
  conversions/to_jpg.cpp
  conversions/to_bmp.c
  conversions/jpge.cpp
  )

set(priv_include_dirs
  conversions/private_include
  )

set(include_dirs
  driver/include
  conversions/include
  )

set(COMPONENT_REQUIRES driver)

# set driver sources only for supported platforms
if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s2" OR IDF_TARGET STREQUAL "esp32s3")
  list(APPEND srcs
    driver/esp_camera.c
    driver/cam_hal.c
    driver/sensor.c
    sensors/ov2640.c
    sensors/ov3660.c
    sensors/ov5640.c
    sensors/ov7725.c
    sensors/ov7670.c
    sensors/nt99141.c
    sensors/gc0308.c
    sensors/gc2145.c
    sensors/gc032a.c
    sensors/bf3005.c
    sensors/bf20a6.c
    sensors/sc101iot.c
    sensors/sc030iot.c
    sensors/sc031gs.c
    sensors/mega_ccm.c
    sensors/hm1055.c
    sensors/hm0360.c
    )

  list(APPEND priv_include_dirs
    driver/private_include
    sensors/private_include
    target/private_include
    )

  if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs
      target/xclk.c
      target/esp32/ll_cam.c
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND srcs
      target/xclk.c
      target/esp32s2/ll_cam.c
      )

    list(APPEND priv_include_dirs
      target/esp32s2/private_include
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND srcs
      target/esp32s3/ll_cam.c
      )
  endif()

  set(priv_requires freertos nvs_flash esp_mm)

  set(min_version_for_esp_timer "4.2")
  if (idf_version VERSION_GREATER_EQUAL min_version_for_esp_timer)
    list(APPEND priv_requires esp_timer)
  endif()

  # include the SCCB I2C driver
  # this uses either the legacy I2C API or the newer version from IDF v5.4
  # as this features a method to obtain the I2C driver from a port number
  if (idf_version VERSION_GREATER_EQUAL "5.4" AND NOT CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY)
    list(APPEND srcs driver/sccb-ng.c)
  else()
    list(APPEND srcs driver/sccb.c)
  endif()

  if (idf_version VERSION_GREATER_EQUAL "6.0")
    list(APPEND priv_requires esp_driver_gpio)
  endif()

endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS ${include_dirs}
  PRIV_INCLUDE_DIRS ${priv_include_dirs}
  REQUIRES driver  # due to include of driver/gpio.h in esp_camera.h
  PRIV_REQUIRES ${priv_requires}
)
