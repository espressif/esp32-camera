if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s2" OR IDF_TARGET STREQUAL "esp32s3")
  set(COMPONENT_SRCS
    driver/esp_camera.c
    driver/cam_hal.c
    driver/sccb.c
    driver/sensor.c
    sensors/ov2640.c
    sensors/ov3660.c
    sensors/ov5640.c
    sensors/ov7725.c
    sensors/ov7670.c
    sensors/nt99141.c
    conversions/yuv.c
    conversions/to_jpg.cpp
    conversions/to_bmp.c
    conversions/jpge.cpp
    conversions/esp_jpg_decode.c

    conversions/libjpeg.c
    conversions/jpeg-9a/jaricom.c
    conversions/jpeg-9a/jcomapi.c
    conversions/jpeg-9a/jutils.c
    conversions/jpeg-9a/jerror.c
    conversions/jpeg-9a/jmemmgr.c
    conversions/jpeg-9a/jdapimin.c
    conversions/jpeg-9a/jdapistd.c
    conversions/jpeg-9a/jdarith.c
    conversions/jpeg-9a/jdmaster.c
    conversions/jpeg-9a/jdinput.c
    conversions/jpeg-9a/jdmarker.c
    conversions/jpeg-9a/jdhuff.c
    conversions/jpeg-9a/jdmainct.c
    conversions/jpeg-9a/jdcoefct.c
    conversions/jpeg-9a/jddctmgr.c
    conversions/jpeg-9a/jdpostct.c
    conversions/jpeg-9a/jdsample.c
    conversions/jpeg-9a/jdcolor.c
    conversions/jpeg-9a/jquant2.c
    conversions/jpeg-9a/jquant1.c
    conversions/jpeg-9a/jdmerge.c
    conversions/jpeg-9a/jmemnobs.c
    conversions/jpeg-9a/jidctint.c
    conversions/jpeg-9a/jidctflt.c
    conversions/jpeg-9a/jidctfst.c
    conversions/jpeg-9a/jcparam.c
    conversions/jpeg-9a/jcapistd.c
    conversions/jpeg-9a/jcapimin.c
    conversions/jpeg-9a/jcinit.c
    conversions/jpeg-9a/jcmaster.c
    conversions/jpeg-9a/jccolor.c
    conversions/jpeg-9a/jchuff.c
    conversions/jpeg-9a/jcmarker.c
    conversions/jpeg-9a/jcmainct.c
    conversions/jpeg-9a/jccoefct.c
    conversions/jpeg-9a/jcarith.c
    conversions/jpeg-9a/jcdctmgr.c
    conversions/jpeg-9a/jcprepct.c
    conversions/jpeg-9a/jcsample.c
    conversions/jpeg-9a/jfdctint.c
    conversions/jpeg-9a/jfdctflt.c
    conversions/jpeg-9a/jfdctfst.c
    conversions/jpeg-9a/jdatasrc.c
    conversions/jpeg-9a/jdatadst.c
    )

  set(COMPONENT_ADD_INCLUDEDIRS
    driver/include
    conversions/include
    )

  set(COMPONENT_PRIV_INCLUDEDIRS
    driver/private_include
    sensors/private_include
    conversions/private_include
    target/private_include
    )

  if(IDF_TARGET STREQUAL "esp32")
    list(APPEND COMPONENT_SRCS
      target/xclk.c
      target/esp32/ll_cam.c
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND COMPONENT_SRCS
      target/xclk.c
      target/esp32s2/ll_cam.c
      target/esp32s2/tjpgd.c
      )

    list(APPEND COMPONENT_PRIV_INCLUDEDIRS
      target/esp32s2/private_include
      )
  endif()

  if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND COMPONENT_SRCS
      target/esp32s3/ll_cam.c
      )
  endif()

  set(COMPONENT_REQUIRES driver)
  set(COMPONENT_PRIV_REQUIRES freertos nvs_flash)

  register_component()
endif()
