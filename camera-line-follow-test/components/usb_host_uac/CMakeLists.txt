idf_component_register(SRCS "uac_host.c" "uac_descriptors.c" INCLUDE_DIRS "include" REQUIRES usb PRIV_REQUIRES esp_ringbuf)
target_compile_definitions(${COMPONENT_LIB} PRIVATE USB_HOST_UAC_VER_MAJOR=1 USB_HOST_UAC_VER_MINOR=5 USB_HOST_UAC_VER_PATCH=0)
