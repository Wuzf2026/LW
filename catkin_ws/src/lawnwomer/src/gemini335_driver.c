/**
 * @file gemini335_driver.c
 * @author Wuzf
 * @date 2026.01.22
 * @brief Implementation of Orbbec Gemini335 depth camera driver
 */
#include "lawnwomer/gemini335_driver.h"
#include <stdio.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

int gemini335_init(void **device_handle) {
    if (device_handle == NULL) {
        printf("Gemini335: Invalid device handle pointer\n");
        return -1;
    }

    // Initialize libusb
    int ret = libusb_init(NULL);
    if (ret != 0) {
        printf("Gemini335: Libusb init failed (code: %d)\n", ret);
        return -2;
    }

    // Find USB device
    libusb_device_handle *usb_handle = libusb_open_device_with_vid_pid(NULL, 
                                                                       GEMINI335_USB_VENDOR_ID, 
                                                                       GEMINI335_USB_PRODUCT_ID);
    if (usb_handle == NULL) {
        printf("Gemini335: USB device not found\n");
        libusb_exit(NULL);
        return -2;
    }

    // Initialize SDK
    Gemini335Handle *handle = (Gemini335Handle *)malloc(sizeof(Gemini335Handle));
    ret = Gemini335_Init(handle, usb_handle);
    if (ret != 0) {
        printf("Gemini335: SDK init failed (code: %d)\n", ret);
        libusb_close(usb_handle);
        libusb_exit(NULL);
        free(handle);
        return -3;
    }

    // Start depth stream
    ret = Gemini335_StartDepthStream(handle);
    if (ret != 0) {
        printf("Gemini335: Start depth stream failed (code: %d)\n", ret);
        Gemini335_Deinit(handle);
        libusb_close(usb_handle);
        libusb_exit(NULL);
        free(handle);
        return -3;
    }

    *device_handle = (void *)handle;
    return 0;
}

int gemini335_read_depth(void *device_handle, uint16_t *depth_data, int *data_len) {
    if (device_handle == NULL || depth_data == NULL || data_len == NULL) {
        printf("Gemini335: Invalid input parameters\n");
        return -1;
    }

    Gemini335Handle *handle = (Gemini335Handle *)device_handle;
    uint16_t raw_data[GEMINI335_DEPTH_SIZE];
    int ret = Gemini335_ReadDepthFrame(handle, raw_data, GEMINI335_DEPTH_SIZE);
    if (ret != 0) {
        printf("Gemini335: Read depth data failed (code: %d)\n", ret);
        return -2;
    }

    // Copy data to output
    memcpy(depth_data, raw_data, GEMINI335_DEPTH_SIZE * sizeof(uint16_t));
    *data_len = GEMINI335_DEPTH_SIZE;
    return 0;
}

int gemini335_deinit(void *device_handle) {
    if (device_handle == NULL) {
        printf("Gemini335: Invalid device handle\n");
        return -1;
    }

    Gemini335Handle *handle = (Gemini335Handle *)device_handle;
    
    // Stop depth stream
    Gemini335_StopDepthStream(handle);
    
    // Deinit SDK
    Gemini335_Deinit(handle);
    
    // Close USB handle
    libusb_device_handle *usb_handle = handle->usb_handle;
    libusb_close(usb_handle);
    libusb_exit(NULL);
    
    // Free memory
    free(handle);
    return 0;
}

int gemini335_get_device_info(void *device_handle, Gemini335DeviceInfo *info) {
    if (device_handle == NULL || info == NULL) {
        printf("Gemini335: Invalid input parameters for get device info\n");
        return -1;
    }

    Gemini335Handle *handle = (Gemini335Handle *)device_handle;
    int ret = Gemini335_GetFirmwareVersion(handle, info->firmware_version, 32);
    if (ret != 0) {
        printf("Gemini335: Get firmware version failed (code: %d)\n", ret);
        return -1;
    }

    info->vendor_id = GEMINI335_USB_VENDOR_ID;
    info->product_id = GEMINI335_USB_PRODUCT_ID;
    info->exposure = handle->exposure;

    return 0;
}

int gemini335_set_exposure(void *device_handle, int exposure) {
    if (device_handle == NULL || exposure < 1 || exposure > 100) {
        printf("Gemini335: Invalid exposure value (1-100 ms required)\n");
        return -1;
    }

    Gemini335Handle *handle = (Gemini335Handle *)device_handle;
    int ret = Gemini335_SetExposure(handle, exposure);
    if (ret != 0) {
        printf("Gemini335: Set exposure failed (code: %d)\n", ret);
        return -2;
    }

    handle->exposure = exposure;
    return 0;
}