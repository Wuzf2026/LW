/**
 * @file gemini335_driver.h
 * @author Wuzf
 * @date 2026.01.18
 * @brief Driver for Orbbec Gemini335 depth camera via USB
 * @input USB device handle
 * @output Depth image data (uint16_t array)
 */
#ifndef GEMINI335_DRIVER_H
#define GEMINI335_DRIVER_H

#include "third_party/gemini335_sdk/gemini335_api.h"
#include <stdint.h>
#include <stdlib.h>

#define GEMINI335_USB_VENDOR_ID 0x2bc5
#define GEMINI335_USB_PRODUCT_ID 0x0401
#define GEMINI335_DEPTH_WIDTH 640
#define GEMINI335_DEPTH_HEIGHT 480
#define GEMINI335_DEPTH_SIZE (GEMINI335_DEPTH_WIDTH * GEMINI335_DEPTH_HEIGHT)

/**
 * @brief Device information structure for Gemini335
 */
typedef struct {
    char firmware_version[32];
    uint16_t vendor_id;
    uint16_t product_id;
    int exposure; // Exposure time in ms
} Gemini335DeviceInfo;

/**
 * @brief Initialize Gemini335 camera
 * @param device_handle [out] Pointer to device handle
 * @return int 0:success, -1:failure (invalid param), -2:USB device not found, -3:SDK init failed
 */
int gemini335_init(void **device_handle);

/**
 * @brief Read depth data from camera
 * @param device_handle [in] Device handle
 * @param depth_data [out] Pointer to depth data array (uint16_t, size: GEMINI335_DEPTH_SIZE)
 * @param data_len [out] Length of depth data (should be GEMINI335_DEPTH_SIZE)
 * @return int 0:success, -1:failure (invalid param), -2:read failed
 */
int gemini335_read_depth(void *device_handle, uint16_t *depth_data, int *data_len);

/**
 * @brief Deinitialize camera
 * @param device_handle [in] Device handle
 * @return int 0:success, -1:failure (invalid handle)
 */
int gemini335_deinit(void *device_handle);

/**
 * @brief Get Gemini335 device information
 * @param device_handle [in] Device handle
 * @param info [out] Pointer to device info structure
 * @return int 0:success, -1:failure
 */
int gemini335_get_device_info(void *device_handle, Gemini335DeviceInfo *info);

/**
 * @brief Set exposure time of Gemini335
 * @param device_handle [in] Device handle
 * @param exposure [in] Exposure time in ms (1-100)
 * @return int 0:success, -1:invalid param, -2:set failed
 */
int gemini335_set_exposure(void *device_handle, int exposure);

#endif // GEMINI335_DRIVER_H
