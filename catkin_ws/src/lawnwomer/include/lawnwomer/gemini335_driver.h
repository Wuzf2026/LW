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

#define GEMINI335_USB_VENDOR_ID 0x2bc5
#define GEMINI335_USB_PRODUCT_ID 0x0401

/**
 * @brief Initialize Gemini335 camera
 * @param device_handle [out] Pointer to device handle
 * @return int 0:success, -1:failure
 */
int gemini335_init(void **device_handle);

/**
 * @brief Read depth data from camera
 * @param device_handle [in] Device handle
 * @param depth_data [out] Pointer to depth data array
 * @param data_len [out] Length of depth data
 * @return int 0:success, -1:failure
 */
int gemini335_read_depth(void *device_handle, uint16_t *depth_data, int *data_len);

/**
 * @brief Deinitialize camera
 * @param device_handle [in] Device handle
 * @return int 0:success, -1:failure
 */
int gemini335_deinit(void *device_handle);

#endif