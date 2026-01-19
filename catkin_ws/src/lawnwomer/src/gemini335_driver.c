/**
 * @file gemini335_driver.c
 * @author Wuzf
 * @date 2026.01.18
 * @brief Implementation of Gemini335 driver functions
 */
#include "lawnwomer/gemini335_driver.h"
#include <stdio.h>
#include <stdlib.h>

int gemini335_init(void **device_handle)
{
    if (device_handle == NULL)
    {
        printf("Gemini335: Invalid device handle pointer\n");
        return -1;
    }

    Gemini335Device dev;
    int ret = Gemini335_OpenDevice(&dev, GEMINI335_USB_VENDOR_ID, GEMINI335_USB_PRODUCT_ID);
    if (ret != 0)
    {
        printf("Gemini335: Failed to open device\n");
        return -1;
    }

    ret = Gemini335_StartStream(&dev);
    if (ret != 0)
    {
        printf("Gemini335: Failed to start stream\n");
        Gemini335_CloseDevice(&dev);
        return -1;
    }

    *device_handle = malloc(sizeof(Gemini335Device));
    memcpy(*device_handle, &dev, sizeof(Gemini335Device));
    return 0;
}

int gemini335_read_depth(void *device_handle, uint16_t *depth_data, int *data_len)
{
    if (device_handle == NULL || depth_data == NULL || data_len == NULL)
    {
        printf("Gemini335: Invalid input parameters\n");
        return -1;
    }

    Gemini335Device *dev = (Gemini335Device *)device_handle;
    Gemini335Frame frame;
    int ret = Gemini335_GetFrame(dev, &frame);
    if (ret != 0)
    {
        printf("Gemini335: Failed to get frame\n");
        return -1;
    }

    *data_len = frame.width * frame.height;
    memcpy(depth_data, frame.depth_data, *data_len * sizeof(uint16_t));
    return 0;
}

int gemini335_deinit(void *device_handle)
{
    if (device_handle == NULL)
    {
        return 0;
    }

    Gemini335Device *dev = (Gemini335Device *)device_handle;
    Gemini335_StopStream(dev);
    Gemini335_CloseDevice(dev);
    free(device_handle);
    return 0;
}