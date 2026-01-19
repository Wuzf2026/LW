/**
 * @file rtk_um982_driver.c
 * @author Wuzf
 * @date 2026.02.01
 * @brief Implementation of RTK UM982 driver functions
 */
#include "lawnwomer/rtk_um982_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int rtk_um982_init(void **serial_handle, const char *serial_name, int baudrate)
{
    if (serial_handle == NULL || serial_name == NULL)
    {
        printf("RTK UM982: Invalid input parameters\n");
        return -1;
    }

    UM982SerialDevice dev;
    strcpy(dev.serial_name, serial_name);
    dev.baudrate = baudrate;

    int ret = UM982_Serial_Open(&dev);
    if (ret != 0)
    {
        printf("RTK UM982: Failed to open serial port\n");
        return -1;
    }

    ret = UM982_Config_Output(&dev, UM982_OUTPUT_RTCM3);
    if (ret != 0)
    {
        printf("RTK UM982: Failed to config output\n");
        UM982_Serial_Close(&dev);
        return -1;
    }

    *serial_handle = malloc(sizeof(UM982SerialDevice));
    memcpy(*serial_handle, &dev, sizeof(UM982SerialDevice));
    return 0;
}

int rtk_um982_read_data(void *serial_handle, RTKData *rtk_data)
{
    if (serial_handle == NULL || rtk_data == NULL)
    {
        printf("RTK UM982: Invalid input parameters\n");
        return -1;
    }

    UM982SerialDevice *dev = (UM982SerialDevice *)serial_handle;
    UM982PositionData pos_data;
    int ret = UM982_Get_Position(dev, &pos_data);
    if (ret != 0)
    {
        printf("RTK UM982: Failed to get position data\n");
        return -1;
    }

    rtk_data->latitude = pos_data.lat;
    rtk_data->longitude = pos_data.lon;
    rtk_data->altitude = pos_data.alt;
    rtk_data->heading = pos_data.heading;
    rtk_data->is_valid = (pos_data.status == UM982_STATUS_FIXED);
    return 0;
}

int rtk_um982_deinit(void *serial_handle)
{
    if (serial_handle == NULL)
    {
        return 0;
    }

    UM982SerialDevice *dev = (UM982SerialDevice *)serial_handle;
    UM982_Serial_Close(&dev);
    free(serial_handle);
    return 0;
}