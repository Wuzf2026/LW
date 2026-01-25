/**
 * @file rtk_um982_driver.c
 * @author Wuzf
 * @date 2026.01.16
 * @brief Implementation of T-RTK UM982 driver functions
 */
#include "lawnwomer/rtk_um982_driver.h"
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int rtk_um982_init(void **device_handle, int baudrate) {
    if (device_handle == NULL) {
        printf("RTK UM982: Invalid device handle pointer\n");
        return -1;
    }

    // Use default baud rate if not provided
    int serial_baud = (baudrate == 0) ? RTK_UM982_DEFAULT_BAUDRATE : baudrate;

    // Allocate RTK device structure
    RTKUM982Handle *handle = (RTKUM982Handle *)malloc(sizeof(RTKUM982Handle));
    if (handle == NULL) {
        printf("RTK UM982: Memory allocation failed\n");
        return -1;
    }

    // Open serial port (auto-detect USB serial device)
    char serial_path[32];
    int ret = RTKUM982_FindSerialPort(RTK_UM982_USB_VENDOR_ID, RTK_UM982_USB_PRODUCT_ID, serial_path, 32);
    if (ret != 0) {
        printf("RTK UM982: Serial port not found\n");
        free(handle);
        return -2;
    }

    // Open serial port
    handle->serial_fd = open(serial_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (handle->serial_fd < 0) {
        printf("RTK UM982: Open serial port failed (path: %s)\n", serial_path);
        free(handle);
        return -2;
    }

    // Configure serial port
    struct termios options;
    tcgetattr(handle->serial_fd, &options);
    cfsetispeed(&options, serial_baud);
    cfsetospeed(&options, serial_baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(handle->serial_fd, TCSANOW, &options);

    // Initialize SDK
    ret = RTKUM982_Init(handle, handle->serial_fd);
    if (ret != 0) {
        printf("RTK UM982: SDK init failed (code: %d)\n", ret);
        close(handle->serial_fd);
        free(handle);
        return -3;
    }

    // Set default mode (RTK)
    RTKUM982_SetMode(handle, 1);

    *device_handle = (void *)handle;
    return 0;
}

int rtk_um982_read_data(void *device_handle, RTKData *rtk_data) {
    if (device_handle == NULL || rtk_data == NULL) {
        printf("RTK UM982: Invalid input parameters\n");
        return -1;
    }

    RTKUM982Handle *handle = (RTKUM982Handle *)device_handle;
    char raw_data[RTK_UM982_BUFFER_SIZE];
    
    // Read raw NMEA data
    int ret = RTKUM982_ReadSerialData(handle, raw_data, RTK_UM982_BUFFER_SIZE);
    if (ret <= 0) {
        printf("RTK UM982: Read serial data failed (code: %d)\n", ret);
        return -2;
    }

    // Parse NMEA data
    RTKRawData raw_rtk;
    ret = RTKUM982_ParseNMEA(raw_data, ret, &raw_rtk);
    if (ret != 0) {
        printf("RTK UM982: Parse NMEA data failed (code: %d)\n", ret);
        return -3;
    }

    // Fill RTK data structure
    rtk_data->latitude = raw_rtk.latitude;
    rtk_data->longitude = raw_rtk.longitude;
    rtk_data->altitude = raw_rtk.altitude;
    rtk_data->hdop = raw_rtk.hdop;
    rtk_data->satellite_count = raw_rtk.satellite_count;
    rtk_data->fix_type = raw_rtk.fix_type;
    rtk_data->timestamp = raw_rtk.timestamp;

    return 0;
}

int rtk_um982_deinit(void *device_handle) {
    if (device_handle == NULL) {
        printf("RTK UM982: Invalid device handle\n");
        return -1;
    }

    RTKUM982Handle *handle = (RTKUM982Handle *)device_handle;
    
    // Deinit SDK
    RTKUM982_Deinit(handle);
    
    // Close serial port
    close(handle->serial_fd);
    
    // Free memory
    free(handle);
    return 0;
}

int rtk_um982_get_gnss_status(void *device_handle, GNSSStatus *status) {
    if (device_handle == NULL || status == NULL) {
        printf("RTK UM982: Invalid input parameters for get GNSS status\n");
        return -1;
    }

    RTKUM982Handle *handle = (RTKUM982Handle *)device_handle;
    int ret = RTKUM982_GetSignalQuality(handle, &status->signal_quality);
    if (ret != 0) {
        printf("RTK UM982: Get signal quality failed (code: %d)\n", ret);
        return -2;
    }

    status->satellite_count = RTKUM982_GetSatelliteCount(handle);
    status->rtk_status = RTKUM982_GetRTKStatus(handle);

    return 0;
}

int rtk_um982_set_baudrate(void *device_handle, int baudrate) {
    if (device_handle == NULL || (baudrate != 9600 && baudrate != 115200 && baudrate != 230400)) {
        printf("RTK UM982: Invalid baud rate (9600/115200/230400 required)\n");
        return -1;
    }

    RTKUM982Handle *handle = (RTKUM982Handle *)device_handle;
    int ret = RTKUM982_SetSerialBaudrate(handle, baudrate);
    if (ret != 0) {
        printf("RTK UM982: Set baud rate failed (code: %d)\n", ret);
        return -2;
    }

    // Reconfigure serial port
    struct termios options;
    tcgetattr(handle->serial_fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    tcsetattr(handle->serial_fd, TCSANOW, &options);

    return 0;
}

int rtk_um982_set_mode(void *device_handle, int mode) {
    if (device_handle == NULL || (mode < 0 || mode > 2)) {
        printf("RTK UM982: Invalid mode (0:single, 1:RTK, 2:static)\n");
        return -1;
    }

    RTKUM982Handle *handle = (RTKUM982Handle *)device_handle;
    int ret = RTKUM982_SetMode(handle, mode);
    if (ret != 0) {
        printf("RTK UM982: Set mode failed (code: %d)\n", ret);
        return -2;
    }

    return 0;
}