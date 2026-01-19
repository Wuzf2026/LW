/**
 * @file rtk_um982_driver.h
 * @author Wuzf
 * @date 2026.02.01
 * @brief Driver for T-RTK UM982 via USB serial port
 * @input Serial port name, baud rate
 * @output RTK positioning data (lat, lon, alt, heading)
 */
#ifndef RTK_UM982_DRIVER_H
#define RTK_UM982_DRIVER_H

#include "third_party/rtk_um982_sdk/um982_api.h"

#define RTK_UM982_DEFAULT_BAUDRATE 115200
#define RTK_UM982_DEFAULT_SERIAL "/dev/ttyUSB0"

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double heading;
    bool is_valid;
} RTKData;

/**
 * @brief Initialize RTK UM982 module
 * @param serial_handle [out] Pointer to serial port handle
 * @param serial_name [in] Serial port device name
 * @param baudrate [in] Serial port baud rate
 * @return int 0:success, -1:failure
 */
int rtk_um982_init(void **serial_handle, const char *serial_name, int baudrate);

/**
 * @brief Read RTK data from module
 * @param serial_handle [in] Serial port handle
 * @param rtk_data [out] Pointer to RTK data structure
 * @return int 0:success, -1:failure
 */
int rtk_um982_read_data(void *serial_handle, RTKData *rtk_data);

/**
 * @brief Deinitialize RTK module
 * @param serial_handle [in] Serial port handle
 * @return int 0:success, -1:failure
 */
int rtk_um982_deinit(void *serial_handle);

#endif