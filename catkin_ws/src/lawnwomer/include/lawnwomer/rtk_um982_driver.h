/**
 * @file rtk_um982_driver.h
 * @author Wuzf
 * @date 2026.01.15
 * @brief Driver for T-RTK UM982 (handsfree_rtk) via USB serial
 * @input USB serial handle, baud rate
 * @output RTK positioning data (NMEA-0183 parsed)
 */
#ifndef RTK_UM982_DRIVER_H
#define RTK_UM982_DRIVER_H

#include "third_party/rtk_um982_sdk/rtk_um982_api.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RTK_UM982_USB_VENDOR_ID 0x10c4
#define RTK_UM982_USB_PRODUCT_ID 0xea60
#define RTK_UM982_DEFAULT_BAUDRATE 115200
#define RTK_UM982_BUFFER_SIZE 1024

/**
 * @brief RTK positioning data structure
 */
typedef struct {
    double latitude;        // Latitude (deg)
    double longitude;       // Longitude (deg)
    double altitude;        // Altitude (m)
    float hdop;             // Horizontal dilution of precision
    int satellite_count;    // Number of satellites
    int fix_type;           // 0:no fix, 1:2D, 2:3D, 3:RTK fixed, 4:RTK float
    uint64_t timestamp;     // Timestamp (ms)
} RTKData;

/**
 * @brief GNSS status structure
 */
typedef struct {
    int signal_quality;     // 0-100 (0:no signal, 100:excellent)
    int satellite_count;    // Number of visible satellites
    int rtk_status;         // 0:invalid, 1:converging, 2:fixed
} GNSSStatus;

/**
 * @brief Initialize RTK UM982 device
 * @param device_handle [out] Pointer to device handle
 * @param baudrate [in] Serial baud rate (0 for default)
 * @return int 0:success, -1:invalid param, -2:serial open failed, -3:SDK init failed
 */
int rtk_um982_init(void **device_handle, int baudrate);

/**
 * @brief Read RTK data from device
 * @param device_handle [in] Device handle
 * @param rtk_data [out] Pointer to RTK data structure
 * @return int 0:success, -1:invalid param, -2:read failed, -3:parse failed
 */
int rtk_um982_read_data(void *device_handle, RTKData *rtk_data);

/**
 * @brief Deinitialize RTK UM982 device
 * @param device_handle [in] Device handle
 * @return int 0:success, -1:invalid handle
 */
int rtk_um982_deinit(void *device_handle);

/**
 * @brief Get GNSS status of RTK device
 * @param device_handle [in] Device handle
 * @param status [out] Pointer to GNSS status structure
 * @return int 0:success, -1:invalid param, -2:get status failed
 */
int rtk_um982_get_gnss_status(void *device_handle, GNSSStatus *status);

/**
 * @brief Set serial baud rate of RTK device
 * @param device_handle [in] Device handle
 * @param baudrate [in] Baud rate (9600/115200/230400)
 * @return int 0:success, -1:invalid param, -2:set failed
 */
int rtk_um982_set_baudrate(void *device_handle, int baudrate);

/**
 * @brief Set RTK working mode
 * @param device_handle [in] Device handle
 * @param mode [in] Mode (0:single, 1:RTK, 2:static)
 * @return int 0:success, -1:invalid param, -2:set failed
 */
int rtk_um982_set_mode(void *device_handle, int mode);

#endif // RTK_UM982_DRIVER_H