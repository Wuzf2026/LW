/**
 * @file hesai_jt128_driver.h
 * @author Wuzf
 * @date 2026.01.25
 * @brief Driver for Hesai JT128 lidar via Ethernet
 * @input IP/Port, lidar handle
 * @output Point cloud data (XYZIT format)
 */
#ifndef HESAI_JT128_DRIVER_H
#define HESAI_JT128_DRIVER_H

#include "third_party/hesai_jt128_sdk/hesai_jt128_api.h"
#include <stdint.h>
#include <stdlib.h>

#define HESAI_JT128_DEFAULT_IP "192.168.1.200"
#define HESAI_JT128_DEFAULT_PORT 9347
#define HESAI_JT128_MAX_POINTS 128000

/**
 * @brief Point structure for Hesai JT128 (XYZ + Intensity + Timestamp)
 */
typedef struct {
    float x;          // X coordinate (m)
    float y;          // Y coordinate (m)
    float z;          // Z coordinate (m)
    uint16_t intensity; // Reflectivity (0-255)
    uint64_t timestamp; // Timestamp (us)
} HesaiPointXYZIT;

/**
 * @brief Lidar status structure
 */
typedef struct {
    int connection_status; // 0:disconnected, 1:connected
    int frame_rate;        // Current frame rate (Hz)
    int point_count;       // Points per frame
} HesaiJT128Status;

/**
 * @brief Lidar device structure
 */
typedef struct {
    char ip_addr[16];      // Lidar IP address
    int port;              // Lidar port
    HesaiJT128Handle handle; // SDK handle
} HesaiLidarDevice;

/**
 * @brief Initialize Hesai JT128 lidar
 * @param lidar_handle [out] Pointer to lidar handle
 * @param ip [in] Lidar IP address (NULL for default)
 * @param port [in] Lidar port (0 for default)
 * @return int 0:success, -1:invalid param, -2:connect failed, -3:start capture failed
 */
int hesai_jt128_init(void **lidar_handle, const char *ip, int port);

/**
 * @brief Read point cloud from lidar
 * @param lidar_handle [in] Lidar handle
 * @param point_cloud [out] Pointer to point cloud array
 * @param point_num [out] Number of points in frame
 * @return int 0:success, -1:invalid param, -2:get point cloud failed
 */
int hesai_jt128_read_pointcloud(void *lidar_handle, HesaiPointXYZIT *point_cloud, int *point_num);

/**
 * @brief Deinitialize Hesai JT128 lidar
 * @param lidar_handle [in] Lidar handle
 * @return int 0:success, -1:invalid handle
 */
int hesai_jt128_deinit(void *lidar_handle);

/**
 * @brief Get Hesai JT128 status
 * @param lidar_handle [in] Lidar handle
 * @param status [out] Pointer to status structure
 * @return int 0:success, -1:invalid param, -2:get status failed
 */
int hesai_jt128_get_status(void *lidar_handle, HesaiJT128Status *status);

/**
 * @brief Set frame rate of Hesai JT128
 * @param lidar_handle [in] Lidar handle
 * @param fps [in] Frame rate (10/20/30 Hz)
 * @return int 0:success, -1:invalid param, -2:set failed
 */
int hesai_jt128_set_framerate(void *lidar_handle, int fps);

#endif // HESAI_JT128_DRIVER_H