/**
 * @file hesai_jt128_driver.h
 * @author Wuzf
 * @date 2026.01.25
 * @brief Driver for Hesai JT128 LiDAR via Ethernet
 * @input LiDAR IP address, port
 * @output Point cloud data (X,Y,Z,intensity)
 */
#ifndef HESAI_JT128_DRIVER_H
#define HESAI_JT128_DRIVER_H

#include "third_party/hesai_jt128_sdk/hesai_lidar_api.h"

#define HESAI_JT128_DEFAULT_IP "192.168.1.201"
#define HESAI_JT128_DEFAULT_PORT 2368

typedef struct {
    float x;
    float y;
    float z;
    float intensity;
} HesaiPointXYZIT;

/**
 * @brief Initialize Hesai JT128 LiDAR
 * @param lidar_handle [out] Pointer to lidar handle
 * @param ip [in] LiDAR IP address
 * @param port [in] LiDAR data port
 * @return int 0:success, -1:failure
 */
int hesai_jt128_init(void **lidar_handle, const char *ip, int port);

/**
 * @brief Read point cloud data from LiDAR
 * @param lidar_handle [in] LiDAR handle
 * @param point_cloud [out] Pointer to point cloud data array
 * @param point_num [out] Number of points in point cloud
 * @return int 0:success, -1:failure
 */
int hesai_jt128_read_pointcloud(void *lidar_handle, HesaiPointXYZIT *point_cloud, int *point_num);

/**
 * @brief Deinitialize LiDAR
 * @param lidar_handle [in] LiDAR handle
 * @return int 0:success, -1:failure
 */
int hesai_jt128_deinit(void *lidar_handle);

#endif