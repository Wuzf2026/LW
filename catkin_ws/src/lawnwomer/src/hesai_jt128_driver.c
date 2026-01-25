/**
 * @file hesai_jt128_driver.c
 * @author Wuzf
 * @date 2026.01.25
 * @brief Implementation of Hesai JT128 driver functions
 */
#include "lawnwomer/hesai_jt128_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

int hesai_jt128_init(void **lidar_handle, const char *ip, int port) {
    if (lidar_handle == NULL) {
        printf("Hesai JT128: Invalid lidar handle pointer\n");
        return -1;
    }

    // Use default IP/Port if not provided
    const char *lidar_ip = (ip == NULL) ? HESAI_JT128_DEFAULT_IP : ip;
    int lidar_port = (port == 0) ? HESAI_JT128_DEFAULT_PORT : port;

    // Allocate device structure
    HesaiLidarDevice *dev = (HesaiLidarDevice *)malloc(sizeof(HesaiLidarDevice));
    if (dev == NULL) {
        printf("Hesai JT128: Memory allocation failed\n");
        return -1;
    }

    // Set IP/Port
    strncpy(dev->ip_addr, lidar_ip, 15);
    dev->ip_addr[15] = '\0';
    dev->port = lidar_port;

    // Initialize SDK handle
    int ret = HesaiLidar_Init(&dev->handle, dev->ip_addr, dev->port);
    if (ret != 0) {
        printf("Hesai JT128: SDK init failed (code: %d)\n", ret);
        free(dev);
        return -2;
    }

    // Connect to lidar
    ret = HesaiLidar_Connect(&dev->handle);
    if (ret != 0) {
        printf("Hesai JT128: Failed to connect to lidar (IP: %s, Port: %d)\n", dev->ip_addr, dev->port);
        HesaiLidar_Deinit(&dev->handle);
        free(dev);
        return -2;
    }

    // Start capture
    ret = HesaiLidar_StartCapture(&dev->handle);
    if (ret != 0) {
        printf("Hesai JT128: Failed to start capture\n");
        HesaiLidar_Disconnect(&dev->handle);
        HesaiLidar_Deinit(&dev->handle);
        free(dev);
        return -3;
    }

    *lidar_handle = (void *)dev;
    return 0;
}

int hesai_jt128_read_pointcloud(void *lidar_handle, HesaiPointXYZIT *point_cloud, int *point_num) {
    if (lidar_handle == NULL || point_cloud == NULL || point_num == NULL) {
        printf("Hesai JT128: Invalid input parameters\n");
        return -1;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    HesaiPointCloudRaw raw_frame;
    int ret = HesaiLidar_GetPointCloud(&dev->handle, &raw_frame);
    if (ret != 0) {
        printf("Hesai JT128: Failed to get point cloud (code: %d)\n", ret);
        return -2;
    }

    // Check point count
    if (raw_frame.point_count > HESAI_JT128_MAX_POINTS) {
        printf("Hesai JT128: Point count exceeds max limit (%d > %d)\n", raw_frame.point_count, HESAI_JT128_MAX_POINTS);
        return -2;
    }

    // Parse raw data to XYZIT format
    *point_num = raw_frame.point_count;
    for (int i = 0; i < *point_num; i++) {
        point_cloud[i].x = raw_frame.points[i].x / 1000.0f; // Convert mm to m
        point_cloud[i].y = raw_frame.points[i].y / 1000.0f;
        point_cloud[i].z = raw_frame.points[i].z / 1000.0f;
        point_cloud[i].intensity = raw_frame.points[i].intensity;
        point_cloud[i].timestamp = raw_frame.timestamp;
    }

    return 0;
}

int hesai_jt128_deinit(void *lidar_handle) {
    if (lidar_handle == NULL) {
        printf("Hesai JT128: Invalid lidar handle\n");
        return -1;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    
    // Stop capture
    HesaiLidar_StopCapture(&dev->handle);
    
    // Disconnect
    HesaiLidar_Disconnect(&dev->handle);
    
    // Deinit SDK
    HesaiLidar_Deinit(&dev->handle);
    
    // Free memory
    free(dev);
    return 0;
}

int hesai_jt128_get_status(void *lidar_handle, HesaiJT128Status *status) {
    if (lidar_handle == NULL || status == NULL) {
        printf("Hesai JT128: Invalid input parameters for get status\n");
        return -1;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    int ret = HesaiLidar_GetConnectionStatus(&dev->handle, &status->connection_status);
    if (ret != 0) {
        printf("Hesai JT128: Get connection status failed (code: %d)\n", ret);
        return -2;
    }

    ret = HesaiLidar_GetFrameRate(&dev->handle, &status->frame_rate);
    if (ret != 0) {
        printf("Hesai JT128: Get frame rate failed (code: %d)\n", ret);
        return -2;
    }

    status->point_count = HesaiLidar_GetPointCountPerFrame(&dev->handle);
    return 0;
}

int hesai_jt128_set_framerate(void *lidar_handle, int fps) {
    if (lidar_handle == NULL || (fps != 10 && fps != 20 && fps != 30)) {
        printf("Hesai JT128: Invalid frame rate (10/20/30 Hz required)\n");
        return -1;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    int ret = HesaiLidar_SetFrameRate(&dev->handle, fps);
    if (ret != 0) {
        printf("Hesai JT128: Set frame rate failed (code: %d)\n", ret);
        return -2;
    }

    return 0;
}