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

int hesai_jt128_init(void **lidar_handle, const char *ip, int port)
{
    if (lidar_handle == NULL || ip == NULL)
    {
        printf("Hesai JT128: Invalid input parameters\n");
        return -1;
    }

    HesaiLidarDevice dev;
    strcpy(dev.ip_addr, ip);
    dev.port = port;

    int ret = HesaiLidar_Connect(&dev);
    if (ret != 0)
    {
        printf("Hesai JT128: Failed to connect to lidar\n");
        return -1;
    }

    ret = HesaiLidar_StartCapture(&dev);
    if (ret != 0)
    {
        printf("Hesai JT128: Failed to start capture\n");
        HesaiLidar_Disconnect(&dev);
        return -1;
    }

    *lidar_handle = malloc(sizeof(HesaiLidarDevice));
    memcpy(*lidar_handle, &dev, sizeof(HesaiLidarDevice));
    return 0;
}

int hesai_jt128_read_pointcloud(void *lidar_handle, HesaiPointXYZIT *point_cloud, int *point_num)
{
    if (lidar_handle == NULL || point_cloud == NULL || point_num == NULL)
    {
        printf("Hesai JT128: Invalid input parameters\n");
        return -1;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    HesaiPointCloud frame;
    int ret = HesaiLidar_GetPointCloud(dev, &frame);
    if (ret != 0)
    {
        printf("Hesai JT128: Failed to get point cloud\n");
        return -1;
    }

    *point_num = frame.point_count;
    for (int i = 0; i < *point_num; i++)
    {
        point_cloud[i].x = frame.points[i].x;
        point_cloud[i].y = frame.points[i].y;
        point_cloud[i].z = frame.points[i].z;
        point_cloud[i].intensity = frame.points[i].intensity;
    }
    return 0;
}

int hesai_jt128_deinit(void *lidar_handle)
{
    if (lidar_handle == NULL)
    {
        return 0;
    }

    HesaiLidarDevice *dev = (HesaiLidarDevice *)lidar_handle;
    HesaiLidar_StopCapture(dev);
    HesaiLidar_Disconnect(dev);
    free(lidar_handle);
    return 0;
}