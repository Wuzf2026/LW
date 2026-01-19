/**
 * @file main_node.cpp
 * @author Wuzf
 * @date 2026.02.05
 * @brief Core ROS node for lawnwomer robot
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include "lawnwomer/gemini335_driver.h"
#include "lawnwomer/hesai_jt128_driver.h"
#include "lawnwomer/rtk_um982_driver.h"
#include "lawnwomer/RTKData.h"
#include "lawnwomer/SetMode.h"

class LawnwomerNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher rtk_pub_;
    ros::Publisher depth_pub_;
    ros::Publisher pcl_pub_;
    ros::ServiceServer mode_srv_;

    void *gemini335_handle_;
    void *hesai_handle_;
    void *rtk_handle_;

    int loop_rate_;
    std::string frame_id_;

public:
    LawnwomerNode()
    {
        // Load parameters
        nh_.param<int>("node/loop_rate", loop_rate_, 10);
        nh_.param<std::string>("node/frame_id", frame_id_, "base_link");

        // Initialize publishers
        rtk_pub_ = nh_.advertise<lawnwomer::RTKData>("/rtk_data", 10);
        depth_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("/depth_data", 10);
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_data", 10);

        // Initialize service
        mode_srv_ = nh_.advertiseService("/set_mode", &LawnwomerNode::setModeCallback, this);

        // Initialize sensors
        std::string rtk_serial;
        int rtk_baud;
        nh_.param<std::string>("rtk_um982/serial_name", rtk_serial, "/dev/ttyUSB0");
        nh_.param<int>("rtk_um982/baudrate", rtk_baud, 115200);
        rtk_um982_init(&rtk_handle_, rtk_serial.c_str(), rtk_baud);

        std::string hesai_ip;
        int hesai_port;
        nh_.param<std::string>("hesai_jt128/ip", hesai_ip, "192.168.1.201");
        nh_.param<int>("hesai_jt128/port", hesai_port, 2368);
        hesai_jt128_init(&hesai_handle_, hesai_ip.c_str(), hesai_port);

        gemini335_init(&gemini335_handle_);

        ROS_INFO("Lawnwomer node initialized successfully");
    }

    ~LawnwomerNode()
    {
        gemini335_deinit(gemini335_handle_);
        hesai_jt128_deinit(hesai_handle_);
        rtk_um982_deinit(rtk_handle_);
        ROS_INFO("Lawnwomer node deinitialized");
    }

    bool setModeCallback(lawnwomer::SetMode::Request &req, lawnwomer::SetMode::Response &res)
    {
        if (req.mode < 0 || req.mode > 2)
        {
            res.success = false;
            res.message = "Invalid mode (0:idle,1:mowing,2:charging)";
            return false;
        }
        res.success = true;
        res.message = "Mode set to " + std::to_string(req.mode);
        ROS_INFO("Mode set to %d", req.mode);
        return true;
    }

    void run()
    {
        ros::Rate rate(loop_rate_);
        lawnwomer::RTKData rtk_msg;
        std_msgs::UInt16MultiArray depth_msg;
        sensor_msgs::PointCloud2 pcl_msg;

        while (ros::ok())
        {
            // Read RTK data
            RTKData rtk_data;
            if (rtk_um982_read_data(rtk_handle_, &rtk_data) == 0)
            {
                rtk_msg.latitude = rtk_data.latitude;
                rtk_msg.longitude = rtk_data.longitude;
                rtk_msg.altitude = rtk_data.altitude;
                rtk_msg.heading = rtk_data.heading;
                rtk_msg.is_valid = rtk_data.is_valid;
                rtk_pub_.publish(rtk_msg);
            }

            // Read depth data (simplified)
            uint16_t depth_data[640*480];
            int data_len;
            if (gemini335_read_depth(gemini335_handle_, depth_data, &data_len) == 0)
            {
                depth_msg.data.assign(depth_data, depth_data + data_len);
                depth_pub_.publish(depth_msg);
            }

            // Read point cloud data (simplified)
            HesaiPointXYZIT pcl_data[128000];
            int point_num;
            if (hesai_jt128_read_pointcloud(hesai_handle_, pcl_data, &point_num) == 0)
            {
                pcl_msg.header.stamp = ros::Time::now();
                pcl_msg.header.frame_id = frame_id_;
                pcl_pub_.publish(pcl_msg);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lawnwomer_node");
    LawnwomerNode node;
    node.run();
    return 0;
}