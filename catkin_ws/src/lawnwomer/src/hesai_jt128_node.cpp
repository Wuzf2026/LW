/**
 * @file hesai_jt128_node.cpp
 * @author Wuzf
 * @date 2026.01.29
 * @brief ROS node for Hesai JT128 lidar
 */
#include "ros/ros.h"
#include "lawnwomer/hesai_jt128_driver.h"
#include "lawnwomer/HesaiJT128PointCloud.h"
#include "lawnwomer/HesaiPoint.h"
#include "lawnwomer/SetMode.srv.h"
#include <unistd.h>

class HesaiJT128Node {
private:
    ros::NodeHandle nh_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceServer set_mode_srv_;
    void *lidar_handle_;
    int publish_rate_;
    std::string frame_id_;
    std::string lidar_ip_;
    int lidar_port_;

public:
    HesaiJT128Node() : lidar_handle_(NULL) {
        // Get parameters from ROS param server
        nh_.param<int>("publish_rate", publish_rate_, 10);
        nh_.param<std::string>("frame_id", frame_id_, "hesai_jt128");
        nh_.param<std::string>("lidar_ip", lidar_ip_, HESAI_JT128_DEFAULT_IP);
        nh_.param<int>("lidar_port", lidar_port_, HESAI_JT128_DEFAULT_PORT);

        // Initialize Hesai JT128 lidar
        int ret = hesai_jt128_init(&lidar_handle_, lidar_ip_.c_str(), lidar_port_);
        if (ret != 0) {
            ROS_ERROR("Hesai JT128 initialization failed (code: %d)", ret);
            ros::shutdown();
            return;
        }
        ROS_INFO("Hesai JT128 lidar initialized (IP: %s, Port: %d)", lidar_ip_.c_str(), lidar_port_);

        // Create publisher
        pointcloud_pub_ = nh_.advertise<lawnwomer::HesaiJT128PointCloud>("/lawnwomer/hesai/pointcloud", 10);

        // Create service server
        set_mode_srv_ = nh_.advertiseService("/lawnwomer/hesai/set_mode", &HesaiJT128Node::setModeCallback, this);
    }

    ~HesaiJT128Node() {
        if (lidar_handle_ != NULL) {
            hesai_jt128_deinit(lidar_handle_);
            ROS_INFO("Hesai JT128 lidar deinitialized");
        }
    }

    bool setModeCallback(lawnwomer::SetMode::Request &req, lawnwomer::SetMode::Response &res) {
        // Mode 0:10Hz, Mode1:20Hz, Mode2:30Hz
        int fps;
        switch (req.mode) {
            case 0: fps = 10; break;
            case 1: fps = 20; break;
            case 2: fps = 30; break;
            default:
                res.success = false;
                res.msg = "Invalid mode (0:10Hz,1:20Hz,2:30Hz)";
                return true;
        }

        int ret = hesai_jt128_set_framerate(lidar_handle_, fps);
        if (ret == 0) {
            res.success = true;
            res.msg = "Frame rate set to " + std::to_string(fps) + " Hz";
            ROS_INFO("Hesai JT128 frame rate set to %d Hz", fps);
            publish_rate_ = fps; // Update publish rate
        } else {
            res.success = false;
            res.msg = "Set frame rate failed (code: " + std::to_string(ret) + ")";
        }
        return true;
    }

    void run() {
        ros::Rate rate(publish_rate_);
        HesaiPointXYZIT point_cloud[HESAI_JT128_MAX_POINTS];
        int point_num;

        while (ros::ok()) {
            // Read point cloud
            int ret = hesai_jt128_read_pointcloud(lidar_handle_, point_cloud, &point_num);
            if (ret != 0) {
                ROS_ERROR("Read Hesai JT128 point cloud failed (code: %d)", ret);
                rate.sleep();
                continue;
            }

            // Create ROS message
            lawnwomer::HesaiJT128PointCloud msg;
            msg.timestamp = ros::Time::now().toNSec() / 1000; // Convert to us

            // Fill points
            for (int i = 0; i < point_num; i++) {
                lawnwomer::HesaiPoint p;
                p.x = point_cloud[i].x;
                p.y = point_cloud[i].y;
                p.z = point_cloud[i].z;
                p.intensity = point_cloud[i].intensity;
                msg.points.push_back(p);
            }

            // Publish message
            pointcloud_pub_.publish(msg);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "hesai_jt128_node");
    HesaiJT128Node node;
    node.run();
    return 0;
}