/**
 * @file gemini335_node.cpp
 * @author Wuzf
 * @date 2026.01.28
 * @brief ROS node for Orbbec Gemini335 depth camera
 */
#include "ros/ros.h"
#include "lawnwomer/gemini335_driver.h"
#include "lawnwomer/Gemini335Depth.h"
#include "lawnwomer/SetMode.srv.h"
#include <unistd.h>

class Gemini335Node {
private:
    ros::NodeHandle nh_;
    ros::Publisher depth_pub_;
    ros::ServiceServer set_mode_srv_;
    void *device_handle_;
    int publish_rate_;
    std::string frame_id_;

public:
    Gemini335Node() : device_handle_(NULL) {
        // Get parameters from ROS param server
        nh_.param<int>("publish_rate", publish_rate_, 10);
        nh_.param<std::string>("frame_id", frame_id_, "gemini335_depth");

        // Initialize Gemini335 camera
        int ret = gemini335_init(&device_handle_);
        if (ret != 0) {
            ROS_ERROR("Gemini335 initialization failed (code: %d)", ret);
            ros::shutdown();
            return;
        }
        ROS_INFO("Gemini335 camera initialized successfully");

        // Create publisher
        depth_pub_ = nh_.advertise<lawnwomer::Gemini335Depth>("/lawnwomer/gemini335/depth", 10);

        // Create service server
        set_mode_srv_ = nh_.advertiseService("/lawnwomer/gemini335/set_mode", &Gemini335Node::setModeCallback, this);
    }

    ~Gemini335Node() {
        if (device_handle_ != NULL) {
            gemini335_deinit(device_handle_);
            ROS_INFO("Gemini335 camera deinitialized");
        }
    }

    bool setModeCallback(lawnwomer::SetMode::Request &req, lawnwomer::SetMode::Response &res) {
        // Mode 0: default exposure, Mode 1: high exposure, Mode 2: low exposure
        int exposure;
        switch (req.mode) {
            case 0: exposure = 10; break;
            case 1: exposure = 50; break;
            case 2: exposure = 5; break;
            default:
                res.success = false;
                res.msg = "Invalid mode (0:default,1:high,2:low)";
                return true;
        }

        int ret = gemini335_set_exposure(device_handle_, exposure);
        if (ret == 0) {
            res.success = true;
            res.msg = "Exposure set to " + std::to_string(exposure) + " ms";
            ROS_INFO("Gemini335 exposure set to %d ms", exposure);
        } else {
            res.success = false;
            res.msg = "Set exposure failed (code: " + std::to_string(ret) + ")";
        }
        return true;
    }

    void run() {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            // Read depth data
            uint16_t depth_data[GEMINI335_DEPTH_SIZE];
            int data_len;
            int ret = gemini335_read_depth(device_handle_, depth_data, &data_len);
            if (ret != 0) {
                ROS_ERROR("Read Gemini335 depth data failed (code: %d)", ret);
                rate.sleep();
                continue;
            }

            // Create ROS message
            lawnwomer::Gemini335Depth msg;
            msg.width = GEMINI335_DEPTH_WIDTH;
            msg.height = GEMINI335_DEPTH_HEIGHT;
            msg.depth_data.assign(depth_data, depth_data + data_len);
            msg.timestamp = ros::Time::now().toNSec() / 1000000; // Convert to ms

            // Publish message
            depth_pub_.publish(msg);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gemini335_node");
    Gemini335Node node;
    node.run();
    return 0;
}