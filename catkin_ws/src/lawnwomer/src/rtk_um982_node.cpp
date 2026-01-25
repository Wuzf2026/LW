/**
 * @file rtk_um982_node.cpp
 * @author Wuzf
 * @date 2026.01.30
 * @brief ROS node for T-RTK UM982
 */
#include "ros/ros.h"
#include "lawnwomer/rtk_um982_driver.h"
#include "lawnwomer/RTKData.msg"
#include "lawnwomer/SetMode.srv.h"
#include <unistd.h>

class RTKUM982Node {
private:
    ros::NodeHandle nh_;
    ros::Publisher rtk_pub_;
    ros::ServiceServer set_mode_srv_;
    void *device_handle_;
    int publish_rate_;
    std::string frame_id_;
    int baudrate_;

public:
    RTKUM982Node() : device_handle_(NULL) {
        // Get parameters from ROS param server
        nh_.param<int>("publish_rate", publish_rate_, 10);
        nh_.param<std::string>("frame_id", frame_id_, "rtk_um982");
        nh_.param<int>("baudrate", baudrate_, RTK_UM982_DEFAULT_BAUDRATE);

        // Initialize RTK UM982
        int ret = rtk_um982_init(&device_handle_, baudrate_);
        if (ret != 0) {
            ROS_ERROR("RTK UM982 initialization failed (code: %d)", ret);
            ros::shutdown();
            return;
        }
        ROS_INFO("RTK UM982 initialized (baud rate: %d)", baudrate_);

        // Create publisher
        rtk_pub_ = nh_.advertise<lawnwomer::RTKData>("/lawnwomer/rtk/data", 10);

        // Create service server
        set_mode_srv_ = nh_.advertiseService("/lawnwomer/rtk/set_mode", &RTKUM982Node::setModeCallback, this);
    }

    ~RTKUM982Node() {
        if (device_handle_ != NULL) {
            rtk_um982_deinit(device_handle_);
            ROS_INFO("RTK UM982 deinitialized");
        }
    }

    bool setModeCallback(lawnwomer::SetMode::Request &req, lawnwomer::SetMode::Response &res) {
        // Mode 0:single, 1:RTK, 2:static
        if (req.mode < 0 || req.mode > 2) {
            res.success = false;
            res.msg = "Invalid mode (0:single,1:RTK,2:static)";
            return true;
        }

        int ret = rtk_um982_set_mode(device_handle_, req.mode);
        if (ret == 0) {
            res.success = true;
            res.msg = "RTK mode set to " + std::to_string(req.mode);
            ROS_INFO("RTK UM982 mode set to %d", req.mode);
        } else {
            res.success = false;
            res.msg = "Set RTK mode failed (code: " + std::to_string(ret) + ")";
        }
        return true;
    }

    void run() {
        ros::Rate rate(publish_rate_);
        RTKData rtk_data;

        while (ros::ok()) {
            // Read RTK data
            int ret = rtk_um982_read_data(device_handle_, &rtk_data);
            if (ret != 0) {
                ROS_ERROR("Read RTK UM982 data failed (code: %d)", ret);
                rate.sleep();
                continue;
            }

            // Create ROS message
            lawnwomer::RTKData msg;
            msg.latitude = rtk_data.latitude;
            msg.longitude = rtk_data.longitude;
            msg.altitude =