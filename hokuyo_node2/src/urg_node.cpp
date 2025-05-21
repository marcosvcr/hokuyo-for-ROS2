#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include <vector>

class HokuyoNode : public rclcpp::Node {
public:
    HokuyoNode() : Node("hokuyo_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        if (urg_open(&urg_, URG_SERIAL, "/dev/ttyACM0", 115200) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Hokuyo LiDAR.");
            return;
        }

        // Get LiDAR parameters (min/max steps, angles)
        urg_set_scanning_parameter(&urg_, urg_deg2step(&urg_, -135), urg_deg2step(&urg_, 135), 0);
        max_data_size_ = urg_max_data_size(&urg_);	

        urg_start_measurement(&urg_, URG_DISTANCE, URG_SCAN_INFINITY, 0);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HokuyoNode::publish_scan, this));
    }

    ~HokuyoNode() {
        urg_close(&urg_);
    }

private:
    void publish_scan() {
        std::vector<long> data(max_data_size_);
        int n = urg_get_distance(&urg_, &data[0], nullptr, nullptr);

        if (n <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get scan data.");
            return;
        }

        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->get_clock()->now();
        scan_msg.header.frame_id = "laser";
        scan_msg.angle_min = -2.356194;
        scan_msg.angle_max = 2.356194;
        scan_msg.angle_increment = urg_step2rad(&urg_, 1);
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 30.0;

        scan_msg.ranges.resize(n);
        for (int i = 0; i < n; ++i) {
            scan_msg.ranges[i] = data[i] / 1000.0;
        }

        publisher_->publish(scan_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    urg_t urg_;
    int max_data_size_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HokuyoNode>());
    rclcpp::shutdown();
    return 0;
}
