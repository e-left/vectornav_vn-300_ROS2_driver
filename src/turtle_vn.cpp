#include "vn_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions nodeOptions;
    nodeOptions.automatically_declare_parameters_from_overrides(true);

    rclcpp::spin(std::make_shared<VNPublisher>(nodeOptions));

    rclcpp::shutdown();
    return 0;
}
