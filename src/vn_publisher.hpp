#ifndef VN_PUBLISHER_HPP
#define VN_PUBLISHER_HPP

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtle_interfaces/msg/vn_euler.hpp"
#include "turtle_interfaces/msg/vn_gps_position.hpp"
#include "turtle_interfaces/msg/vn_gps_velocity.hpp"
#include "turtle_interfaces/msg/vn_imu.hpp"
#include "turtle_interfaces/msg/vn_ins_status.hpp"
#include "turtle_interfaces/msg/vn_quat.hpp"
#include "turtle_interfaces/msg/vn_utc_time.hpp"
#include "vn/include/vn/sensors.h"
#include <string>
#include <tf2_ros/transform_broadcaster.h>

class VNPublisher : public rclcpp::Node {
private:
    // sensor object
    vn::sensors::VnSensor vs;

    // connection parameters
    uint32_t baudRate;
    std::string port;
    bool configureRegisters;

    // other parameters
    double initialLon;
    double initialLat;
    bool publishTf;

    // function to load parameters
    void loadParameters();

    // function to try and connect
    bool tryConnection(std::string port, uint32_t baudRate);

    // setup binary registers
    void setupBinaryOutputRegisters();

    // binary registers async callback
    static void binaryOutputRegisterCallback(void *userData, vn::protocol::uart::Packet &p, size_t index);

    // setup callbacks
    void setupVNCallbacks();

    // ros2 stuff
    void initializePublishersAndMessages();

    // reboot service and function
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr vnRebootService;
    void vnRebootDevice(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

    // utc publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnUtcTime>::SharedPtr vnUtcPublisher;

    // euler publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnEuler>::SharedPtr vnEulerPublisher;

    // quaternion publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnQuat>::SharedPtr vnQuaternionPublisher;

    // IMU publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnImu>::SharedPtr vnImuPublisher;

    // GPS position 1 publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnGpsPosition>::SharedPtr vnGpsPosition1Publisher;

    // GPS position 2 publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnGpsPosition>::SharedPtr vnGpsPosition2Publisher;

    // GPS velocity 1 publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnGpsVelocity>::SharedPtr vnGpsVelocity1Publisher;

    // GPS velocity 2 publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnGpsVelocity>::SharedPtr vnGpsVelocity2Publisher;

    // odom publisher (EKF body velocities + LLA positions), along with INS status
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vnOdomPublisher;

    // INS status publisher
    rclcpp::Publisher<turtle_interfaces::msg::VnInsStatus>::SharedPtr vnInsStatusPublisher;

    // tf broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // utility functions
    double calcGeoRadius(double lat);
    double degToRad(double deg);

public:
    // due to the binaryOutputRegisterCallback function being static,
    // the publishers have to be public
    // utc publisher function
    void publishUtc(uint64_t timestamp);

    // euler publisher function
    void publishEuler(float roll, float pitch, float yaw, float rollU, float pitchU, float yawU);

    // quaternion publisher function
    void publishQuaternion(float x, float y, float z, float w);

    // IMU publisher function
    void publishImu(float ax, float ay, float az, float rx, float ry, float rz);

    // GPS position 1 publisher function
    void publishGpsPosition1(uint8_t numSats, uint8_t fix, double lon, double lat, double att, double uncertaintyN, double uncertaintyE, double uncertaintyD);

    // GPS position 2 publisher function
    void publishGpsPosition2(uint8_t numSats, uint8_t fix, double lon, double lat, double att, double uncertaintyN, double uncertaintyE, double uncertaintyD);

    // GPS velocity 1 publisher function
    void publishGpsVelocity1(float veln, float vele, float veld, float uncertainty);

    // GPS velocity 2 publisher function
    void publishGpsVelocity2(float veln, float vele, float veld, float uncertainty);

    // odom publisher (EKF body velocities + LLA positions), along with INS status function
    void publishOdom(double lon, double lat, double att, float velx, float vely, float r, float x, float y, float z, float w, float rollU, float pitchU, float yawU, float posU);

    // INS status publisher function
    void publishInsStatus(uint16_t status);

    // tf broadcaster
    void broadcastTf(double lon, double lat, double att, float qx, float qy, float qz, float qw);

    VNPublisher(rclcpp::NodeOptions nodeOptions);
    ~VNPublisher();
};

#endif
