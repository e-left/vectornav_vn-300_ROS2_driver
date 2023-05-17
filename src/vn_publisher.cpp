#include "vn_publisher.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#define EQU_R 6378.1370          // Equatorial radius in km
#define EQU_R2 40680631.590769   // km2
#define POL_R 6356.7523          // Polar radius in km
#define POL_R2 40408299.80355529 // km2

std::string exec(const char *cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

VNPublisher::VNPublisher(rclcpp::NodeOptions nodeOptions) : Node("vn_publisher", "", nodeOptions) {
    // load parameters
    this->loadParameters();

    // debug
    RCLCPP_INFO(this->get_logger(), "Connecting...");

    // connect to specified port
    if (!this->tryConnection(this->port, this->baudRate)) {
        RCLCPP_ERROR(this->get_logger(), "Error: no sensor found on port %s", this->port.c_str());
        RCLCPP_ERROR(this->get_logger(), "Exiting");
        exit(1);
    }

    // Device found
    // debug: print model number
    std::string modelNumber = this->vs.readModelNumber();
    RCLCPP_INFO(this->get_logger(), "Device found at %s", this->port.c_str());
    RCLCPP_INFO(this->get_logger(), "Model Number: %s", modelNumber.c_str());

    // debug: print connected
    RCLCPP_INFO(this->get_logger(), "Connected");

    // initialize publishers and messages
    this->initializePublishersAndMessages();

    // setup callbacks here
    this->setupVNCallbacks();

    RCLCPP_INFO(this->get_logger(), "Publishing...");
};

void VNPublisher::loadParameters() {
    // load parameters from file
    this->get_parameter_or<uint32_t>("baudrate", this->baudRate, 115200);
    this->get_parameter_or<std::string>("port", this->port, "/dev/ins");
    this->get_parameter_or<bool>("configBor", this->configureRegisters, true);
    this->get_parameter_or<double>("initialLat", this->initialLat, 40.57699825538099);
    this->get_parameter_or<double>("initialLon", this->initialLon, 23.027571201613714);
    this->get_parameter_or<bool>("publishTf", this->publishTf, true);

    // debug: print parameters
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", this->baudRate);
    RCLCPP_INFO(this->get_logger(), "Port: %s", this->port.c_str());
    RCLCPP_INFO(this->get_logger(), "Configure binary output: %s", this->configureRegisters ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Initial Latitude: %f", this->initialLat);
    RCLCPP_INFO(this->get_logger(), "Initial Longitude: %f", this->initialLon);
    RCLCPP_INFO(this->get_logger(), "Publish TF: %s", this->publishTf ? "true" : "false");

    if (this->initialLat == 40.57699825538099 && this->initialLon == 23.027571201613714) {
        RCLCPP_INFO(this->get_logger(), "Sus");
    }
}

bool VNPublisher::tryConnection(std::string port, uint32_t baudRate) {
    try {
        // connect to sensor
        this->vs.connect(this->port, this->baudRate);
    } catch (vn::not_found e) {
        // could not find sensor
        return false;
    }

    // sensor connected without error
    return true;
}


void VNPublisher::setupBinaryOutputRegisters() {

    // first, common message
    vn::sensors::BinaryOutputRegister bor1(
        vn::protocol::uart::ASYNCMODE_BOTH, // only port 1 when deploying TODO: Include DSPACE messages
        (uint16_t)8,                        // 800 / divisor(8) = 100 Hz
        vn::protocol::uart::COMMONGROUP_NONE,
        vn::protocol::uart::TIMEGROUP_NONE,
        vn::protocol::uart::IMUGROUP_NONE,
        vn::protocol::uart::GPSGROUP_NONE,
        vn::protocol::uart::ATTITUDEGROUP_NONE,
        vn::protocol::uart::INSGROUP_VELBODY,
        vn::protocol::uart::GPSGROUP_NONE);

    // second message
    vn::sensors::BinaryOutputRegister bor2(
        vn::protocol::uart::ASYNCMODE_BOTH, // only port 2 when deploying
        (uint16_t)4,                        // 800 / divisor(8) = 100 Hz
        vn::protocol::uart::COMMONGROUP_YAWPITCHROLL | vn::protocol::uart::COMMONGROUP_TIMEGPS | vn::protocol::uart::COMMONGROUP_QUATERNION | vn::protocol::uart::COMMONGROUP_ANGULARRATE | vn::protocol::uart::COMMONGROUP_IMU | vn::protocol::uart::COMMONGROUP_VELOCITY,
        vn::protocol::uart::TIMEGROUP_NONE,
        vn::protocol::uart::IMUGROUP_NONE,
        vn::protocol::uart::GPSGROUP_NONE,
        vn::protocol::uart::ATTITUDEGROUP_YPRU,
        vn::protocol::uart::INSGROUP_VELBODY | vn::protocol::uart::INSGROUP_POSLLA | vn::protocol::uart::INSGROUP_INSSTATUS | vn::protocol::uart::INSGROUP_POSU,
        vn::protocol::uart::GPSGROUP_NONE);

    // third message
    vn::sensors::BinaryOutputRegister bor3(
        vn::protocol::uart::ASYNCMODE_BOTH, // only port 2 when deploying
        (uint16_t)4,                        // 800 / divisor(16) = 50 Hz
        vn::protocol::uart::COMMONGROUP_NONE,
        vn::protocol::uart::TIMEGROUP_NONE,
        vn::protocol::uart::IMUGROUP_NONE,
        vn::protocol::uart::GPSGROUP_NUMSATS | vn::protocol::uart::GPSGROUP_FIX | vn::protocol::uart::GPSGROUP_POSLLA | vn::protocol::uart::GPSGROUP_VELNED | vn::protocol::uart::GPSGROUP_POSU | vn::protocol::uart::GPSGROUP_VELU,
        vn::protocol::uart::ATTITUDEGROUP_NONE,
        vn::protocol::uart::INSGROUP_NONE,
        vn::protocol::uart::GPSGROUP_NUMSATS | vn::protocol::uart::GPSGROUP_FIX | vn::protocol::uart::GPSGROUP_POSLLA | vn::protocol::uart::GPSGROUP_VELNED | vn::protocol::uart::GPSGROUP_POSU | vn::protocol::uart::GPSGROUP_VELU);

    // register the custom messages here
    this->vs.writeBinaryOutput1(bor1);
    this->vs.writeBinaryOutput2(bor2);
    this->vs.writeBinaryOutput3(bor3);

    RCLCPP_INFO(this->get_logger(), "Configured binary output registers");
}

// note that this is a static function, to overcome C++ limitations on types and passing bound functions as callbacks
// so, we use the userData pointer to access the this object
void VNPublisher::binaryOutputRegisterCallback(void *userData, vn::protocol::uart::Packet &p, size_t index) {
    // get access to the class pointer
    VNPublisher *thisPointer = (VNPublisher *)userData;

    // distinction

    // first ensure that the package is a binary package
    if (p.type() != vn::protocol::uart::Packet::TYPE_BINARY) {
        RCLCPP_INFO(thisPointer->get_logger(), "Non binary packet received");
        return;
    }

    RCLCPP_INFO(thisPointer->get_logger(), "Binary packet received");

    // variable to track which output register message it is. either 1 or 2
    int borPackage;

    // differentiate between the two messages
    if (p.isCompatible(
            vn::protocol::uart::COMMONGROUP_TIMEGPS | vn::protocol::uart::COMMONGROUP_YAWPITCHROLL | vn::protocol::uart::COMMONGROUP_QUATERNION | vn::protocol::uart::COMMONGROUP_ANGULARRATE | vn::protocol::uart::COMMONGROUP_POSITION | vn::protocol::uart::COMMONGROUP_VELOCITY | vn::protocol::uart::COMMONGROUP_ACCEL | vn::protocol::uart::COMMONGROUP_IMU,
            vn::protocol::uart::TIMEGROUP_NONE,
            vn::protocol::uart::IMUGROUP_NONE,
            vn::protocol::uart::GPSGROUP_NONE,
            vn::protocol::uart::ATTITUDEGROUP_NONE,
            vn::protocol::uart::INSGROUP_NONE,
            vn::protocol::uart::GPSGROUP_NONE)) {
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 1 found");
        borPackage = 1;
    } else if (p.isCompatible(
                   vn::protocol::uart::COMMONGROUP_YAWPITCHROLL | vn::protocol::uart::COMMONGROUP_TIMEGPS | vn::protocol::uart::COMMONGROUP_QUATERNION | vn::protocol::uart::COMMONGROUP_ANGULARRATE | vn::protocol::uart::COMMONGROUP_IMU | vn::protocol::uart::COMMONGROUP_VELOCITY,
                   vn::protocol::uart::TIMEGROUP_NONE,
                   vn::protocol::uart::IMUGROUP_NONE,
                   vn::protocol::uart::GPSGROUP_NONE,
                   vn::protocol::uart::ATTITUDEGROUP_YPRU,
                   vn::protocol::uart::INSGROUP_VELBODY | vn::protocol::uart::INSGROUP_POSLLA | vn::protocol::uart::INSGROUP_INSSTATUS | vn::protocol::uart::INSGROUP_POSU,
                   vn::protocol::uart::GPSGROUP_NONE)) {
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 2 found");
        borPackage = 2;
    } else if (p.isCompatible(
                   vn::protocol::uart::COMMONGROUP_NONE,
                   vn::protocol::uart::TIMEGROUP_NONE,
                   vn::protocol::uart::IMUGROUP_NONE,
                   vn::protocol::uart::GPSGROUP_NUMSATS | vn::protocol::uart::GPSGROUP_FIX | vn::protocol::uart::GPSGROUP_POSLLA | vn::protocol::uart::GPSGROUP_VELNED | vn::protocol::uart::GPSGROUP_POSU | vn::protocol::uart::GPSGROUP_VELU,
                   vn::protocol::uart::ATTITUDEGROUP_NONE,
                   vn::protocol::uart::INSGROUP_NONE,
                   vn::protocol::uart::GPSGROUP_NUMSATS | vn::protocol::uart::GPSGROUP_FIX | vn::protocol::uart::GPSGROUP_POSLLA | vn::protocol::uart::GPSGROUP_VELNED | vn::protocol::uart::GPSGROUP_POSU | vn::protocol::uart::GPSGROUP_VELU)) {
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 3 found");
        borPackage = 3;
    } else {
        RCLCPP_INFO(thisPointer->get_logger(), "Not a maching package type received");
        return;
    }

    if (borPackage == 2) {
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 2 processing");
        // unpacking
        // common group
        uint64_t utcTime = p.extractUint64();           // utc time
        vn::math::vec3f ypr = p.extractVec3f();         // yaw pitch roll
        vn::math::vec4f quat = p.extractVec4f();        // quaternion
        vn::math::vec3f angularRate = p.extractVec3f(); // angular rate
        vn::math::vec3f velocity = p.extractVec3f();    // ned velocity
        vn::math::vec3f accelImu = p.extractVec3f();    // accelerations imu
        vn::math::vec3f gyroImu = p.extractVec3f();     // gyroscope imu
        // attitude group
        vn::math::vec3f ypru = p.extractVec3f();
        // ins group
        uint16_t insStatus = p.extractUint16();          // ins status
        vn::math::vec3d positionsIns = p.extractVec3d(); // ins positions lla
        vn::math::vec3f velocityIns = p.extractVec3f();  // ins velocity body
        float posU = p.extractFloat();                   // ins velocity body
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 2 processing done");

        // publishing
        // utc, euler, quat, imu, ins status
        // odom, ins status, tf
        thisPointer->publishUtc(utcTime);
        thisPointer->publishEuler(ypr[2], ypr[1], ypr[0], ypru[2], ypru[1], ypru[0]);
        thisPointer->publishQuaternion(quat[0], quat[1], quat[2], quat[3]);
        thisPointer->publishImu(accelImu[0], accelImu[1], accelImu[2], gyroImu[0], gyroImu[1], gyroImu[2]);
        thisPointer->publishOdom(positionsIns[0], positionsIns[1], positionsIns[2], velocityIns[0], velocityIns[1], angularRate[2], quat[0], quat[1], quat[2], quat[3], ypru[2], ypru[1], ypru[0], posU);
        thisPointer->publishInsStatus(insStatus);
        if (thisPointer->publishTf) {
            thisPointer->broadcastTf(positionsIns[0], positionsIns[1], positionsIns[2], quat[0], quat[1], quat[2], quat[3]);
        }
    }

    if (borPackage == 3) {
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 3 processing");
        // unpacking
        // gps 1 group
        uint8_t numSats1 = p.extractUint8();                      // number of satelires
        uint8_t fix1 = p.extractUint8();                          // gps fix type
        vn::math::vec3d positions1 = p.extractVec3d();            // gps 1 positions lla
        vn::math::vec3f velocity1 = p.extractVec3f();             // gps 1 velocity ned
        vn::math::vec3f positionsUncertainty1 = p.extractVec3f(); // gps 1 positions uncertainty NED
        float velocityUncertainty1 = p.extractFloat();
        // gps 2 group
        uint8_t numSats2 = p.extractUint8();                      // number of satelires
        uint8_t fix2 = p.extractUint8();                          // gps fix type
        vn::math::vec3d positions2 = p.extractVec3d();            // gps 1 positions lla
        vn::math::vec3f velocity2 = p.extractVec3f();             // gps 1 velocity ned
        vn::math::vec3f positionsUncertainty2 = p.extractVec3f(); // gps 1 positions uncertainty NED
        float velocityUncertainty2 = p.extractFloat();
        RCLCPP_INFO(thisPointer->get_logger(), "Packet type 3 processing done");

        // publishing
        // gps position 1, gps position 2, gps velocity 1, gps velocity 2
        // along with positions publish number of satelites, fix type and position uncertainty
        // along with velocities publish velocity uncertainty
        thisPointer->publishGpsPosition1(numSats1, fix1, positions1[0], positions1[1], positions1[2], positionsUncertainty1[0], positionsUncertainty1[1], positionsUncertainty1[2]);
        thisPointer->publishGpsVelocity1(velocity1[0], velocity1[1], velocity1[2], velocityUncertainty1);
        thisPointer->publishGpsPosition2(numSats2, fix2, positions2[0], positions2[1], positions2[2], positionsUncertainty2[0], positionsUncertainty2[1], positionsUncertainty2[2]);
        thisPointer->publishGpsVelocity2(velocity2[0], velocity2[1], velocity2[2], velocityUncertainty2);
    }
}

void VNPublisher::setupVNCallbacks() {
    // setup BOR if the parameter specifies it, won't be needed when configured from the app
    if (this->configureRegisters) {
        this->setupBinaryOutputRegisters();
    }

    // since the function is static, we have to somehow give it access to the class objects
    void *userData = (void *)this;

    // register the callback
    this->vs.registerAsyncPacketReceivedHandler(userData, VNPublisher::binaryOutputRegisterCallback);
}

void VNPublisher::initializePublishersAndMessages() {
    // initialize reboot service
    this->vnRebootService = this->create_service<std_srvs::srv::Empty>("vn_reboot", std::bind(&VNPublisher::vnRebootDevice, this, std::placeholders::_1, std::placeholders::_2));

    // initialize publishers
    rclcpp::SensorDataQoS sensorQos;
    this->vnUtcPublisher = this->create_publisher<turtle_interfaces::msg::VnUtcTime>("vn_utc_time", sensorQos);
    this->vnEulerPublisher = this->create_publisher<turtle_interfaces::msg::VnEuler>("vn_euler", sensorQos);
    this->vnQuaternionPublisher = this->create_publisher<turtle_interfaces::msg::VnQuat>("vn_quaternion", sensorQos);
    this->vnImuPublisher = this->create_publisher<turtle_interfaces::msg::VnImu>("vn_imu", sensorQos);
    this->vnGpsPosition1Publisher = this->create_publisher<turtle_interfaces::msg::VnGpsPosition>("vn_gps_position_1", sensorQos);
    this->vnGpsPosition2Publisher = this->create_publisher<turtle_interfaces::msg::VnGpsPosition>("vn_gps_position_2", sensorQos);
    this->vnGpsVelocity1Publisher = this->create_publisher<turtle_interfaces::msg::VnGpsVelocity>("vn_gps_velocity_1", sensorQos);
    this->vnGpsVelocity2Publisher = this->create_publisher<turtle_interfaces::msg::VnGpsVelocity>("vn_gps_velocity_2", sensorQos);
    this->vnOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("vn_odom", sensorQos);
    this->vnInsStatusPublisher = this->create_publisher<turtle_interfaces::msg::VnInsStatus>("vn_ins_status", sensorQos);

    // tf broadcaster
    this->tfBroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));
}

void VNPublisher::vnRebootDevice(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    // reset the device
    this->vs.reset();
    RCLCPP_INFO(this->get_logger(), "VN device reset");
}

// utc publisher function
void VNPublisher::publishUtc(uint64_t timestamp) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing UTC");

    // create message
    turtle_interfaces::msg::VnUtcTime msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.time_stamp = timestamp;

    // publish message
    this->vnUtcPublisher->publish(msg);
}

// euler publisher function
void VNPublisher::publishEuler(float roll, float pitch, float yaw, float rollU, float pitchU, float yawU) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing Euler");

    // create message
    turtle_interfaces::msg::VnEuler msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.angle.x = roll;
    msg.angle.y = pitch;
    msg.angle.z = yaw;

    msg.accuracy.x = rollU;
    msg.accuracy.y = pitchU;
    msg.accuracy.z = yawU;

    // publish message
    this->vnEulerPublisher->publish(msg);
}

// quaternion publisher function
void VNPublisher::publishQuaternion(float x, float y, float z, float w) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing Quaternion");

    // create message
    turtle_interfaces::msg::VnQuat msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.quaternion.x = x;
    msg.quaternion.y = y;
    msg.quaternion.z = z;
    msg.quaternion.w = w;

    // publish message
    this->vnQuaternionPublisher->publish(msg);
}

// IMU publisher function
void VNPublisher::publishImu(float ax, float ay, float az, float rx, float ry, float rz) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing Imu");

    // create message
    turtle_interfaces::msg::VnImu msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.accel.x = ax;
    msg.accel.y = ay;
    msg.accel.z = az;
    msg.gyro.x = rx;
    msg.gyro.y = ry;
    msg.gyro.z = rz;

    // publish message
    this->vnImuPublisher->publish(msg);
}

// GPS position 1 publisher function
void VNPublisher::publishGpsPosition1(uint8_t numSats, uint8_t fix, double lon, double lat, double att, double uncertaintyN, double uncertaintyE, double uncertaintyD) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing GPS position 1");

    // create message
    turtle_interfaces::msg::VnGpsPosition msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.position.x = lon;
    msg.position.y = lat;
    msg.position.z = att;

    // set up number of satelites
    msg.num_sats = numSats;

    // fix type
    msg.fix = fix;

    // uncertainty
    msg.positionu.x = uncertaintyN;
    msg.positionu.y = uncertaintyE;
    msg.positionu.z = uncertaintyD;

    // publish message
    this->vnGpsPosition1Publisher->publish(msg);
}

// GPS position 2 publisher function
void VNPublisher::publishGpsPosition2(uint8_t numSats, uint8_t fix, double lon, double lat, double att, double uncertaintyN, double uncertaintyE, double uncertaintyD) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing GPS position 2");

    // create message
    turtle_interfaces::msg::VnGpsPosition msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.position.x = lon;
    msg.position.y = lat;
    msg.position.z = att;

    // set up number of satelites
    msg.num_sats = numSats;

    // fix type
    msg.fix = fix;

    // uncertainty
    msg.positionu.x = uncertaintyN;
    msg.positionu.y = uncertaintyE;
    msg.positionu.z = uncertaintyD;

    // publish message
    this->vnGpsPosition2Publisher->publish(msg);
}

// GPS velocity 1 publisher function
void VNPublisher::publishGpsVelocity1(float veln, float vele, float veld, float uncertainty) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing GPS velocity 1");

    // create message
    turtle_interfaces::msg::VnGpsVelocity msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.vel.x = veln;
    msg.vel.y = vele;
    msg.vel.z = veld;

    // uncertainty
    msg.uncertainty = uncertainty;

    // publish message
    this->vnGpsVelocity1Publisher->publish(msg);
}

// GPS velocity 2 publisher function
void VNPublisher::publishGpsVelocity2(float veln, float vele, float veld, float uncertainty) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing GPS velocity 2");

    // create message
    turtle_interfaces::msg::VnGpsVelocity msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.vel.x = veln;
    msg.vel.y = vele;
    msg.vel.z = veld;

    // uncertainty
    msg.uncertainty = uncertainty;

    // publish message
    this->vnGpsVelocity2Publisher->publish(msg);
}

// odom publisher (EKF body velocities + LLA positions), along with INS status function
void VNPublisher::publishOdom(double lon, double lat, double att, float velx, float vely, float r, float x, float y, float z, float w, float rollU, float pitchU, float yawU, float posU) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing odometry");

    // create message
    nav_msgs::msg::Odometry msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // set up values
    msg.pose.pose.position.z = att;
    msg.twist.twist.linear.x = velx;
    msg.twist.twist.linear.y = vely;
    msg.twist.twist.angular.z = r;

    double geoRad = this->calcGeoRadius(lat);
    msg.pose.pose.position.x = 1000 * geoRad * this->degToRad(lat - this->initialLat);  // x = geoR*(P_LAT - this->rosConf.initialLat)
    msg.pose.pose.position.y = -1000 * geoRad * this->degToRad(lon - this->initialLon); // y = geoR*(P_LON - this->rosConf.initialLon)
    msg.pose.pose.orientation.x = x;
    msg.pose.pose.orientation.y = y;
    msg.pose.pose.orientation.z = z;
    msg.pose.pose.orientation.w = w;

    msg.pose.covariance[0] = posU * posU;
    msg.pose.covariance[7] = posU * posU;
    msg.pose.covariance[21] = rollU * rollU;
    msg.pose.covariance[28] = pitchU * pitchU;
    msg.pose.covariance[35] = yawU * yawU;

    // publish message
    this->vnOdomPublisher->publish(msg);
}

// INS status publisher function
void VNPublisher::publishInsStatus(uint16_t status) {
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing INS status");

    // create message
    turtle_interfaces::msg::VnInsStatus msg;

    // set up header
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // set up values
    msg.status = status;

    // publish message
    this->vnInsStatusPublisher->publish(msg);
}

// tf broadcaster
void VNPublisher::broadcastTf(double lon, double lat, double att, float qx, float qy, float qz, float qw) {
    // create tf message
    geometry_msgs::msg::TransformStamped tfMsg;

    // set up header
    tfMsg.header.stamp = this->now();
    tfMsg.header.frame_id = "odom";
    tfMsg.child_frame_id = "base_link";

    // translation
    tfMsg.transform.translation.x = lon;
    tfMsg.transform.translation.y = lat;
    tfMsg.transform.translation.z = att;

    // rotation
    tfMsg.transform.rotation.x = qx;
    tfMsg.transform.rotation.y = qy;
    tfMsg.transform.rotation.z = qz;
    tfMsg.transform.rotation.w = qw;

    // broadcasting tf message
    this->tfBroadcaster->sendTransform(tfMsg);
}

VNPublisher::~VNPublisher() {
    // unregister the callback
    this->vs.unregisterAsyncPacketReceivedHandler();
    // debug
    RCLCPP_INFO(this->get_logger(), "Disconnecting...");
    // disconnect from sensor
    this->vs.disconnect();
    // debug
    RCLCPP_INFO(this->get_logger(), "Disconnected");
};

double VNPublisher::calcGeoRadius(double lat) {
    // https://en.wikipedia.org/wiki/Earth_radius#Geocentric_radius
    double latRad = this->degToRad(lat);
    double cosf = cos(latRad);
    double sinf = sin(latRad);
    double a2cosf = EQU_R2 * cosf;
    double a2sinf = EQU_R2 * sinf;
    double acosf = EQU_R * cosf;
    double asinf = EQU_R * sinf;
    return sqrt(((a2cosf * a2cosf) + (a2sinf * a2sinf)) / ((acosf * acosf) + (asinf * asinf)));
}

double VNPublisher::degToRad(double deg) {
    return deg * M_PI / 180.0;
}
