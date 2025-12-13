/**
 * DS4 IMU Publisher Node
 * Reads gyroscope and accelerometer data from DualShock 4 controller via evdev
 * and publishes to /ds4/imu topic
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#include <string>

class DS4ImuPublisher : public rclcpp::Node {
public:
    DS4ImuPublisher() : Node("ds4_imu_publisher"), fd_(-1) {
        this->declare_parameter<std::string>("device", "");
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ds4/imu", 10);
        
        std::string device = this->get_parameter("device").as_string();
        
        if (device.empty()) {
            device = findDS4Device();
        }
        
        if (device.empty()) {
            RCLCPP_WARN(this->get_logger(), "DS4 controller not found. IMU data unavailable.");
            RCLCPP_INFO(this->get_logger(), "Connect DS4 via USB or Bluetooth and restart.");
        } else {
            openDevice(device);
        }
        
        // Timer to read IMU data at 100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DS4ImuPublisher::readAndPublish, this));
    }
    
    ~DS4ImuPublisher() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    std::string findDS4Device() {
        DIR* dir = opendir("/dev/input");
        if (!dir) return "";
        
        struct dirent* entry;
        std::string fallback_device;
        
        while ((entry = readdir(dir)) != nullptr) {
            if (strncmp(entry->d_name, "event", 5) != 0) continue;
            
            std::string path = "/dev/input/" + std::string(entry->d_name);
            int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;
            
            char name[256] = {0};
            if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
                std::string devName(name);
                RCLCPP_DEBUG(this->get_logger(), "Checking device: %s (%s)", path.c_str(), name);
                
                // Prefer "Motion Sensors" device specifically
                if (devName.find("Motion Sensors") != std::string::npos) {
                    close(fd);
                    closedir(dir);
                    RCLCPP_INFO(this->get_logger(), "Found DS4 Motion Sensors: %s (%s)", 
                                path.c_str(), name);
                    return path;
                }
                // Keep Sony device as fallback
                if (devName.find("Sony") != std::string::npos && fallback_device.empty()) {
                    fallback_device = path;
                }
            }
            close(fd);
        }
        closedir(dir);
        
        if (!fallback_device.empty()) {
            RCLCPP_WARN(this->get_logger(), "Motion Sensors not found, using fallback: %s", 
                        fallback_device.c_str());
        }
        return fallback_device;
    }
    
    void openDevice(const std::string& device) {
        fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", 
                         device.c_str(), strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Opened DS4 IMU device: %s", device.c_str());
    }
    
    void readAndPublish() {
        if (fd_ < 0) {
            // Try to find device periodically
            static int retry_count = 0;
            if (++retry_count >= 500) {  // Every 5 seconds
                retry_count = 0;
                std::string device = findDS4Device();
                if (!device.empty()) {
                    openDevice(device);
                }
            }
            return;
        }
        
        struct input_event ev;
        bool updated = false;
        
        // Read all available events
        while (read(fd_, &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type == EV_ABS) {
                updated = true;
                switch (ev.code) {
                    // Accelerometer (ABS_X, ABS_Y, ABS_Z or similar)
                    case ABS_X:
                        accel_x_ = ev.value / 8192.0;  // Normalize
                        break;
                    case ABS_Y:
                        accel_y_ = ev.value / 8192.0;
                        break;
                    case ABS_Z:
                        accel_z_ = ev.value / 8192.0;
                        break;
                    // Gyroscope (ABS_RX, ABS_RY, ABS_RZ)
                    case ABS_RX:
                        gyro_x_ = ev.value / 1024.0;  // rad/s approximately
                        break;
                    case ABS_RY:
                        gyro_y_ = ev.value / 1024.0;
                        break;
                    case ABS_RZ:
                        gyro_z_ = ev.value / 1024.0;
                        break;
                }
            }
        }
        
        // Always publish at timer rate (even if no new events)
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "ds4_imu";
        
        msg.angular_velocity.x = gyro_x_;
        msg.angular_velocity.y = gyro_y_;
        msg.angular_velocity.z = gyro_z_;
        
        msg.linear_acceleration.x = accel_x_;
        msg.linear_acceleration.y = accel_y_;
        msg.linear_acceleration.z = accel_z_;
        
        // Orientation not provided
        msg.orientation_covariance[0] = -1;
        
        imu_pub_->publish(msg);
        
        // Debug logging every second
        static int log_counter = 0;
        if (++log_counter >= 100) {
            log_counter = 0;
            RCLCPP_DEBUG(this->get_logger(), "Gyro: x=%.3f y=%.3f z=%.3f", gyro_x_, gyro_y_, gyro_z_);
        }
    }
    
    int fd_;
    double accel_x_ = 0, accel_y_ = 0, accel_z_ = 0;
    double gyro_x_ = 0, gyro_y_ = 0, gyro_z_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DS4ImuPublisher>());
    rclcpp::shutdown();
    return 0;
}
