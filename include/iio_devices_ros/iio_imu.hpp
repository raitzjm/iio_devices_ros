#ifndef IIO_IMU_HPP
#define IIO_IMU_HPP
#include <iio.h>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IIO_IMU: public rclcpp::Node 
{

public:
IIO_IMU();
~IIO_IMU();
long long accel_data_x, accel_data_y, accel_data_z;
long long gyro_data_x, gyro_data_y, gyro_data_z;
struct iio_channel *accel_x;
struct iio_channel *accel_y;
struct iio_channel *accel_z;

struct iio_device *accel_dev;
struct iio_device *gyro_dev;
struct iio_channel *gyro_x;
struct iio_channel *gyro_y;
struct iio_channel *gyro_z;

private:
void timer_callback(void);



rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
rclcpp::TimerBase::SharedPtr sampling_timer_;
struct iio_context *ctx;

//Parameters

double sampling_frequency_;
   
void initialize_params(void);
int initialize_iio_channels(void);
std::unordered_map<std::string, struct iio_channel*> accel_channels;
std::unordered_map<std::string, struct iio_channel*> gyro_channels;
struct iio_buffer* accel_buffer;
struct iio_buffer * gyro_buffer;
std::string imu_frame_id_;

};







#endif // IIO_IMU_HPP