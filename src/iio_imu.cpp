#include <cstdio>


#include <iostream>
#include "iio_devices_ros/iio_imu.hpp"
#include <chrono>


IIO_IMU::IIO_IMU() : Node("imu_publisher")
{
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    this->initialize_params();


    // Initialize your devices and channels here

    // Create IIO context
    ctx = iio_create_default_context();
    if (!ctx) {
        std::cerr << "Unable to create IIO context.\n";
       // return 1;
    }


    // Find the IIO device
    accel_dev = iio_context_find_device(ctx, "lsm6dsr_accel");
    if (!accel_dev) {
        std::cerr << "Unable to find IIO device.\n";
       // return 1;
    }


    this->get_parameter("sampling_frequency", sampling_frequency_);
    int ret = iio_device_attr_write_double(accel_dev, "sampling_frequency", sampling_frequency_);
    if (ret < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to set accel frequency to: " << sampling_frequency_);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Accel frequency set to: " << sampling_frequency_);
    }


    //Initialize gyro

    gyro_dev = iio_context_find_device(ctx, "lsm6dsr_gyro");
    if (!gyro_dev) {
        std::cerr << "Unable to find IIO device.\n";
        //return 1;
    }

    this->initialize_iio_channels();


    ret = iio_device_attr_write_double(gyro_dev, "sampling_frequency", sampling_frequency_);
    if (ret < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to set gyro frequency to: " << sampling_frequency_);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Gyro frequency set to: " << sampling_frequency_);
    }

    
    this->accel_buffer = iio_device_create_buffer(accel_dev, 1, false);
    if(accel_buffer == nullptr)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to create accel buffer");
    }
    
    
    this->gyro_buffer = iio_device_create_buffer(gyro_dev, 1, false);
    if(gyro_buffer == nullptr)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to create gyro buffer");
    }


    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / sampling_frequency_));

    sampling_timer_ = this->create_wall_timer(
        period,  // Adjust the rate as needed
        std::bind(&IIO_IMU::timer_callback, this));

}


void IIO_IMU::initialize_params()
{
    // Get parameters from the parameter server
    this->declare_parameter("sampling_frequency", 208.0);
    this->get_parameter("sampling_frequency", sampling_frequency_);
    this->declare_parameter("imu_frame_id", "imu_link");
    this->get_parameter("imu_frame_id", imu_frame_id_);
}

int IIO_IMU::initialize_iio_channels()
{
    this->accel_channels["accel_x"] = nullptr;
    this->accel_channels["accel_y"] = nullptr;
    this->accel_channels["accel_z"] = nullptr;
    this->gyro_channels["anglvel_x"] = nullptr;
    this->gyro_channels["anglvel_y"] = nullptr;
    this->gyro_channels["anglvel_z"] = nullptr;

    for(auto &[name,channel] : this->accel_channels)
    {
        accel_channels[name]= iio_device_find_channel(accel_dev, name.c_str(), false);
        if(!accel_channels[name])
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to find channel: " << name);
            return 0;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Found channel: " << name);
            iio_channel_enable(accel_channels[name]);
        }
    }

    for(auto &[name,channel] : this->gyro_channels)
    {
        gyro_channels[name]= iio_device_find_channel(gyro_dev, name.c_str(), false);
        if(!gyro_channels[name])
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to find channel: " << name);
            return  0;
        }
        else
        {  
            RCLCPP_INFO_STREAM(this->get_logger(), "Found channel: " << name);
            iio_channel_enable(gyro_channels[name]);
        }
    }

    this->accel_x = accel_channels["accel_x"];
    this->accel_y = accel_channels["accel_y"];
    this->accel_z = accel_channels["accel_z"];
    this->gyro_x = gyro_channels["anglvel_x"];
    this->gyro_y = gyro_channels["anglvel_y"];
    this->gyro_z = gyro_channels["anglvel_z"];




    return 1;
}

void IIO_IMU::timer_callback()
{


    // Read data from the IMU
    if (iio_buffer_refill(accel_buffer) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error refilling accel buffer.\n");
        //return 1;
    }

    if (iio_buffer_refill(gyro_buffer) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error refilling gyro buffer.\n");
        //return 1;
    }

    void* accel_x_ptr = iio_buffer_first(accel_buffer, accel_x);
    void* accel_y_ptr = iio_buffer_first(accel_buffer, accel_y);
    void* accel_z_ptr = iio_buffer_first(accel_buffer, accel_z);

    void* gyro_x_ptr = iio_buffer_first(gyro_buffer, gyro_x);
    void* gyro_y_ptr = iio_buffer_first(gyro_buffer, gyro_y);
    void* gyro_z_ptr = iio_buffer_first(gyro_buffer, gyro_z);

    accel_data_x = *static_cast<long long*>(accel_x_ptr);
    accel_data_y = *static_cast<long long*>(accel_y_ptr);
    accel_data_z = *static_cast<long long*>(accel_z_ptr);

    gyro_data_x = *static_cast<long long*>(gyro_x_ptr);
    gyro_data_y = *static_cast<long long*>(gyro_y_ptr);
    gyro_data_z = *static_cast<long long*>(gyro_z_ptr);




    auto imu_msg = sensor_msgs::msg::Imu();
    double accel_scale=0.000598205;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation_covariance = { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

    imu_msg.linear_acceleration.x = accel_data_x * accel_scale;
    imu_msg.linear_acceleration.y = accel_data_y * accel_scale;
    imu_msg.linear_acceleration.z = accel_data_z * accel_scale;

    double gyro_scale = 0.000152716;
    imu_msg.angular_velocity.x = gyro_data_x * gyro_scale;
    imu_msg.angular_velocity.y = gyro_data_y * gyro_scale;
    imu_msg.angular_velocity.z = gyro_data_z * gyro_scale;

    imu_publisher_->publish(imu_msg);
    
}

IIO_IMU::~IIO_IMU()
{
    iio_buffer_destroy(accel_buffer);
    iio_buffer_destroy(gyro_buffer);
    iio_context_destroy(ctx);
    
}


int main(int argc, char **argv) 
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IIO_IMU>());
    rclcpp::shutdown();
    return 0;
    
}