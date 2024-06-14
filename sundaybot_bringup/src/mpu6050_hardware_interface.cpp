#include "sundaybot_bringup/mpu6050_hardware_interface.hpp"

namespace sundaybot_bringup
{

// Initialize MPU6050 device
// MPU6050 device(0x68); 

MPU6050Hardware::MPU6050Hardware()
	: logger_(rclcpp::get_logger("MPU6050Hardware"))
{}

CallbackReturn MPU6050Hardware::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
	{
		return CallbackReturn::ERROR;
	}

	RCLCPP_INFO(logger_, "Initializing...");

	cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
	cfg_.device = info_.hardware_parameters["device"];
  	cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  	cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

	// Set up the Arduino
  	arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

	RCLCPP_INFO(logger_, "Finished initialization");
	
	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MPU6050Hardware::export_state_interfaces()
{
	// Set up the MPU6050 state interfaces
	
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &orientation.x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &orientation.y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &orientation.z));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &orientation.w));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &angular_vel_x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &angular_vel_y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &angular_vel_z));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &linear_accel_x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_accel_y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &linear_accel_z));

	return state_interfaces;
}

CallbackReturn MPU6050Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(logger_, "Starting controller ...");

	return CallbackReturn::SUCCESS;
}

CallbackReturn MPU6050Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(logger_, "Stopping Controller...");
	
	return CallbackReturn::SUCCESS;
}

return_type MPU6050Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	// // Obtain current euler angles
	// device.getAngle(0, &euler_angles[0]);
	// device.getAngle(1, &euler_angles[1]);
	// device.getAngle(2, &euler_angles[2]);

	// // Obtain current quaternion from euler angles
	// quat = device.getQuat(&euler_angles[0], &euler_angles[1], &euler_angles[2]);

	// // Obtain current gyroscope and accelerometer values
	// device.getGyro(&gyro_values[0], &gyro_values[1], &gyro_values[2]);
	// device.getAccel(&accel_values[0], &accel_values[1], &accel_values[2]); 

	// Assign values to the state interfaces
	// Orientation, angular velocity and linear acceleration conform the East North Up (ENU) coordinate frame
	// convention (https://www.ros.org/reps/rep-0103.html) required by the robot_localization package

	arduino_.readQValues(quatx, quaty, quatz, quatw);
	arduino_.readLValues(accelx, accely, accelz);
	arduino_.readGValues(gyrox, gyroy, gyroz);

	orientation.x = (double)quatx/100000;
	orientation.y = (double)quaty/100000;
	orientation.z = (double)quatz/100000;
	orientation.w = (double)quatw/100000;
	angular_vel_x = (double)gyrox/100000;
	angular_vel_y = (double)gyroy/100000;	
	angular_vel_z = (double)gyroz/100000;	
	linear_accel_x = (double)accelx/100000;
	linear_accel_y = (double)accely/100000;
	linear_accel_z = (double)accelz/100000;

	// orientation.x = quat.y;
	// orientation.y = quat.x;
	// orientation.z = quat.z;
	// orientation.w = quat.w;
	// angular_vel_x = (double)gyro_values[1];	
	// angular_vel_y = (double)gyro_values[0];	
	// angular_vel_z = (double)gyro_values[2];	
	// linear_accel_x = 0;//(double)accel_values[1];
	// linear_accel_y = 0;//(double)accel_values[0];
	// linear_accel_z = (double)accel_values[2];

	return return_type::OK;
}

} // namespace sundaybot_bringup

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	sundaybot_bringup::MPU6050Hardware,
	hardware_interface::SensorInterface)
