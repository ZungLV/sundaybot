#include "sundaybot_bringup/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <iostream>

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::readImuValues(int &val_1,  int &val_2,  int &val_3,  int &val_4,  int &val_5,  int &val_6,  int &val_7,  int &val_8,  int &val_9,  int &val_10)
{
    std::string response = sendMsg("i\r");
    std::vector<int> values;
    std::string delimiter = " ";
    splitString(response,values);
    val_1 = values[0];
    val_2 = values[1];
    val_3 = values[2];
    val_4 = values[3];
    val_5 = values[4];
    val_6 = values[5];
    val_7 = values[6];
    val_8 = values[7];
    val_9 = values[8];
    val_10 = values[9];     
}

void ArduinoComms::readQValues(int &val_1,  int &val_2,  int &val_3,  int &val_4)
{
    std::string response = sendMsg("q\r");
    std::vector<int> values;
    std::string delimiter = " ";
    splitString(response,values);
    val_1 = values[0];
    val_2 = values[1];
    val_3 = values[2];
    val_4 = values[3];  
}
void ArduinoComms::readLValues(int &val_1,  int &val_2,  int &val_3)
{
    std::string response = sendMsg("l\r");
    std::vector<int> values;
    std::string delimiter = " ";
    splitString(response,values);
    val_1 = values[0];
    val_2 = values[1];
    val_3 = values[2];
}
void ArduinoComms::readGValues(int &val_1,  int &val_2,  int &val_3)
{
    std::string response = sendMsg("g\r");
    std::vector<int> values;
    std::string delimiter = " ";
    splitString(response,values);
    val_1 = values[0];
    val_2 = values[1];
    val_3 = values[2];
}

void ArduinoComms::splitString(const std::string& s, std::vector<int>& values) {
    std::istringstream iss(s);
    std::string token;

    // Clear the vector to ensure it's empty before adding new values
    values.clear();

    while (std::getline(iss, token, ' ')) {
        values.push_back(std::stoi(token));
    }
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}