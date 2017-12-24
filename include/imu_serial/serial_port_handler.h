#ifndef SERIAL_PORT_HANDLER_H
#define SERIAL_PORT_HANDLER_H

#include <string>
#include <vector>
#include "serial/serial.h"
#include "ros/ros.h"
#include "imu_serial/utility.h"

bool initSerial(std::string port, int baud);		//设置串口、打开串口

bool read_msg(std::string &value, int& status);		//从串口获取信息

#endif
