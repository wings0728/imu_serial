#include "imu_serial/utility.h"
#include <cstdlib>
#include "ros/ros.h"
#include <sstream>


bool isValidSOF(std::string sStartByte)     //检查帧开始符是否正确(0x68)
{
    unsigned long check_first_byte = std::strtoul(sStartByte.c_str(), 0, 16);   //把16进制的字符串转换成10进制
    return check_first_byte == SOF_DECI;    //如果是 104， 将返回 true (HEX 0x68 == DEC 104)
}

bool isValidChecksum(std::string msg)       //检查checksum是否正确
{
    /* 把上传数据里对应的“信息长度、地址、内容”数据相加，转换成16进制，取得最低位字节，再与checksum作比较 */

    unsigned long msg_length = std::strtoul(msg.substr(2,2).c_str(), 0, 16);
    unsigned long addr = std::strtoul(msg.substr(4,2).c_str(), 0, 16);
    unsigned long cmd = std::strtoul(msg.substr(6,2).c_str(), 0, 16);
    unsigned long roll = std::strtoul(msg.substr(8,6).c_str(), 0, 16);
    unsigned long pitch = std::strtoul(msg.substr(14,6).c_str(), 0, 16);
    unsigned long yaw = std::strtoul(msg.substr(20,6).c_str(), 0, 16);
    unsigned long acc_x = std::strtoul(msg.substr(26,6).c_str(), 0, 16);
    unsigned long acc_y = std::strtoul(msg.substr(32,6).c_str(), 0, 16);
    unsigned long acc_z = std::strtoul(msg.substr(38,6).c_str(), 0, 16);
    unsigned long gyro_x = std::strtoul(msg.substr(44,6).c_str(), 0, 16);
    unsigned long gyro_y = std::strtoul(msg.substr(50,6).c_str(), 0, 16);
    unsigned long gyro_z = std::strtoul(msg.substr(56,6).c_str(), 0, 16);

    unsigned long sum = msg_length + addr + cmd + roll + pitch + yaw + acc_x +acc_y + acc_z + gyro_x + gyro_y + gyro_z;     // 把数据相加
    std::stringstream csum;
    csum << std::hex << sum;    //转换成16进制
    int cs_size = csum.str().size();
    unsigned long checksum_lsb = std::strtoul(csum.str().substr(cs_size - 2).c_str(), 0, 16);
    unsigned long checksum = std::strtoul(msg.substr(62).c_str(), 0 ,16);

    return checksum_lsb == checksum;    //结果与checksum一样的话，返回true
}

bool obtain_imu_data(std::string value, float &roll, float &pitch, float &yaw, float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    // 从上传的字符串里取得对应的陀螺仪数据

    std::string roll_str = value.substr(8, 6);
    std::string pitch_str = value.substr(14, 6);
    std::string yaw_str = value.substr(20, 6);
    std::string ax_str = value.substr(26, 6);
    std::string ay_str = value.substr(32, 6);
    std::string az_str = value.substr(38, 6);
    std::string gx_str = value.substr(44, 6);
    std::string gy_str = value.substr(50, 6);
    std::string gz_str = value.substr(56, 6);


    // 检查数据第一位是否为0或1,0代表正数，1代表负数，其他数字将报错

    if (std::atoi(roll_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("roll has an invalid data: %s First digit should be 0 or 1", roll_str.c_str());
        return false;
    }
    if (std::atoi(pitch_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("pitch has an invalid data: %s First digit should be 0 or 1", pitch_str.c_str());
        return false;
    }
    if (std::atoi(yaw_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("yaw has an invalid data: %s First digit should be 0 or 1", yaw_str.c_str());
        return false;
    }
    if (std::atoi(ax_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("acc_x has an invalid data: %s First digit should be 0 or 1", ax_str.c_str());
        return false;
    }
    if (std::atoi(ay_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("acc_y has an invalid data: %s First digit should be 0 or 1", ay_str.c_str());
        return false;
    }
    if (std::atoi(az_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("acc_z has an invalid data: %s First digit should be 0 or 1", az_str.c_str());
        return false;
    }
    if (std::atoi(gx_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("gyro_x has an invalid data: %s First digit should be 0 or 1", gx_str.c_str());
        return false;
    }
    if (std::atoi(gy_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("gyro_y has an invalid data: %s First digit should be 0 or 1", gy_str.c_str());
        return false;
    }
    if (std::atoi(gz_str.substr(0,1).c_str()) > 1) {
        ROS_WARN("gyro_z has an invalid data: %s First digit should be 0 or 1", gz_str.c_str());
        return false;
    }

    //把取得的字符串解析成正确的格式
    
    roll = std::atoi(roll_str.substr(1,3).c_str()) + std::atoi(roll_str.substr(4).c_str()) / 100.0;
    roll = (std::atoi(roll_str.substr(0,1).c_str()) == 1)? -roll: roll;

    pitch = std::atoi(pitch_str.substr(1,3).c_str()) + std::atoi(pitch_str.substr(4).c_str()) / 100.0;
    pitch = (std::atoi(pitch_str.substr(0,1).c_str()) == 1)? -pitch: pitch;

    yaw = std::atoi(yaw_str.substr(1,3).c_str()) + std::atoi(yaw_str.substr(4).c_str()) / 100.0;
    yaw = (std::atoi(yaw_str.substr(0,1).c_str()) == 1)? -yaw: yaw;

    ax = std::atoi(ax_str.substr(1,2).c_str()) + std::atoi(ax_str.substr(3).c_str()) / 1000.0;
    ax = (std::atoi(ax_str.substr(0,1).c_str()) == 1)? -ax: ax;

    ay = std::atoi(ay_str.substr(1,2).c_str()) + std::atoi(ay_str.substr(3).c_str()) / 1000.0;
    ay = (std::atoi(ay_str.substr(0,1).c_str()) == 1)? -ay: ay;

    az = std::atoi(az_str.substr(1,2).c_str()) + std::atoi(az_str.substr(3).c_str()) / 1000.0;
    az = (std::atoi(az_str.substr(0,1).c_str()) == 1)? -az: az;

    gx = std::atoi(gx_str.substr(1,3).c_str()) + std::atoi(gx_str.substr(4).c_str()) / 100.0;
    gx = (std::atoi(gx_str.substr(0,1).c_str()) == 1)? -gx: gx;

    gy = std::atoi(gy_str.substr(1,3).c_str()) + std::atoi(gy_str.substr(4).c_str()) / 100.0;
    gy = (std::atoi(gy_str.substr(0,1).c_str()) == 1)? -gy: gy;

    gz = std::atoi(gz_str.substr(1,3).c_str()) + std::atoi(gz_str.substr(4).c_str()) / 100.0;
    gz = (std::atoi(gz_str.substr(0,1).c_str()) == 1)? -gz: gz;

    return true;
}