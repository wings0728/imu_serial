#ifndef UTILITY_H

#define UTILITY_H
#include <string>

#define MSG_LEN 	64				//应有的数据长度
#define SOF_DECI 	104				//转换成10进制后的帧开始符

#define GOOD_DATA			0		//好的数据（帧开始符、长度【MSG_LEN】、checksum正确）
#define EXCEED_MSG_LEN 		1		//数据超过应有的长度
#define INCORRECT_SOF 		2		//帧开始符错误
#define INVALID_CHECKSUM	3		//checksum错误

bool isValidSOF(std::string sStartByte);	//检查帧开始符是否正确

bool isValidChecksum(std::string msg);		//检查checksum是否正确

//解析数据，获取9轴信息
bool obtain_imu_data(std::string value, float &roll, float &pitch, float &yaw, float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

#endif
