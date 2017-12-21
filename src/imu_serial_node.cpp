#include "ros/ros.h"
#include <iostream>
#include "serial/serial.h"
#include <string>
#include <stdio.h>
#include "imu_serial/imu_serial.h"
#include "imu_serial/serial_port_handler.h"
#include "imu_serial/utility.h"
#include "imu_serial/MadgwickAHRS.h"

Madgwick filter;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_serial_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ros::Publisher imu_pub = n.advertise<imu_serial::imu_serial>("imu_serial", 10);

	std::string port = "/dev/ttyUSB0";
	int baud_rate = 115200;
	pn.getParam("port", port);
	pn.getParam("baud_rate", baud_rate);

	ros::Rate r(20); 	//20赫兹

	bool ret = initSerial(port, baud_rate); 	//串口初始化
	if (!ret) return 0;							//初始化不成功直接退出
	
	while (ros::ok())
	{
		int status;			//接收到的数据状态，见utility.h
		std::string value;
		bool aRet = read_msg(value, status);	//读取串口上传的数据

		if (aRet) {
			if (status == GOOD_DATA) {		//只有好的数据会进入到下一步，好的数据为“帧开始符正确、长度正确、checksum正确”。里面的信息可能有误
				float roll, pitch, yaw, ax, ay, az, gx, gy ,gz;
				bool bRet = obtain_imu_data(value, roll, pitch, yaw, ax, ay, az, gx, gy, gz);	//从上传的数据解析后得到的9轴信息

				if (!bRet) ROS_WARN("msg: %s", value.c_str());		//解析数据发现问题的话，将不发布信息
				else {
					imu_serial::imu_serial imu_data;
					imu_data.roll = roll;
					imu_data.pitch = pitch;
					imu_data.yaw = yaw;
					imu_data.acc_x = ax;
					imu_data.acc_y = ay;
					imu_data.acc_z = az;
					imu_data.gyro_x = gx;
					imu_data.gyro_y = gy;
					imu_data.gyro_z = gz;

					imu_pub.publish(imu_data);		//发布陀螺仪信息
				}
			}
			else
				ROS_INFO("msg: %s status: %d", value.c_str(), status);		//有问题的数据将不做处理

		}

		ros::spinOnce();
		r.sleep();
	}
	
	
	pn.deleteParam("port");
	pn.deleteParam("baud_rate");
	return 0;
}