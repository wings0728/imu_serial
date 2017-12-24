#include "ros/ros.h"
#include <iostream>
#include "serial/serial.h"
#include <string>
#include <stdio.h>
#include "imu_serial/imu_serial.h"
#include "imu_serial/serial_port_handler.h"
#include "imu_serial/utility.h"
#include "imu_serial/MadgwickAHRS.h"
#include <sys/time.h>
#include <sensor_msgs/Imu.h>

Madgwick filter;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_serial_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	sensor_msgs::Imu imu_msg;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu1", 10);

	std::string port = "/dev/ttyUSB0";
	int baud_rate = 115200;
	pn.getParam("port", port);
	pn.getParam("baud_rate", baud_rate);

	ros::Rate r(200); 	//20赫兹
	
	bool ret = initSerial(port, baud_rate); 	//串口初始化
	if (!ret) return 0;							//初始化不成功直接退出
	struct timeval currentTime, lastTime;
	
	//gettimeofday(&currentTime, NULL);
        gettimeofday(&lastTime, NULL);
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
					// imu_serial::imu_serial imu_data;
					// imu_data.roll = roll;
					// imu_data.pitch = pitch;
					// imu_data.yaw = yaw;
					// imu_data.acc_x = ax;
					// imu_data.acc_y = ay;
					// imu_data.acc_z = az;
					// imu_data.gyro_x = gx;
					// imu_data.gyro_y = gy;
					// imu_data.gyro_z = gz;
					gettimeofday(&currentTime, NULL);
					double csec = (double)currentTime.tv_sec + ((double)currentTime.tv_usec)/1000000.0;
					double lsec = (double)lastTime.tv_sec + ((double)lastTime.tv_usec)/1000000.0;
					double freq = (csec - lsec);
					filter.invSampleFreq = freq;
					lastTime.tv_sec =  currentTime.tv_sec;
					lastTime.tv_usec = currentTime.tv_usec;
					filter.updateIMU(gx, gy, gz, ax, ay, az);

					imu_msg.header.stamp    = ros::Time::now();
					imu_msg.header.frame_id = "imu1_link";

					imu_msg.angular_velocity.x = gx;
					imu_msg.angular_velocity.y = gy;
					imu_msg.angular_velocity.z = gz;
					imu_msg.angular_velocity_covariance[0] = 0.02;
					imu_msg.angular_velocity_covariance[1] = 0;
					imu_msg.angular_velocity_covariance[2] = 0;
					imu_msg.angular_velocity_covariance[3] = 0;
					imu_msg.angular_velocity_covariance[4] = 0.02;
					imu_msg.angular_velocity_covariance[5] = 0;
					imu_msg.angular_velocity_covariance[6] = 0;
					imu_msg.angular_velocity_covariance[7] = 0;
					imu_msg.angular_velocity_covariance[8] = 0.02;

					//  imu_msg.linear_acceleration.x = imu.ax;
					imu_msg.linear_acceleration.x = ax;
					imu_msg.linear_acceleration.y = ay;
					imu_msg.linear_acceleration.z = az;
					imu_msg.linear_acceleration_covariance[0] = 0.04;
					imu_msg.linear_acceleration_covariance[1] = 0;
					imu_msg.linear_acceleration_covariance[2] = 0;
					imu_msg.linear_acceleration_covariance[3] = 0;
					imu_msg.linear_acceleration_covariance[4] = 0.04;
					imu_msg.linear_acceleration_covariance[5] = 0;
					imu_msg.linear_acceleration_covariance[6] = 0;
					imu_msg.linear_acceleration_covariance[7] = 0;
					imu_msg.linear_acceleration_covariance[8] = 0.04;

					imu_msg.orientation.w = filter.q0;
					imu_msg.orientation.x = filter.q1;
					imu_msg.orientation.y = filter.q2;
					imu_msg.orientation.z = filter.q3;

					//  imu_msg.orientation.x = imu.mx;
					//  imu_msg.orientation.y = imu.my;
					//  imu_msg.orientation.z = imu.mz;

					imu_msg.orientation_covariance[0] = filter.getYaw();
					imu_msg.orientation_covariance[1] = 0;
					imu_msg.orientation_covariance[2] = 0;
					imu_msg.orientation_covariance[3] = 0;
					imu_msg.orientation_covariance[4] = 0.0025;
					imu_msg.orientation_covariance[5] = 0;
					imu_msg.orientation_covariance[6] = 0;
					imu_msg.orientation_covariance[7] = 0;
					imu_msg.orientation_covariance[8] = 0.0025;
					//gettimeofday(&lastTime, NULL);
					//ROS_INFO("filter: %f f: %f",filter.getYaw(), freq);
					imu_pub.publish(imu_msg);		//发布陀螺仪信息

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
