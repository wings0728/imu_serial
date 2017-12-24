#include "imu_serial/serial_port_handler.h"


serial::Serial my_serial;
#define bufferLength 5
#define readBuffLength 32
unsigned char buffer[bufferLength] = {0x68, 0x04, 0x00, 0x04, 0x08};
//std::string readBuffer;//[readBuffLength] = {0};

bool openSerial()
{
	if (my_serial.isOpen())			//检查串口是否已打开
		return true;
	try {
		my_serial.open();			//如果还未打开，将尝试打开
		return my_serial.isOpen();
	}
	catch (serial::IOException& e) {
		ROS_ERROR("%s", e.what());
		return false;
	}
	catch (serial::SerialException& e) {
		ROS_ERROR("%s", e.what());
		return false;
	}

	
}

bool initSerial(std::string port, int baud)
{
	my_serial.setPort(port.c_str());
	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	my_serial.setTimeout(timeout);
	my_serial.setBaudrate(baud);

	if (!openSerial()) {
		ROS_WARN("Unable to open port: %s", port.c_str());
		return false;
	}
	else {
		ROS_INFO("Opened port: %s", port.c_str());
		return true;
	}

}

bool read_msg(std::string &value, int &status)
{
	if (!openSerial()) {												//无法打开串口将返回
		ROS_WARN("Serial port not opened. Try to open again ..");
		return false;
	}
	
/*
	while(true)
	{	
		bool wRet = my_serial.write(buffer, bufferLength);
		ROS_INFO("write result is : %d", wRet);	
		//my_serial.read(head);
		size_t rRet = my_serial.read(readBuffer, readBuffLength);
		ROS_INFO("rRet: %s" ,readBuffer);

		for(unsigned char i = 0; i < readBuffLength; i++)
		{
			ROS_INFO("Received head: %x", readBuffer[i]);
		}

	}*/
	
	bool wRet = my_serial.write(buffer, bufferLength);

	//uint8_t readBuffer[readBuffLength] = {0};
	//ROS_INFO("write result is : %d", wRet);
	//std::vector<uint8_t> readBuffer;
	unsigned char readBuffer[readBuffLength] = {'0'};	
	size_t n_size = my_serial.read(readBuffer, readBuffLength);
	//ROS_INFO("write result is : %ld", n_size);
	//value = new string(readBuffLength * 2 , readBuffer);

	for(int idx = 0; idx < readBuffLength; idx++)
	{
	//ROS_INFO("rRet: %c %d" ,((readBuffer[idx]>>4)+0x30), idx);
	//ROS_INFO("rRet: %c %d" ,((readBuffer[idx]&0x0f)+0x30), idx);
	value += ((readBuffer[idx]>>4)+0x30);
	value += ((readBuffer[idx]&0x0f)+0x30);	
	}
	//ROS_INFO("%s", value.c_str());
	
	
//	size_t n_size = my_serial.readline(value);
	if (n_size == 0) {												//读取空的数据将返回
		ROS_INFO("Received an empty message from port");
		return false;
	}

//	 value = "681F008410502300600011600000100010120500052510502301500010900013";

	bool examine_checksum = false;

	/* 获得串口上传的数据后，将坐几项检查：
		1. 数据是否比应有的数据较长（32bytes == 64characters， 见utility.h）。是的话，这次数据不取
		2. 数据的帧开始符是否正确。不正确的话，这次数据不取
		3. 数据长度是否等于应有数据的长度。是的话，进入第二步检查，见下
		4. 如果数据拥有正确的帧开始符，但是长度比应有的短，将再次从串口读取数据，叫做数据（二）
			a. 如果数据（二）的长度与应有的长度一样，同时拥有正确的帧开始符，丢弃原有数据，直接取数据（二），进入第二步检查，见下
			b. 否则，原有数据将和数据（二）组成新的数据字符串。如果这个新的数据拥有应有的长度，进入第二步检查，见下
			c. 新数据如果长度较长或较短，返回第一步检查1.

		第二步检查：
		进入这一步的数据有正确的帧开始符和长度，这个检查将检查数据是否拥有正确的checksum
		checksum错误，数据不取
		checksum正确，解析数据然后发布 
	*/
	//ROS_WARN("msg: %ld", value.length());

	while (true) {
		if (value.length() > MSG_LEN) {									//数据长度比应有的较长
			ROS_INFO("msg has more that %d bytes of data", MSG_LEN);
			status = EXCEED_MSG_LEN;
			examine_checksum = false;
			break;
		}

		if (!isValidSOF(value.substr(0,2))) {							//帧开始符错误
			ROS_INFO("msg does not have the correct SOF (0x68)");
			status = INCORRECT_SOF;
			examine_checksum = false;
			break;
		}

		if (value.length() == MSG_LEN) {								//数据长度正确
			examine_checksum = true;									//进入第二步检查
			break;
		}

		std::string result_tmp;
		size_t n_size_tmp = my_serial.readline(result_tmp);				//如果数据有正确的帧开始符，但长度较短，将再次从串口读取数据

		if (n_size_tmp == MSG_LEN && isValidSOF(result_tmp.substr(0,2))) {		//如果数据（二）长度和帧开始符正确， 进入第二步检查
			ROS_INFO("Discarded a msg of size %ld with valid SOF but invalid length: %s", value.length(), value.c_str());
			examine_checksum = true;
			value = result_tmp;					//丢弃原有数据，直取数据（二）
			break;
		}

		value = value + result_tmp;				//原有数据和数据（二）组合成新数据

		if (value.length() == MSG_LEN) {		//新数据长度正确
			examine_checksum = true;			//进入第二步检查
			break;
		}
	}

	if (examine_checksum) {						//第二步检查
		if (isValidChecksum(value))
			status = GOOD_DATA;
		else {
			//ROS_INFO("msg has an invalid checksum");
			//status = INVALID_CHECKSUM;
			status = GOOD_DATA;
		}
	}

	return true;
}
