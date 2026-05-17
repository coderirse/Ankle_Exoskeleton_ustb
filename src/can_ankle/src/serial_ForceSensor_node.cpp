#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"


serial::Serial ser;
// 实例化串口对象

void signalHandler(int signum)
{
  cout << "close" << endl;
  usleep(4000); // 等待0.004秒 
  ros::shutdown();
}

// 清除字符串中的非数字字符
std::string removeNonNumeric(const std::string& str) {
    std::string result;
    for (char c : str) {
        if (std::isdigit(c) || c == '.' || c == '-') {
            result += c;
        }
    }
    return result;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_ForceSensor_node");
    ros::NodeHandle nh;
    ros::Publisher force_pub = nh.advertise<std_msgs::Float32>("Force", 10);
    ros::Rate loop_rate(200);
    signal(SIGINT, signalHandler);
    // 打开串口
    try
    {
        ser.setPort("/dev/ttyUSB1");  // 设置串口设备路径
        ser.setBaudrate(115200);  // 设置波特率
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);  // 设置超时时间
        ser.setTimeout(timeout);
        ser.setBytesize(serial::eightbits);
        ser.open();  // 打开串口
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("无法打开串口: " << e.what());
        return -1;
    }
    // 判断串口是否成功打开
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized. \n");         // 成功打开串口，打印信息
        std::string receivedData;
        while (ros::ok())
        {
            if (ser.available())
            {
                // 读取所有可用数据
                receivedData += ser.read(ser.available());

                // 查找回车符，拆分数据
                size_t pos = 0;
                while ((pos = receivedData.find('\r')) != string::npos)
                {
                    std::string singleData = receivedData.substr(0, pos);
                    if (singleData.length() >= 6)
                    {
                        try
                        {
                            // 去除多余空格
                            singleData.erase(0, singleData.find_first_not_of(" "));

                            // 将接收到的数据转换为浮点数
                            std::istringstream iss(singleData);
                            float value = 0;
                            if (iss >> value)
                            {
                                value = value * 9.8;
                                // 打印应力数值
                                printf("力传感器数值N:%.3f\n", value);

                                // 发布消息
                                std_msgs::Float32 msg;
                                msg.data = value;
                                force_pub.publish(msg);
                            }
                        }
                        catch (const std::invalid_argument& e)
                        {
                            ROS_ERROR_STREAM("无法将接收到的数据转换为浮点数: " << e.what());
                        }
                    }
                    // 移除已处理的数据
                    receivedData.erase(0, pos + 1);
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        // 关闭串口
        ser.close();
        return 0;
    }
    else
    {
        return -1;
    }
}
