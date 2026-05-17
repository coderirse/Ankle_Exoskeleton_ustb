#include<ros/ros.h>
#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"

// 实例化串口对象
serial::Serial ser;
void signalHandler(int signum)
{
  cout << "close" << endl;
  usleep(100000); // 等待0.1秒 
  ros::shutdown();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_encoder_node");
    ros::NodeHandle nh;
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("angle", 100);
    ros::Rate loop_rate(200);
    signal(SIGINT,signalHandler);
    // 打开串口
    try
    {
        ser.setPort("/dev/ttyUSB0");  // 设置串口设备路径
        ser.setBaudrate(9600);  // 设置波特率
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);  // 设置超时时间
        ser.setTimeout(timeout);
        ser.open();  // 打开串口
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("无法打开串口: " << e.what());
        return -1;
    }
    //判断串口是否成功打开
    if( ser.isOpen() )
    {
        while(ros::ok())
        {
            ROS_INFO_STREAM("Serial Port initialized. \n");         //成功打开串口，打印信息
	       /* uint8_t zerodata[8] = {0x01, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC9, 0xC8};
	        ser.write(zerodata,8);//置零
	        cout<<"以当前位置为零点"<<endl;*/
            uint8_t senddata[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
            ser.write(senddata,8); //向串口发送数据
            usleep(1000); // 等待1毫秒
            uint8_t receivedata[7] = {0};
            ser.read(receivedata, 7);
            // 取出有效数据位 将第三位左移动8位，将第四放在高16位上
            uint16_t combined_value = (receivedata[3] << 8) | receivedata[4];
            // 转换数据格式
            double encodervalue = static_cast<double>(combined_value) * 360.0f / 32768.0f;
            if (encodervalue > 200)
            {
                encodervalue = encodervalue - 360;
            }
            // 打印角度
            printf("编码器关节角度: %.2f\n", encodervalue);
            // 循环等待回调函数
            //back_pub.publish(combined_value); 
            std_msgs::Float64 msg;
            msg.data = encodervalue;
            angle_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
        // 关闭串口
        ser.close();
    }    
    else
    {
        return -1;
    }
}


