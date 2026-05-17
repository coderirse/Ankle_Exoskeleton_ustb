#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"
#include <iomanip>

serial::Serial ser;
chrono::steady_clock::time_point phase_start_time;
chrono::steady_clock::time_point support_start_time;

// 添加状态变量
uint8_t prev_command = 0x00;
bool waiting_for_start = false;
bool valid_command_received = false;  // 新增：标记是否收到有效指令
uint8_t current_valid_command = 0x00; // 新增：当前有效指令

void signalHandler(int signum) 
{
    cout << "Closing..." << endl;
    usleep(100000);
    ros::shutdown();
}

void publishPhaseTime(ros::Publisher& pub, const string& phase_name, double duration) 
{
    std_msgs::Float64 msg;
    
    msg.data = duration;
    pub.publish(msg);
    ROS_INFO("%s time: %.9f seconds", phase_name.c_str(), duration);
}

// 添加重置函数
void resetToStart() {
    prev_command = 0x00;
    waiting_for_start = true;
    valid_command_received = false;  // 重置有效指令标记
    ROS_WARN("Sequence broken, waiting for next 0x41 to restart");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "serial_sendCommand_node");
    ros::NodeHandle n;
    auto resolution = chrono::steady_clock::period::num / static_cast<double>(chrono::steady_clock::period::den);
    cout << "system clock resolution:"<< resolution << "seconds" << endl;
    
    // 创建多个发布者
    ros::Publisher command_pub = n.advertise<std_msgs::UInt8>("command_topic", 10);
    ros::Publisher one_support_pub = n.advertise<std_msgs::Float64>("one_support_time", 10);
    ros::Publisher two_support_pub = n.advertise<std_msgs::Float64>("two_support_time", 10);
    ros::Publisher three_support_pub = n.advertise<std_msgs::Float64>("three_support_time", 10);
    ros::Publisher swing_pub = n.advertise<std_msgs::Float64>("swing_time", 10);
    ros::Publisher support_pub = n.advertise<std_msgs::Float64>("support_time", 10);

    ros::Rate loop_rate(5000);
    signal(SIGINT, signalHandler);

    try 
    {
        ser.setPort("/dev/ttyUSB2");
        ser.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);  // 设置超时时间
        ser.setTimeout(timeout);
        ser.open();
    } 
    catch (serial::IOException& e) 
    {
        ROS_ERROR_STREAM("Port open failed: " << e.what());
        return -1;
    }

    if (ser.isOpen()) 
    {
        ROS_INFO_STREAM("Serial port initialized");
        uint8_t byte;

        while (ros::ok()) 
        {
            if (ser.available()) 
            {
                ser.read(&byte, 1);
                
                // 特殊指令0x45单独处理，不参与顺序检查
                if(byte == 0x45)
                { 
                    ROS_WARN("Received special command :E");
                    // 仍然发布特殊指令
                    std_msgs::UInt8 cmd_msg;
                    cmd_msg.data = byte;
                    command_pub.publish(cmd_msg);
                    continue;
                }
                
                // 处理0x41指令（起始指令）
                if(byte == 0x41)
                {
                    // 0x41总是可以开始新的循环
                    waiting_for_start = false;
                    prev_command = 0x41;
                    valid_command_received = true;  // 标记为有效指令
                    current_valid_command = byte;
                    
                    cout << "接收指令："<< byte <<endl;
                    auto now = chrono::steady_clock::now();
                    chrono::duration<double> swing_duration = now - phase_start_time;
                    publishPhaseTime(swing_pub, "Swing Phase", swing_duration.count());
                    phase_start_time = now;
                    support_start_time = now;
                    cout << "摆动相时间：" <<std::fixed<<std::setprecision(9)<< swing_duration.count() <<endl;
                    ROS_INFO("FULL_SUPPORT started");
                }
                // 处理0x42指令（需要前一个指令是0x41）
                else if(byte == 0x42)
                {
                    if(prev_command == 0x41 && !waiting_for_start)
                    {
                        prev_command = 0x42;
                        valid_command_received = true;  // 标记为有效指令
                        current_valid_command = byte;
                        
                        cout << "接收指令："<< byte <<endl;
                        auto now = chrono::steady_clock::now();
                        chrono::duration<double> onesupport_duration = now - phase_start_time;
                        publishPhaseTime(one_support_pub, "Support Phase One", onesupport_duration.count());
                        phase_start_time = now;
                        ROS_INFO("TENSION_TORQUE started");
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                // 处理0x43指令（需要前一个指令是0x42）
                else if(byte == 0x43)
                {
                    if(prev_command == 0x42 && !waiting_for_start)
                    {
                        prev_command = 0x43;
                        valid_command_received = true;  // 标记为有效指令
                        current_valid_command = byte;
                        
                        cout << "接收指令："<< byte <<endl;
                        auto now = chrono::steady_clock::now();
                        chrono::duration<double> twosupport_duration =now - phase_start_time;
                        publishPhaseTime(two_support_pub, "Support Phase Two", twosupport_duration.count());
                        phase_start_time = now;
                        ROS_INFO("TORQUE_CURVE started");
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                // 处理0x44指令（需要前一个指令是0x43）
                else if(byte == 0x44)
                {
                    if(prev_command == 0x43 && !waiting_for_start)
                    {
                        prev_command = 0x44;
                        valid_command_received = true;  // 标记为有效指令
                        current_valid_command = byte;
                        
                        cout << "接收指令："<< byte <<endl;
                        auto now = chrono::steady_clock::now();
                        chrono::duration<double> threesupport_duration = now - phase_start_time;
                        publishPhaseTime(three_support_pub, "Support Phase Three", threesupport_duration.count());
                        phase_start_time = now;
                        ROS_INFO("SWING_PHASE started");
                        chrono::duration<double> support_duration = now - support_start_time;
                        publishPhaseTime(support_pub, "support Phase", support_duration.count());
                        cout << "支撑相时间："<<std::fixed<<std::setprecision(9) << support_duration.count() <<endl;
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                else
                {
                    ROS_WARN("Unknown command: 0x%02X", byte);
                }
                
                // 在条件判断外部统一发布有效指令
                if (valid_command_received)
                {
                    std_msgs::UInt8 cmd_msg;
                    cmd_msg.data = current_valid_command;
                    command_pub.publish(cmd_msg);
                    valid_command_received = false;  // 重置标记
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}