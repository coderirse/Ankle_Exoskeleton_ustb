#include "can_ankle/msg/torque.hpp"
#include "can_ankle/can_ankle_node.h"
// controlcan.h replaced by can subprocess in header

// 参数宏定义
uint8_t pendingCommand = 0;      // 等待执行的指令
const double RETURN_TORQUE = 0.8;        // 归零扭矩(绝对值)
uint8_t byte0;
uint8_t byte1;
uint8_t byte2;
uint8_t byte3;
bool motor_enabled = false;

//canopen配置指令
class ConfigNode
{
public:
    BYTE para1[8] = {0x23, 0x00, 0x14, 0x01, 0x53, 0x02, 0x00, 0x80};
    BYTE para2[8] = {0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00};
    BYTE para3[8] = {0x23, 0x01, 0x14, 0x01, 0x53, 0x03, 0x00, 0x80};
    BYTE para4[8] = {0x2F, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00};
    BYTE para5[8] = {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para6[8] = {0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60};
    BYTE para7[8] = {0x23, 0x00, 0x16, 0x02, 0x10, 0x00, 0x71, 0x60};
    BYTE para8[8] = {0x23, 0x00, 0x16, 0x03, 0x20, 0x00, 0x7A, 0x60};
    BYTE para9[8] = {0x2F, 0x00, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00};
    BYTE para10[8] = {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para11[8] = {0x23, 0x01, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60};
    BYTE para12[8] = {0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0xFF, 0x60};
    BYTE para13[8] = {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00};
    BYTE para14[8] = {0x23, 0x00, 0x14, 0x01, 0x53, 0x02, 0x00, 0x00};
    BYTE para15[8] = {0x23, 0x01, 0x14, 0x01, 0x53, 0x03, 0x00, 0x00};
    BYTE para16[8] = {0x23, 0x02, 0x14, 0x01, 0x53, 0x04, 0x00, 0x80};
    BYTE para17[8] = {0x23, 0x03, 0x14, 0x01, 0x53, 0x05, 0x00, 0x80};
    BYTE para18[8] = {0x23, 0x00, 0x18, 0x01, 0xD3, 0x01, 0x00, 0xC0};
    BYTE para19[8] = {0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00};
    BYTE para20[8] = {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00};
    BYTE para21[8] = {0x2B, 0x00, 0x18, 0x05, 0x0A, 0x00, 0x00, 0x00};
    BYTE para22[8] = {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para23[8] = {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60};
    BYTE para24[8] = {0x23, 0x00, 0x1A, 0x02, 0x10, 0x00, 0x77, 0x60};
    BYTE para25[8] = {0x23, 0x00, 0x1A, 0x03, 0x20, 0x00, 0x64, 0x60};
    BYTE para26[8] = {0x2F, 0x00, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00};
    BYTE para27[8] = {0x23, 0x02, 0x18, 0x01, 0xD3, 0x03, 0x00, 0xC0};
    BYTE para28[8] = {0x2F, 0x02, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00};
    BYTE para29[8] = {0x2B, 0x02, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00};
    BYTE para30[8] = {0x2B, 0x02, 0x18, 0x05, 0x0A, 0x00, 0x00, 0x00};
    BYTE para31[8] = {0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para32[8] = {0x23, 0x02, 0x1A, 0x01, 0x08, 0x00, 0x61, 0x60};
    BYTE para33[8] = {0x23, 0x02, 0x1A, 0x02, 0x20, 0x00, 0x6C, 0x60};
    BYTE para34[8] = {0x2F, 0x02, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00};
    BYTE para35[8] = {0x23, 0x00, 0x18, 0x01, 0xD3, 0x01, 0x00, 0x40};
    BYTE para36[8] = {0x23, 0x01, 0x18, 0x01, 0xD3, 0x02, 0x00, 0xC0};
    BYTE para37[8] = {0x23, 0x02, 0x18, 0x01, 0xD3, 0x03, 0x00, 0x40};
    BYTE para38[8] = {0x23, 0x03, 0x18, 0x01, 0xD3, 0x04, 0x00, 0xC0};
    BYTE para39[8] = {0x2F, 0xC2, 0x60, 0x01, 0x14, 0x00, 0x00, 0x00};
    BYTE para40[2] = {0x01, 0x00};
    BYTE para41[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para_V[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
    BYTE para_T[5] = {0x04, 0x00, 0x00, 0x00, 0x00};
    BYTE para_P[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    BYTE para_motor1[8] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para_motor2[8] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para_motor3[8] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE para_test[5] = {0x03, 0x00, 0x00, 0x04, 0x00};
};
ConfigNode cfg;

//时间相关函数
class Timer 
{
public:
    Timer() : lastCommandTime(high_resolution_clock::now()) {}

    void startNewTiming() {
        lock_guard<mutex> lock(mutex_);
        lastCommandTime = high_resolution_clock::now();
    }

    double getElapsedTime() {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        return duration_cast<duration<double>>(now - lastCommandTime).count();
    }

private:
    time_point<high_resolution_clock> lastCommandTime;
    mutable mutex mutex_;
};
Timer timer;

void signalHandler(int signum)
{
  //归零完成后关闭
  SendData_five(config_node, 0x00000353, cfg.para_V);
  //SendData(config_node, 0x00000353, cfg.para_P);
  //SendData(config_node, 0x00000353, cfg.para_T);
  usleep(200000);
  cout << "关闭节点" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor2); // 停止电机
  usleep(200000);
  rclcpp::shutdown();
  exit(0);
}

vector<uint8_t> speed_to_command(double speed) 
{
    // 将°/s转换为rpm
    double rpm = speed / 6.0;
    
    // 计算原始数值（32位有符号整数）
    // 0x00010000 (65536) 对应 0.15rpm，因此系数为 65536 / 0.15
    int32_t raw_value = static_cast<int32_t>(rpm * (65536.0 / 0.15));
    
    // 小端模式拆分四位字节（低位在前）
    byte0 = static_cast<uint8_t>(raw_value & 0xFF);
    byte1 = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
    byte2 = static_cast<uint8_t>((raw_value >> 16) & 0xFF);
    byte3 = static_cast<uint8_t>((raw_value >> 24) & 0xFF);
    
    return {byte0, byte1, byte2, byte3};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ankle_node");
  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info", 10); //发布各项信息

  //初始化can节点
  Init_Can();
  sendMSG();
  motor_on_V();
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));  // 200Hz
  signal(SIGINT, signalHandler);
  steady_clock::time_point start_time;
  //发送指令
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
    velocity_output[0] = 0x03;
    velocity_output[1] = 0x00;
    velocity_output[2] = 0x00;
    velocity_output[3] = 0x00;
    velocity_output[4] = 0x00;
    if (motor_enabled == true)
    {
      timer.startNewTiming();
      motor_enabled = false;
    }
    double y = 0.0;
    double elapsed = timer.getElapsedTime();
    cout << "经过的时间" << elapsed << endl;

    y = 5 * sin(1 * M_PI * elapsed); //单位是度/s

    speed_to_command(y);

    //发布消息
    velocity_value = y;
    can_ankle::msg::Torque torque_msg;
    torque_msg.velocity_value = velocity_value;
    torque_msg.return_velocity = Output_VelocityValue;
    torque_msg.return_torque_value = Output_TorqueValue;
    torque_pub->publish(torque_msg);
    //发送速度指令
    velocity_output[1] = byte0;
    velocity_output[2] = byte1;
    velocity_output[3] = byte2;
    velocity_output[4] = byte3;
    BYTE velocity_data[5] = {velocity_output[0],velocity_output[1],velocity_output[2],velocity_output[3],velocity_output[4]};
    cout<<"发出的指令："<<"["<< hex << setw(2) << setfill('0') << velocity_output[0] <<"],["<< hex << setw(2) << setfill('0')<< velocity_output[1] <<"],["<< hex << setw(2) << setfill('0')<< velocity_output[2] <<"],["<< hex << setw(2) << setfill('0')<< velocity_output[3] <<"],["<< hex << setw(2) << setfill('0')<< velocity_output[4] <<"]"<< endl;
    SendData_five(send_motor_torque, 0x00000353, velocity_data);
  }
}

void sendMSG()
{
    SendData(config_node, 0x00000653, cfg.para1);
    cout << "指令1发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para2);
    cout << "指令2发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para3);
    cout << "指令3发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para4);
    cout << "指令4发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para5);
    cout << "指令5发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para6);
    cout << "指令6发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para7);
    cout << "指令7发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para8);
    cout << "指令8发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para9);
    cout << "指令9发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para10);
    cout << "指令10发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para11);
    cout << "指令11发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para12);
    cout << "指令12发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para13);
    cout << "指令13发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para14);
    cout << "指令14发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para15);
    cout << "指令15发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para16);
    cout << "指令16发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para17);
    cout << "指令17发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para18);
    cout << "指令18发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para19);
    cout << "指令19发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para20);
    cout << "指令20发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para21);
    cout << "指令21发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para22);
    cout << "指令22发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para23);
    cout << "指令23发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para24);
    cout << "指令24发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para25);
    cout << "指令25发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para26);
    cout << "指令26发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para27);
    cout << "指令27发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para28);
    cout << "指令28发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para29);
    cout << "指令29发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para30);
    cout << "指令30发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para31);
    cout << "指令31发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para32);
    cout << "指令32发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para33);
    cout << "指令33发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para34);
    cout << "指令34发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para35);
    cout << "指令35发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para36);
    cout << "指令36发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para37);
    cout << "指令37发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para38);
    cout << "指令38发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000653, cfg.para39);
    cout << "指令39发送完毕" << endl;
    usleep(200000);
    SendData(config_node, 0x00000000, cfg.para40);
    cout << "指令40发送完毕" << endl;
}

void motor_on_V()
{
  SendData_five(config_node, 0x00000353, cfg.para_V);
  cout << "速度模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3);
  usleep(200000);
  cout << "使能完毕" << endl;
  motor_enabled = true;
}

void motor_on_P()
{
  SendData_five(config_node, 0x00000353, cfg.para_P);
  cout << "位置模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3);
  usleep(200000);
  cout << "使能完毕" << endl;
  motor_enabled = true;
}

void motor_on_T()
{
  SendData_five(config_node, 0x00000353, cfg.para_T);
  cout << "扭矩模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2);
  usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3);
  usleep(200000);
  cout << "使能完毕" << endl;
  motor_enabled = true;
}
