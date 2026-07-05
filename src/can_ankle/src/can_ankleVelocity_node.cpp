#include "can_ankle/msg/torque.hpp"
#include "can_ankle/can_ankle_node.h"
// controlcan.h replaced by can subprocess in header

// 参数宏定义
double currentY = 0.0; // 用于存储当前的y值
bool isDriving = false; // 用于标记是否在驱动状态
bool isForward = true; // 用于标记是否正转
bool isSwitchDelay = false; // 标记是否处于转换延时阶段
bool isWaitingForward = false;  //  标记是否在等待正转条件
double x = 0.0; //总时间
double y = 0.0; //速度驱动函数
double Encoder_Value = 0; //编码器角度值
uint8_t pendingCommand = 0;      // 等待执行的指令
double initialEncoderValue = 0;       // 初始编码器位置
std::atomic<bool> isReturningHome{false}; // 归零状态标志
const double RETURN_TORQUE = 1.0;        // 归零扭矩(绝对值)
const int ENCODER_SAFE_LIMIT = 45;      // 编码器安全阈值
const double  ENCODER_FORWARD = 8;  //电机正转编码器限定值
const double DRIVE_DURATION = 1.25; // 正弦半周期持续时间 1.25秒 (π/(0.8π) = 1.25)
const double DELAY_DURATION = 0.2; //延时0.2s
time_point<high_resolution_clock> startTime; // 节点开始运行的时间
time_point<high_resolution_clock> delayStartTime;   // 延时开始时间

class Timer {
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
  cout << "启动归零程序..." << endl;
  isReturningHome = true;
  
  // 发送归零速度指令
  double return_torque = (initialEncoderValue > Encoder_Value) ? RETURN_TORQUE : -RETURN_TORQUE;
  uint16_t TransformValue = (return_torque - T_min)/((T_max - T_min)/4096);
  
  BYTE return_data[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x07, 0xFF,static_cast<BYTE>((TransformValue >> 8) & 0xFF), static_cast<BYTE>(TransformValue & 0xFF)};
  
  // 持续发送直到到达初始位置
  while(abs(Encoder_Value - initialEncoderValue) > 1 && rclcpp::ok()) 
  {
    SendData(send_motor_torque, 0x00000001, return_data);
    usleep(50000); // 50ms周期
  }

  // 归零完成后关闭
  cout << "归零完成，关闭节点" << endl;
  SendData(send_motor_torque, 0x00000001, config_motor3); // 停止电机
  usleep(200000);
  rclcpp::shutdown();
  exit(0);
}

// 指令回调函数，处理接收到的指令消息
void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t command = msg->data;
    switch (command) {
        case 0x41:
            if (isDriving && !isForward) 
            {  // 反转中收到正转指令
                pendingCommand = 0x41;
            } 
            else if (!isDriving && !isSwitchDelay) 
            {
                // 启动前检查编码器值
                if (Encoder_Value > ENCODER_FORWARD) 
                {
                    isDriving = true;
                    isForward = true;
                    timer.startNewTiming();
                } 
                else 
                {
                    isWaitingForward = true;
                    RCLCPP_WARN(rclcpp::get_logger("ankle"), "Encoder value %d ≤8°, waiting for >8° to start forward.", Encoder_Value);
                }
            } 
            else if (isSwitchDelay) 
            {
                pendingCommand = 0x41;  // 允许延时期间接收指令
            }
            break;

        case 0x42:
             if (isDriving && isForward) 
             {  // 正转中收到反转指令
                pendingCommand = 0x42;
            } 
            else if (!isDriving && !isSwitchDelay) 
            {
                isDriving = true;
                isForward = false;
                timer.startNewTiming();
            } 
            else if (isSwitchDelay) 
            {
                pendingCommand = 0x42;  // 允许延时期间接收指令
            }
            break;
    }
}

void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double receive_angle = msg->data;
    
    // 角度规范化
    if(receive_angle > 300) receive_angle -= 360;
    Encoder_Value = receive_angle;

    // 安全限制检查
    if(abs(Encoder_Value) > ENCODER_SAFE_LIMIT && !isReturningHome) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("ankle"), "Encoder value %d exceeds safety limit! Triggering emergency return.", Encoder_Value);
        signalHandler(SIGINT); // 触发归零程序
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ankle_node");
  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info", 10); //发布各项信息

  //初始化can节点
  Init_Can();
  //记录起始编码器的数值
  usleep(500000); // 等待500ms确保编码器数据稳定
  initialEncoderValue = Encoder_Value;
  RCLCPP_INFO(rclcpp::get_logger("ankle"), "Initial encoder position: %d", initialEncoderValue);
  //设置当前时间为起始时间
  auto start_time = chrono::high_resolution_clock::now();
  //输入kp、kd值并换算成指令
  //cout<<"请输入Kp:"<<endl;
  //cin>> kp;
  cout<<"请输入Kd:"<<endl;
  cin>>kd;
  //double r_kp=8.192*kp;
  double r_kd = 819.2*kd;
  int intr_kd = static_cast<int>(r_kd);
  //int intr_kp = static_cast<int>(r_kp);
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));  // 200Hz
  signal(SIGINT, signalHandler);
  //发送指令
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
    output[0]=0x7F;
    output[1]=0xFF;
    output[2]=0x7F;
    output[3]=0xF0;
    output[4]=0x00;
    output[5]=0x00;
    output[6]=0x07;
    output[7]=0xFF;
    /*cout<<"请输入电机的扭矩值："<<endl;
    cin>>torque_value;
    if (torque_value>T_max || torque_value<T_min)
    {  
        cout<<"输入有误"<<endl;
    }*/

    double y = 0.0;

    // 状态机核心
    if (isDriving) 
    {
        if (isForward) 
        {
            double elapsed = timer.getElapsedTime();
            y = 5 * sin(0.8 * M_PI * elapsed);
            if (elapsed >= DRIVE_DURATION) 
            {
                isDriving = false;
                y = 0;
                if (pendingCommand != 0) 
                {
                    isSwitchDelay = true;
                    delayStartTime = high_resolution_clock::now();
                } 
                else 
                {
                    isForward = false;  // 重置方向状态
                }
            }
        } 
        else 
        {
            double elapsed = timer.getElapsedTime();
            y = -5 * sin(0.8 * M_PI * elapsed);
            if (elapsed >= DRIVE_DURATION) 
            {
                isDriving = false;
                y = 0;
                if (pendingCommand != 0) 
                {
                    isSwitchDelay = true;
                    delayStartTime = high_resolution_clock::now();
                }
            }
        }
    }

    else if (isWaitingForward) 
    {  //等待编码器条件
        if (Encoder_Value > ENCODER_FORWARD) 
        {
            // 条件满足，启动正转
            isWaitingForward = false;
            isDriving = true;
            isForward = true;
            timer.startNewTiming();
            RCLCPP_INFO(rclcpp::get_logger("ankle"), "编码器值已超过8°,开始正转");
        } 
        else 
        {
            // 持续等待
            RCLCPP_INFO(rclcpp::get_logger("ankle"), "等待编码器值超过8°,当前值: %d", Encoder_Value);
        }
    }

    else if (isSwitchDelay) 
    {
        auto now = high_resolution_clock::now();
        duration<double> elapsed = now - delayStartTime;
        if (elapsed.count() >= DELAY_DURATION) 
        {
            isSwitchDelay = false;
            isDriving = true;
            isForward = (pendingCommand == 0x41);  // 切换方向
            pendingCommand = 0;                    // 清除标记
            timer.startNewTiming();
        }
    }

    cout<<fixed<<setprecision(3)<<"at time"<<x<<"seconds,y="<<y<<endl;
    //发布消息
    velocity_value = y;
    can_ankle::msg::Torque torque_msg;
    torque_msg.velocity_value = velocity_value;
    torque_msg.return_velocity = Output_VelocityValue;
    torque_msg.return_torque_value = Output_TorqueValue;
    torque_pub->publish(torque_msg);
    //把驱动函数换算成指令
    uint16_t TransformValue = (velocity_value-V_min)/((V_max-V_min)/4096);
    cout<<hex<<TransformValue<<endl;
    cout<<hex<<intr_kd<<endl;
    //cout<<hex<<intr_kp<<endl;
    //发送速度指令
    output[2] = static_cast<uint16_t>((TransformValue >> 4) & 0xFF);
    output[3] &= 0x0F;
    uint8_t lowFourBits = static_cast<uint16_t>((TransformValue) & 0xFF);
    lowFourBits =  lowFourBits << 4;
    output[3] |=  lowFourBits;
    //发送kd指令
    output[5] = static_cast<uint16_t>((intr_kd >> 4) & 0xFF);
    output[6] = static_cast<uint16_t>((intr_kd & 0x0F) << 4 | 0x07);
    //发送can指令
    BYTE velocity_data[8] = {output[0],output[1],output[2],output[3],output[4],output[5],output[6],output[7]};
    cout<<"发出的指令："<<"["<<output[0]<<"],["<<output[1]<<"],["<<output[2]<<"],["<<output[3]<<"],["<<output[4]<<"],["<<output[5]<<"],["<<output[6]<<"],["<<output[7]<<"]"<<endl;
    SendData(send_motor_torque, 0x00000001, velocity_data);
  }
}
