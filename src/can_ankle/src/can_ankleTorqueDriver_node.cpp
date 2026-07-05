#include "can_ankle/msg/torque.hpp"

#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"

// 参数宏定义
uint8_t pendingCommand = 0;      // 等待执行的指令
const double RETURN_TORQUE = 0.8;        // 归零扭矩(绝对值)

// using std::clamp from <algorithm>

//状态变量
class state_parameter
{
public:
    bool isForward = false; // 用于标记是否正转
    bool need_homing = false; // 归零状态标志
    bool initialEncoderRecorded = false;//起始位置记录
    bool initialMotorRecorded = false;//支撑相起始位置记录
};
state_parameter ST;

//编码器参数
class encoder_parameter
{
public:
    double Encoder_Value = 0;     //编码器角度值
    double initialEncoderValue = 0;       // 初始编码器位置
    double position_angle = 0;    //转换位置控制时的关节角度
    const int ENCODER_SAFE_LIMIT = 45;      // 编码器安全阈值
    double initialMotorPosition_one = NAN;  // 单数次支撑相开始时的位置
    double initialMotorPosition_two = NAN;  // 双数次支撑相开始时的位置
    double initialMotorPosition = 0;//支撑相开始时位置
    int initialMotorPosition_counter = 0;   // 支撑相计数器
    double SwingMotorPosition = 0;     //摆动相电机位置
};
encoder_parameter Enc;

//支撑相扭矩驱动参数信息
class Torque_parameter
{
public:
    const double GAIT_PERIOD = 2.2;//步态周期时间
    double DRIVE_DURATION = GAIT_PERIOD * 0.64; // 驱动持续时间(支撑相时间)
    double DRIVE_DELAY_DURATION = 0.26 * DRIVE_DURATION; // 延时时间
    double TARGET_TORQUE_RISE_TIME = 0.23 * DRIVE_DURATION; // 上升时间
    double TARGET_TORQUE_PEAK_TIME = 0.49 * DRIVE_DURATION; // 峰值时间
    double TARGET_TORQUE_FALL_TIME = 0.12 * DRIVE_DURATION; // 下降时间
    double TARGET_TORQUE_PEAK = 15.0 +CompensateTorque; // 峰值扭矩
    double PRE_TORQUE = 1 + CompensateTorque*0.2; // 预张紧力
    double CompensateTorque = 0;//补偿扭矩
};
Torque_parameter Tor;

//驱动时间相关
class Timer 
{
public:
    Timer() : lastCommandTime(high_resolution_clock::now()), torqueStartTime(high_resolution_clock::now()) {}

    void startNewTiming() 
    {
        lock_guard<mutex> lock(mutex_);
        lastCommandTime = high_resolution_clock::now();
    }

    double getElapsedTime() 
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        return duration_cast<duration<double>>(now - lastCommandTime).count();
    }

    void startTorqueTiming() 
    {
        lock_guard<mutex> lock(mutex_);
        torqueStartTime = high_resolution_clock::now();
    }

    double getTorqueElapsedTime() 
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        return duration_cast<duration<double>>(now - torqueStartTime).count();
    }

    double getDt() 
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        dt = duration_cast<duration<double>>(now - lastDtTime).count();
        lastDtTime = now;
        if (dt <= 1e-6) dt = 1e-6;
        return dt;
    }
    
private:
    time_point<high_resolution_clock> lastDtTime;
    double dt = 0.0;
    time_point<high_resolution_clock> lastCommandTime;
    time_point<high_resolution_clock> torqueStartTime;
    mutable mutex mutex_;
};
Timer timer;

class IMU_parameter
{
    public:
        double imu_orientation_w = 0; // 存储imu方向，四元数
        double imu_orientation_x = 0;
        double imu_orientation_y = 0;
        double imu_orientation_z = 0;
        double imu_AngelVelocity_x = 0;//x轴角速度
        double imu_AngelVelocity_y = 0;//y轴角速度
        double imu_AngelVelocity_z = 0;//z轴角速度
        double imu_LinerAcceleration_x = 0;//x轴线加速度
        double imu_LinerAcceleration_y = 0;//y轴线加速度
        double imu_LinerAcceleration_z = 0;//z轴线加速度

    };
IMU_parameter IMU;
//状态机模式
enum DriveMode {VELOCITY_MODE, TORQUE_MODE, STAND_MODE};
DriveMode currentDriveMode = STAND_MODE;

// PD跟踪控制器（扭矩模式）
struct PDController 
{
    double Kp; //扭矩误差比例增益
    double Kd; //速度阻尼增益
    double prev_error{0.0};
    const double max_output = V_max;   // 输出限幅

    PDController(double kp,  double kd) : Kp(kp) , Kd(kd) {}

    double compute(double setpoint, double measurement, double dt) 
    {
        if (dt <= 1e-6) dt = 1e-6;
        
        double error = setpoint - measurement;
        double derivative = Output_VelocityValue;
        double output = Kp * error  - Kd * derivative;
        output = std::clamp(output, -max_output, max_output); // 输出限幅
        
        prev_error = error;
        return output;
    }
};

// PD参数
PDController pd(0.4, 0.005); 
 
//比例+阻尼跟踪控制器（位置模式）
struct PositionController 
{
    double K_theta;  // 位置比例增益
    double K_d;      // 速度阻尼增益
    double prev_theta_m; // 上一时刻的测量位置
    time_point<high_resolution_clock> prev_time; // 上一时刻的时间戳

    PositionController(double kt, double kd) : K_theta(kt), K_d(kd), prev_theta_m(0), prev_time(high_resolution_clock::now()) {}
};

// 初始化位置参数
PositionController pos_controller(0.1, 0.0005); 

//退出程序
void signalHandler(int signum) 
{
    cout << "启动归零程序..." << endl;
    ST.need_homing = true; // 设置标志位
}

// 退出归零函数
void processHoming() 
{
    bool is_homing = false;
    if (ST.need_homing && !is_homing) 
    {
        is_homing = true;
        // 发送归零指令
        double return_torque = (Enc.initialEncoderValue > Enc.Encoder_Value) ? RETURN_TORQUE : -RETURN_TORQUE;
        uint16_t TransformValue = (return_torque - T_min)/((T_max - T_min)/4096);
        BYTE return_data[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x07, 0xFF,static_cast<BYTE>((TransformValue >> 8) & 0xFF), static_cast<BYTE>(TransformValue & 0xFF)};
        SendData(send_motor_torque, 0x00000001, return_data);
        usleep(50000);
        while(abs(Enc.Encoder_Value - Enc.initialEncoderValue) < 3 && rclcpp::ok()) 
        {   // 关闭流程
            SendData(send_motor_torque, 0x00000001, config_motor3);
            usleep(200000);
            SendData(send_motor_torque, 0x00000001, config_motor2);
            usleep(200000);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            rclcpp::shutdown();
            SendData(send_motor_torque, 0x00000001, return_data);
            usleep(50000);
        }
    }
}

//编码器回调函数，接收关节角度信息
void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double receive_angle = msg->data;
    // 角度规范化
    if(receive_angle > 300) receive_angle -= 360;
    Enc.Encoder_Value = receive_angle;
    //记录初始位置
    if(!ST.initialEncoderRecorded)
    {
        Enc.initialEncoderValue = Enc.Encoder_Value;
        RCLCPP_INFO(rclcpp::get_logger("ankle"), "Initial encoder position: %.2f\n", Enc.initialEncoderValue);
        ST.initialEncoderRecorded = true;
    }
    // 安全限制检查
    if(abs(Enc.Encoder_Value) > Enc.ENCODER_SAFE_LIMIT) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("ankle"), "Encoder value %.2f exceeds safety limit! Triggering emergency return.", Enc.Encoder_Value);
        signalHandler(SIGINT); // 触发归零程序
    }
    Enc.position_angle = Enc.Encoder_Value;
}

//力传感器回调函数，接收力传感器返回值
void torqueCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float forcevalue = msg->data;
    SensorTorque = forcevalue * 0.018 ;
}

//imu数据存储函数
void extractImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    IMU.imu_orientation_w = msg->orientation.w;
    IMU.imu_orientation_x = msg->orientation.x;
    IMU.imu_orientation_y = msg->orientation.y;
    IMU.imu_orientation_z = msg->orientation.z;

    IMU.imu_AngelVelocity_x = msg->angular_velocity.x;
    IMU.imu_AngelVelocity_y = msg->angular_velocity.y;
    IMU.imu_AngelVelocity_z = msg->angular_velocity.z;

    IMU.imu_LinerAcceleration_x = msg->linear_acceleration.x;
    IMU.imu_LinerAcceleration_y = msg->linear_acceleration.y;
    IMU.imu_LinerAcceleration_z = msg->linear_acceleration.z;
}

//imu回调函数
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    extractImuData(msg);
    RCLCPP_INFO(rclcpp::get_logger("ankle"), "IMU Data Received");
}

//补偿扭矩输出函数
void updateCompensateTorque(encoder_parameter& Enc , Torque_parameter&  Tor) 
{
    if (isnan(Enc.initialMotorPosition_one))return;
    if (isnan(Enc.initialMotorPosition_two)) return;

    // 根据计数器判断比较规则
    bool even_cycle = (Enc.initialMotorPosition_counter / 2) % 2 == 0;
    double a = Enc.initialMotorPosition_two - Enc.initialMotorPosition_one;
    double b = Enc.initialMotorPosition_one - Enc.initialMotorPosition_two;
    if(abs(a) > 10)
    {
        a = 10;
    }
    else if(abs(a) < -10)
    {
        a = -10;
    }
    if(abs(b) > 10)
    {
        b = 10;
    }
    else if(abs(b) < -10)
    {
        b = -10;
    }
    if (even_cycle) 
    {//偶数次
        Tor.CompensateTorque = a * -0.8;
    } 
    else 
    {//奇数次
        Tor.CompensateTorque = b * -0.8;
    }

    RCLCPP_INFO(rclcpp::get_logger("ankle"), "Compensation torque set to: %f", Tor.CompensateTorque);
}

//发送扭矩指令函数
void sendTorqueCommand(double y) 
{
    torque_value = y;
    //double dt = timer.getDt(); // 从Timer获取dt
    //adjusted_torque = pd.compute(y, SensorTorque, dt);
    uint16_t transformValue = (torque_value - T_min) / (T_max - T_min) * 4096;
    BYTE torque_data[8] = 
    {
        0x7F, 0xFF, 0x7F, 0xF0, 
        0x00, 0x00, 
        static_cast<BYTE>((transformValue >> 8) & 0xFF), 
        static_cast<BYTE>(transformValue & 0xFF)
    };
    cout << "发出的指令：" ;
    for (int i = 0; i < 8; i++)
    {
        cout << "[" << hex << static_cast<int>(torque_data[i]) << "]";
    }
    cout << endl;
    SendData(send_motor_torque, 0x00000001, torque_data);
}

//发送扭矩转换速度指令函数
void sendTorTransVelCommand(double y) 
{
    torque_value = y;
    double dt = timer.getDt(); // 从Timer获取dt
    adjusted_velocity = pd.compute(torque_value, SensorTorque, dt); //PD+阻抗
    velocity_value = adjusted_velocity;//发送的速度值
    uint16_t transformValue = (adjusted_velocity - V_min) / (V_max - V_min) * 4096;
    double r_kd = 819.2 * kd;
    int intr_kd = static_cast<int>(r_kd);
    // 发送速度指令
    output[2] = static_cast<uint16_t>((transformValue >> 4) & 0xFF);
    output[3] &= 0x0F;
    uint8_t lowFourBits = static_cast<uint16_t>((transformValue) & 0xFF);
    lowFourBits = lowFourBits << 4;
    output[3] |= lowFourBits;

    // 发送kd指令
    output[5] = static_cast<uint16_t>((intr_kd >> 4) & 0xFF);
    output[6] = static_cast<uint16_t>((intr_kd & 0x0F) << 4 | 0x07);

    // 发送CAN指令
    BYTE velocity_data[8] = {output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7]};
    cout << "发出的指令：" << "[" << hex << output[0] << "],["<< hex  << output[1] << "],[" << hex << output[2] << "],[" << hex << output[3] << "],[" << hex << output[4] << "],[" << hex << output[5] << "],[" << hex << output[6] << "],[" << hex << output[7] << "]" << endl;
    SendData(send_motor_torque, 0x00000001, velocity_data);
}

//发送速度指令函数
void sendVelocityCommand(double y, double kd) 
{
    // 把驱动函数换算成指令
    velocity_value = y;
    uint16_t TransformValue = (velocity_value - V_min) / ((V_max - V_min) / 4096);
    double r_kd = 819.2 * kd;
    int intr_kd = static_cast<int>(r_kd);

    // 发送速度指令
    output[2] = static_cast<uint16_t>((TransformValue >> 4) & 0xFF);
    output[3] &= 0x0F;
    uint8_t lowFourBits = static_cast<uint16_t>((TransformValue) & 0xFF);
    lowFourBits = lowFourBits << 4;
    output[3] |= lowFourBits;

    // 发送kd指令
    output[5] = static_cast<uint16_t>((intr_kd >> 4) & 0xFF);
    output[6] = static_cast<uint16_t>((intr_kd & 0x0F) << 4 | 0x07);

    // 发送CAN指令
    BYTE velocity_data[8] = {output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7]};
    cout << "发出的指令：" << "[" << hex << output[0] << "],["<< hex  << output[1] << "],[" << hex << output[2] << "],[" << hex << output[3] << "],[" << hex << output[4] << "],[" << hex << output[5] << "],[" << hex << output[6] << "],[" << hex << output[7] << "]" << endl;
    SendData(send_motor_torque, 0x00000001, velocity_data);
}

// 指令回调函数，处理接收到的状态指令
void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg) 
{
    uint8_t command = msg->data;
    switch (command) 
    {
        case 0x41:
            currentDriveMode = TORQUE_MODE;
            Enc.initialMotorPosition_counter++;
            timer.startNewTiming();
        break;
        case 0x43:
            if(ST.isForward)
            {
                pendingCommand = 0x43;
            }
            else
            {
                currentDriveMode = VELOCITY_MODE;
            }
        break;
    }
}

//主函数（can通讯/状态机/发送消息）
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nh");
  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info", 10); //发布各项信息
  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>("command_topic", 10, commandCallback); // 订阅开关指令
  auto encoder_sub = node->create_subscription<std_msgs::msg::Float64>("angle", 10, encoderCallback); //订阅关节角度值
  auto forcesensor_sub = node->create_subscription<std_msgs::msg::Float32>("Force", 10, torqueCallback); //订阅力传感器值
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imuCallback); //订阅imu传感器值

  //设置机械零位
  //SendData(send_motor_torque, 0x00000001, config_motor4);

  //初始化can节点
  Init_Can();

  //输入kp、kd值并换算成指令
  //cout<<"请输入Kp:"<<endl;
  //cin>> kp;
  cout<<"请输入Kd:"<<endl;
  cin>>kd;
  //double r_kp=8.192*kp;
  //int intr_kp = static_cast<int>(r_kp);
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));
  signal(SIGINT, signalHandler);

  //发送指令
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    processHoming(); //检查是否需要归零位
    loop_rate.sleep();
    /*cout<<"请输入电机的扭矩值："<<endl;
    cin>>torque_value;
    if (torque_value>T_max || torque_value<T_min)
    {  
        cout<<"输入有误"<<endl;
    }*/
    double y = 0.0;
    double torque = 0;
    // 状态机核心
    if (currentDriveMode == TORQUE_MODE) 
    {
        ST.isForward = true;
        if(!ST.initialMotorRecorded)
        {
            if (Enc.initialMotorPosition_counter % 2 == 1) 
            { // 奇数次
                Enc.initialMotorPosition_one = Output_Angle;
                Enc.initialMotorPosition = Enc.initialMotorPosition_one;
                cout << "支撑相起始电机角度1已存储" <<endl;
                ST.initialMotorRecorded = true;
            } 
            else 
            { // 偶数次
                Enc.initialMotorPosition_two = Output_Angle;
                Enc.initialMotorPosition = Enc.initialMotorPosition_two;
                cout << "支撑相起始电机角度2已存储" <<endl;
                ST.initialMotorRecorded = true;
            }
        }
        if (!isnan(Enc.initialMotorPosition_one) && !isnan(Enc.initialMotorPosition_two)) 
        {
            updateCompensateTorque(Enc , Tor);
        }
        double elapsed = timer.getElapsedTime();
        y = calculateTargetTorque();
        torque = y;
        sendTorTransVelCommand(y);
        if (elapsed  > Tor.DRIVE_DURATION) 
        {
            currentDriveMode = STAND_MODE;
            ST.isForward = false;
            if(pendingCommand == 0x43)
            {
                currentDriveMode = VELOCITY_MODE;
                pendingCommand = 0;
            }
        }
    }    
    else if (currentDriveMode == VELOCITY_MODE) 
    {
        ST.initialMotorRecorded = false;
        Enc.SwingMotorPosition = Output_Angle;
        if (abs(Enc.SwingMotorPosition - Enc.initialMotorPosition) < 5) 
        {
            currentDriveMode = STAND_MODE;
        }
        else
        {
            y = calculateTargetPosition();
            sendVelocityCommand(y , kd);
        }
    }
    else if (currentDriveMode == STAND_MODE) 
    {
        SendData(send_motor_torque, 0x00000001, config_motor3);
    }
    //发布消息
    can_ankle::msg::Torque torque_msg;
    torque_msg.velocity_value = y; //期望转速
    torque_msg.return_velocity = Output_VelocityValue; //实际返回转速
    torque_msg.torque_value = torque;         // 目标扭矩
    torque_msg.return_torque_value = Output_TorqueValue; // 实际返回扭矩
    torque_msg.pdvelocity = adjusted_velocity; // PD输出速度
    torque_msg.force_sensortorque = SensorTorque; //力传感器真实扭矩
    torque_pub->publish(torque_msg);
    }
}

void Init_Can(void)
{
  int ret;
  int i = 0;
  int m_run0 = 1; //线程创建参数
  if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ankle"), "Open CAN deivice error!");
    exit(1);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ankle"), "CAN connected!");

  config.AccCode = 0x80000008;
  config.AccMask = 0xffffffff;
  config.Filter = 2;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x14; // 1M波特率

  VCI_ResetCAN(VCI_USBCAN2, 0, 0);
  usleep(50000);
  VCI_ClearBuffer(VCI_USBCAN2, 0, 0);
  if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ankle"), "Init CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ankle"), "Start CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("ankle"), "CAN started!");

  // 初始化设置
  SendData(config_node, 0x00000001, config_motor1);
  usleep(100000);
  ret = pthread_create(&threadid, NULL, receive_func, &m_run0);
  if(ret != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ankle"), "创建线程失败！");
  }
}

void *receive_func(void *param)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ankle"), "Start CAN recieve....");
  usleep(20000);
  int reclen = 0;
  VCI_CAN_OBJ rec[2]; // 接收长度
  int *run = (int *)param;
  int ind=((*run)>>2);
  int i,j;
  while (rclcpp::ok())
  { 
    //调用接收函数，有数据则进行处理，接收长度2，等待10ms
    if ((reclen = VCI_Receive(VCI_USBCAN2, 0 , ind , rec, 2 , 10)) >= 0) 
    {  
        for (j = 0; j < reclen; j++) // 逐帧处理
        {   
            printf("data:0x");
            for (i=0 ; i<rec[j].DataLen ; i++)
            {  
                printf(" %02X",rec[j].Data[i]); 
            }
            printf("\n");
            uint16_t p1 = rec[j].Data[1];
            uint16_t p2 = rec[j].Data[2];
            uint16_t rv = rec[j].Data[3];
            uint16_t rv_t = rec[j].Data[4];
            uint16_t v_r = rv_t & 0xF0;
            uint16_t t_r = rv_t & 0x0F;
            uint16_t rt = rec[j].Data[5];
            cout<<"接收:["<<p1<<"],["<<p2<<"],["<<rv<<"],["<<rv_t<<"],["<<rt<<"]"<<endl;
            uint16_t current = (t_r<<8) | rt;
            cout<<current<<endl;
            cout<<dec<<"返回的电流参数:"<<current<<endl;
            uint16_t rv_high = (rv >> 4) & 0xF;
            uint16_t rv_low = rv  & 0xF;
            uint16_t realVelocity = (rv_high << 8) | (rv_low << 4) | (v_r & 0xF);
            cout<<realVelocity<<endl;
            cout<<dec<<"返回的速度参数:"<<realVelocity<<endl;
            uint16_t realPosition = (p1 << 8) | p2;
            cout<<dec<<"返回的位置参数:"<<current<<endl;
            double output_position = (realPosition * (P_max-P_min)/65536) + P_min;
            printf("返回的位置值：%.2f\n", output_position);
            double output_angle = output_position * 180 / M_PI;
            Output_Angle = output_angle;
            printf("返回的角度值：%.2f\n", output_angle);
		    double output_velocity = (realVelocity * (V_max-V_min)/4096) + V_min;
            Output_VelocityValue = output_velocity;
            printf("返回的速度值：%.2f\n", output_velocity);
            double output_torque = ((current*(T_max-T_min)/4096)+T_min) + (output_velocity-velocity_value)*kd;
            Output_TorqueValue = output_torque;
            printf("返回的扭矩值：%.2f\n", output_torque);
        }  
    }
  }
}

void SendData(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = 8;
  for (int i = 0; i < handle_obj.DataLen; i++)
  {
    handle_obj.Data[i] = data[i];
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, 0, &handle_obj, 1) > 0)
  {
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ankle"), "Error initializing!");
  }
}

double calculateTargetTorque() 
{
    double elapsed = timer.getElapsedTime();
    double torque = 0.0;
    // 阶段时间计算
    double rise_phase_start = Tor.DRIVE_DELAY_DURATION;
    double peak_phase_start = Tor.TARGET_TORQUE_PEAK_TIME;
    double fall_phase_end = peak_phase_start + Tor.TARGET_TORQUE_FALL_TIME;

    // 阶段1：张紧阶段
    if (elapsed < rise_phase_start) 
    {
        torque = (elapsed / rise_phase_start) * Tor.PRE_TORQUE;
    } 
    // 阶段2：上升阶段
    else if (elapsed < peak_phase_start) 
    {
        double t_rise = elapsed - rise_phase_start;
        double phase = M_PI * t_rise / Tor.TARGET_TORQUE_RISE_TIME;
        torque = ((1 - cos(phase)) * (Tor.TARGET_TORQUE_PEAK - Tor.PRE_TORQUE) / 2) + Tor.PRE_TORQUE;
    } 
    // 阶段3：下降阶段
    else if (elapsed < fall_phase_end) 
    {
        double t_fall = elapsed - peak_phase_start;
        double phase = M_PI * t_fall / Tor.TARGET_TORQUE_FALL_TIME;
        torque = (1 + cos(phase)) / 2 * Tor.TARGET_TORQUE_PEAK;
    }
    // 超出阶段归零
    return torque;
}

double calculateTargetPosition() 
{
    // 获取当前时间和位置
    auto now = high_resolution_clock::now();
    double theta_m = Enc.Encoder_Value; // 当前测量位置
    double theta_d = Enc.initialEncoderValue; // 期望位置

    // 计算时间差dt（单位：秒）
    double dt = duration_cast<duration<double>>(now - pos_controller.prev_time).count();
    if (dt < 1e-6) dt = 1e-6; // 防止除以零

    // 计算速度：当前速度 = (当前位置 - 上一时刻位置) / dt
    double theta_m_dot = (theta_m - pos_controller.prev_theta_m) / dt;

    // 根据公式计算期望速度
    double theta_d_dot = pos_controller.K_theta * (theta_d - theta_m) - pos_controller.K_d * theta_m_dot;

    // 更新上一时刻的数据
    pos_controller.prev_theta_m = theta_m;
    pos_controller.prev_time = now;

    // 返回控制量（期望速度）
    return theta_d_dot;
}