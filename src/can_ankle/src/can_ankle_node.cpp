#include "/home/user/ankle_ws/devel/include/can_ankle/Torque.h"
#include "/home/user/ankle_ws/devel/include/can_ankle/ForceSensor.h"
#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"


// 使用原子变量保证线程安全
atomic<bool> isDriving{false};
atomic<bool> isForward{true};
atomic<bool> isDelayPhase{false};
atomic<bool> isForwardDelay{false};
atomic<uint8_t> pendingCommand{0};

// 全局参数
double PID_TorqueValue = 0.0;
time_point<high_resolution_clock> startTime; // 节点开始运行的时间
time_point<high_resolution_clock> delayStartTime;   // 延时开始时间
time_point<high_resolution_clock> forwardDelayStart; // 正转延时开始时间

// 时间与驱动参数
const double DRIVE_DURATION = 0.625;
const double DELAY_DURATION = 0.1;
const double FORWARD_DELAY = 0.3;

// 线程安全队列
queue<uint8_t> commandQueue;
mutex commandMutex;

//clamp函数
template<typename T>
const T& clamp(const T& value , const T& low , const T& high) 
{
  return (value < low) ? low : ((value > high) ? high : value);
}

class Timer {
public:
    Timer() : lastCommandTime(high_resolution_clock::now()) {}
    
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

private:
    time_point<high_resolution_clock> lastCommandTime;
    mutable mutex mutex_;
};

Timer timer;

// PD控制器（带输出限幅）
struct PDController {
    double Kp, Kd;
    double prev_error{0.0};
    const double max_integral = 100.0; // 积分限幅
    const double max_output = T_max;   // 输出限幅

    PDController(double kp,  double kd) : Kp(kp) , Kd(kd) {}

    double compute(double setpoint, double measurement, double dt) 
    {
        if (dt <= 1e-6) return 0.0;
        
        double error = setpoint - measurement;
        double derivative = (error - prev_error) / dt;
        double output = Kp * error  + Kd * derivative;
        output = clamp(output, -max_output, max_output); // 输出限幅
        
        prev_error = error;
        return output;
    }
};

// PD参数
PDController pd(0.5 , 0.02); // 更稳定的参数

// 信号处理
void signalHandler(int signum) 
{
    cout << "Shutting down..." << std::endl;
    //退出时电机速度、位置、扭矩赋0
    SendData(send_motor_torque, 0x00000001, config_motor3);
    usleep(200000); // 等待0.2秒
    SendData(send_motor_torque, 0x00000001, config_motor2);
    usleep(200000); // 等待0.2秒 
    VCI_CloseDevice(VCI_USBCAN2, 0);
    ros::shutdown();
    exit(0);
}

// 指令回调
void commandCallback(const std_msgs::UInt8::ConstPtr& msg) 
{
    uint8_t command = msg->data;
    switch (command) 
    {
        case 0x41:  // 正向指令
            if (isDriving || isDelayPhase || isForwardDelay) 
            { 
                // 驱动中/延时中/正转延时中收到新指令，排队
                if (!isForward) pendingCommand = 0x41;
            } 
            else 
            { 
                // 空闲状态，启动正转延时
                isForwardDelay = true;
                forwardDelayStart = high_resolution_clock::now();
            }
            break;

        case 0x42:  // 反向指令
        
            if (isDriving || isDelayPhase || isForwardDelay) 
            { 
                if (isForward) pendingCommand = 0x42;
            } 
            else 
            { 
                isDriving = true;
                isForward = false;
                timer.startNewTiming();
            }
            break;
    }
}

//接收力传感器回调
void torqueCallback(const can_ankle::ForceSensor::ConstPtr& msg)
{
    double forcevalue = msg->ForceValue;
    SensorTorque = forcevalue * 0.018 * 9.8;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "can_ankle_node");
    ros::NodeHandle nh;
    ros::Publisher torque_pub = nh.advertise<can_ankle::Torque>("torque_info", 10);
    ros::Subscriber command_sub = nh.subscribe("command_topic", 10, commandCallback);
    ros::Subscriber ForceValue_sub = nh.subscribe("Force", 10, torqueCallback);
    
    Init_Can();
    signal(SIGINT, signalHandler);
    
    auto prev_time = high_resolution_clock::now();
    ros::Rate loop_rate(200);
    
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        // 计算时间差
        auto now = high_resolution_clock::now();
        double dt = duration_cast<duration<double>>(now - prev_time).count();
        prev_time = now;
        
        // 状态机逻辑
        double y = 0.0;
        if (isForwardDelay)
        {  // 处理正转延时阶段
            auto now = high_resolution_clock::now();
            duration<double> elapsed = now - forwardDelayStart;
            if (elapsed.count() >= FORWARD_DELAY) 
            { // 延时结束
                isForwardDelay = false;
                isDriving = true;
                isForward = true;
                timer.startNewTiming();
            }
        } 
        else if (isDriving) 
        {  // 驱动阶段
            double localX_T = timer.getElapsedTime();
            if (localX_T >= DRIVE_DURATION) 
            { 
                isDriving = false;
                BYTE torque_data[8] = 
                {
                0x7F, 0xFF, 0x7F, 0xF0, 
                0x00, 0x00, 0x07, 0xFF
                };
                SendData(send_motor_torque, 0x00000001, torque_data);
                if (pendingCommand != 0) 
                { 
                    isDelayPhase = true;
                    delayStartTime = high_resolution_clock::now();
                }
            } 
            else 
            { 
                if (isForward)//正转
                {
                    y = 4.743 * sin(5.301 * localX_T - 0.348) + 1.676* sin(16.06 * localX_T + 1.851);
                    // PD计算（跟踪y值）
                    adjusted_torque = pd.compute(y, SensorTorque, dt);
                    uint16_t transformValue1 = static_cast<uint16_t>((adjusted_torque - T_min) / (T_max - T_min) * 4096);
                    BYTE torque_data[8] = 
                    {
                    0x7F, 0xFF, 0x7F, 0xF0, 
                    0x00, 0x00, 
                    static_cast<BYTE>((transformValue1 >> 8) & 0xFF), 
                    static_cast<BYTE>(transformValue1 & 0xFF)
                    };
                    SendData(send_motor_torque, 0x00000001, torque_data);
                }
                if (!isForward)//反转
                {
                    y = 4.743 * sin(5.301 * localX_T + 2.794) + 1.676* sin(16.06 * localX_T - 1.291);
                    // PD计算（跟踪y值）
                    adjusted_torque = pd.compute(y, SensorTorque, dt);
                    uint16_t transformValue2 = static_cast<uint16_t>((adjusted_torque - T_min) / (T_max - T_min) * 4096);
                    BYTE torque_data[8] = 
                    {
                    0x7F, 0xFF, 0x7F, 0xF0, 
                    0x00, 0x00, 
                    static_cast<BYTE>((transformValue2 >> 8) & 0xFF), 
                    static_cast<BYTE>(transformValue2 & 0xFF)
                    };
                    SendData(send_motor_torque, 0x00000001, torque_data);
                }
            }
        }
        else if (isDelayPhase) 
        {// 延时阶段处理
            auto now = high_resolution_clock::now();
            duration<double> elapsed = now - delayStartTime;
            if (elapsed.count() >= DELAY_DURATION) 
            { // 延时结束
                isDelayPhase = false;
                isForward = (pendingCommand == 0x41);
                isDriving = true;
                pendingCommand = 0;
                timer.startNewTiming();
            }
        }
        
        // 发布消息（包含原始指令和实际扭矩）
        can_ankle::Torque torque_msg;
        torque_msg.TorqueValue = y;         // 期望扭矩
        torque_msg.ReturnTorqueValue = Output_TorqueValue; // 实际扭矩
        torque_msg.PIDtorque = adjusted_torque; // PD输出扭矩
        torque_msg.ForceSensortorque = SensorTorque; //力传感器真实扭矩
        torque_pub.publish(torque_msg);
        
        
        loop_rate.sleep();
    }
    return 0;
}

void Init_Can(void)
{
  int ret;
  int i = 0;
  int m_run0 = 1;
 
  if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1)
  {
    ROS_ERROR_STREAM("Open CAN deivice error!");
    exit(1);
  }
  ROS_INFO_STREAM("CAN connected!");

  config.AccCode = 0;
  config.AccMask = 0xffffffff;
  config.Filter = 2;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x14; // 1M波特率

  if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
  {
    ROS_ERROR_STREAM("Init CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
  {
    ROS_ERROR_STREAM("Start CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  ROS_INFO_STREAM("CAN started!");

  // 初始化设置
  SendData(config_node, 0x00000001, config_motor1);
  usleep(100000);
  ret = pthread_create(&threadid, NULL, receive_func, &m_run0);
}

void *receive_func(void *param)
{
  ROS_INFO_STREAM("Start CAN recieve....");
  usleep(20000);
  int reclen = 0;
  VCI_CAN_OBJ rec[2]; // 接收长度
  int *run = (int *)param;
  int ind=((*run)>>2);
  int i,j;
  int32_t signedInt_Tor[6]={0x00,0x00,0x00,0x00,0x00,0x00};
  while (ros::ok())
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
            uint32_t r1 = rec[j].Data[4];
            uint32_t d_r1 = r1 & 0x0F;
            uint32_t r2 = rec[j].Data[5];
            cout<<d_r1<<r2<<endl;
            uint32_t current = (d_r1<<8) | r2;
            cout<<current<<endl;
            cout<<dec<<"返回的电流参数:"<<current<<endl;
		        double output_torque = (current*(T_max-T_min)/4096)+T_min;
            Output_TorqueValue = output_torque;
            printf("返回的扭矩值：%.2f\n",output_torque);
        }  
    }
  }
}

void SendData(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = getArrayLen(data);
  for (int i = 0; i < handle_obj.DataLen; i++)
  {
    handle_obj.Data[i] = data[i];
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, 0, &handle_obj, 1) > 0)
  {
  }
  else
  {
    ROS_ERROR_STREAM("Error initializing!");
  }
}
