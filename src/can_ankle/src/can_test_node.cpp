#include "/home/user/ankle_ws/devel/include/can_ankle/Torque.h"
#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"
#include "/home/user/ankle_ws/devel/include/can_ankle/ForceSensor.h"

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
  VCI_CloseDevice(VCI_USBCAN2, 0);
  ros::shutdown();
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
  ros::init(argc, argv, "can_ankle_node");
  ros::NodeHandle nh;
  ros::Publisher torque_pub = nh.advertise<can_ankle::Torque>("torque_info", 10); //发布各项信息

  //初始化can节点
  Init_Can();
  ros::Rate loop_rate(200);
  signal(SIGINT, signalHandler);
  steady_clock::time_point start_time;
  //发送指令
  while (ros::ok())
  {
    ros::spinOnce();
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

    y = 30 * sin(1 * M_PI * elapsed); //单位是度/s

    speed_to_command(y);

    //发布消息
    velocity_value = y;
    can_ankle::Torque torque_msg;
    torque_msg.VelocityValue = velocity_value;
    torque_msg.ReturnVelocity = Output_VelocityValue;
    torque_msg.ReturnTorqueValue = Output_TorqueValue;
    torque_pub.publish(torque_msg);
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

  config.AccCode = 0x80000000;
  config.AccMask = 0xffffffff;
  config.Filter = 1;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1C; // 500k波特率

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
  usleep(100000);
  ret = pthread_create(&threadid, NULL, receive_func, &m_run0);
  sendMSG();
  //速度模式
  motor_on_V();
}

void *receive_func(void *param)
{
  ROS_INFO_STREAM("Start CAN recieve....");
  usleep(20000);
  int reclen = 0;
  VCI_CAN_OBJ rec[200]; // 接收长度
  int *run = (int *)param;
  int ind=((*run)>>2);
  int i,j;
  while (ros::ok())
  { 
    //调用接收函数，有数据则进行处理，接收长度200，等待0ms
    if ((reclen = VCI_Receive(VCI_USBCAN2, 0 , ind , rec, 200 , 0)) >= 0) 
    {  
      for (j = 0; j < reclen; j++) // 逐帧处理
      {   
          if (rec[j].ID == 0x000001D3)
          {
            printf("八位数据data:0x");
            for (i=0 ; i<rec[j].DataLen ; i++)
            {  
              printf(" %02X",rec[j].Data[i]); 
            }
            printf("\n");
            uint8_t rt1 = rec[j].Data[2];
            uint8_t rt2 = rec[j].Data[3];
            uint8_t rp1 = rec[j].Data[4];
            uint8_t rp2 = rec[j].Data[5];
            uint8_t rp3 = rec[j].Data[6];
            uint8_t rp4 = rec[j].Data[7];
            uint16_t combined_torque = (rt2 << 8) | rt1;
            uint32_t combined_position = (rp4 << 24) | (rp3 << 16) | (rp2 << 8) | rp1;
            int16_t torque_output = static_cast<int16_t>(combined_torque);
            int32_t position_output = static_cast<int32_t>(combined_position);
          }
          else if (rec[j].ID == 0x000003D3)
          {
            printf("五位数据data:0x");
            for (i=0 ; i<rec[j].DataLen ; i++)
            {  
              printf(" %02X",rec[j].Data[i]); 
            }
            printf("\n");
            uint8_t rv1 = rec[j].Data[1];
            uint8_t rv2 = rec[j].Data[2];
            uint8_t rv3 = rec[j].Data[3];
            uint8_t rv4 = rec[j].Data[4];
            uint32_t combined_value = (rv4 <<24) | (rv3 <<16) | (rv2 <<8) | rv1;
            int32_t velocity_output = static_cast<int32_t>(combined_value);
            const double RPM_SCALE = 0.6 / 0x00040000;
            double speed_rpm = RPM_SCALE * velocity_output;
            double speed_deg_per_sec = 6.0 * speed_rpm;
            cout << "速度：（度/s)" << speed_deg_per_sec <<endl;
            Output_VelocityValue = speed_deg_per_sec;
          }
          else
          {
            printf("data:0x");
            for (i=0 ; i<rec[j].DataLen ; i++)
            {  
              printf(" %02X",rec[j].Data[i]); 
            }
            printf("\n");
          }
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
    ROS_ERROR_STREAM("Error initializing!");
  }
}

void SendData_five(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = 5;
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
