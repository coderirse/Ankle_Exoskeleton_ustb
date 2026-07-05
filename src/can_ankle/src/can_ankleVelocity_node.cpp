#include "can_ankle/msg/torque.hpp"
#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"

double currentY = 0.0;
bool isDriving = false;
bool isForward = true;
bool isSwitchDelay = false;
bool isWaitingForward = false;
double Encoder_Value = 0;
uint8_t pendingCommand = 0;
double initialEncoderValue = 0;
std::atomic<bool> isReturningHome{false};
const double RETURN_TORQUE = 1.0;
const int ENCODER_SAFE_LIMIT = 45;
const double ENCODER_FORWARD = 8;
const double DRIVE_DURATION = 1.25;
const double DELAY_DURATION = 0.2;
time_point<high_resolution_clock> delayStartTime;

rclcpp::Node::SharedPtr g_node_vel;

class Timer {
public:
    Timer() : lastCommandTime(high_resolution_clock::now()) {}
    void startNewTiming() { lock_guard<mutex> lock(mutex_); lastCommandTime = high_resolution_clock::now(); }
    double getElapsedTime() { lock_guard<mutex> lock(mutex_); return duration_cast<duration<double>>(high_resolution_clock::now() - lastCommandTime).count(); }
private:
    time_point<high_resolution_clock> lastCommandTime;
    mutable mutex mutex_;
};
Timer timer;

void signalHandler(int signum) {
  cout << "启动归零程序..." << endl;
  isReturningHome = true;
  double return_torque = (initialEncoderValue > Encoder_Value) ? RETURN_TORQUE : -RETURN_TORQUE;
  uint16_t tv = (return_torque - T_min)/((T_max - T_min)/4096);
  BYTE rd[8] = {0x7F,0xFF,0x7F,0xF0,0x07,0xFF,(BYTE)((tv>>8)&0xFF),(BYTE)(tv&0xFF)};
  while(abs(Encoder_Value - initialEncoderValue) > 1 && rclcpp::ok()) { SendData(send_motor_torque,0x00000001,rd); usleep(50000); }
  cout << "归零完成，关闭节点" << endl;
  SendData(send_motor_torque,0x00000001,config_motor3); usleep(200000);
  VCI_CloseDevice(VCI_USBCAN2,0); rclcpp::shutdown(); exit(0);
}

void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t cmd = msg->data;
    switch(cmd){
        case 0x41:
            if(isDriving && !isForward) pendingCommand=0x41;
            else if(!isDriving && !isSwitchDelay){ if(Encoder_Value>ENCODER_FORWARD){ isDriving=true; isForward=true; timer.startNewTiming(); } else { isWaitingForward=true; RCLCPP_WARN(g_node_vel->get_logger(),"等待编码器>8°: %.1f",Encoder_Value); } }
            else if(isSwitchDelay) pendingCommand=0x41;
            break;
        case 0x42:
            if(isDriving && isForward) pendingCommand=0x42;
            else if(!isDriving && !isSwitchDelay){ isDriving=true; isForward=false; timer.startNewTiming(); }
            else if(isSwitchDelay) pendingCommand=0x42;
            break;
    }
}

void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    double a = msg->data; if(a>300) a-=360; Encoder_Value=a;
    if(abs(Encoder_Value)>ENCODER_SAFE_LIMIT && !isReturningHome){ RCLCPP_ERROR(g_node_vel->get_logger(),"编码器超限: %.1f",Encoder_Value); signalHandler(SIGINT); }
}

int main(int argc, char **argv) {
  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("can_ankle_velocity"); g_node_vel=node;
  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info",10);
  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>("command_topic",10,commandCallback);
  auto encoder_sub = node->create_subscription<std_msgs::msg::Float64>("angle",10,encoderCallback);

  Init_Can(); usleep(500000);
  initialEncoderValue = Encoder_Value;
  RCLCPP_INFO(g_node_vel->get_logger(),"初始编码器: %.1f",initialEncoderValue);

  cout<<"请输入Kd:"<<endl; cin>>kd;
  double r_kd=819.2*kd; int intr_kd=(int)r_kd;
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));
  signal(SIGINT,signalHandler);

  while(rclcpp::ok()){
    rclcpp::spin_some(node); loop_rate.sleep();
    output[0]=0x7F;output[1]=0xFF;output[2]=0x7F;output[3]=0xF0;output[4]=0x00;output[5]=0x00;output[6]=0x07;output[7]=0xFF;
    double y=0.0;

    if(isDriving){
        double elapsed=timer.getElapsedTime();
        if(isForward){ y=5*sin(0.8*M_PI*elapsed); if(elapsed>=DRIVE_DURATION){ isDriving=false; y=0; if(pendingCommand){ isSwitchDelay=true; delayStartTime=high_resolution_clock::now(); } else isForward=false; } }
        else { y=-5*sin(0.8*M_PI*elapsed); if(elapsed>=DRIVE_DURATION){ isDriving=false; y=0; if(pendingCommand){ isSwitchDelay=true; delayStartTime=high_resolution_clock::now(); } } }
    } else if(isWaitingForward){
        if(Encoder_Value>ENCODER_FORWARD){ isWaitingForward=false; isDriving=true; isForward=true; timer.startNewTiming(); RCLCPP_INFO(g_node_vel->get_logger(),"开始正转"); }
    } else if(isSwitchDelay){
        if(duration_cast<duration<double>>(high_resolution_clock::now()-delayStartTime).count()>=DELAY_DURATION){ isSwitchDelay=false; isDriving=true; isForward=(pendingCommand==0x41); pendingCommand=0; timer.startNewTiming(); }
    }

    velocity_value=y;
    auto torque_msg=can_ankle::msg::Torque();
    torque_msg.velocity_value=velocity_value; torque_msg.return_velocity=Output_VelocityValue; torque_msg.return_torque_value=Output_TorqueValue;
    torque_pub->publish(torque_msg);

    uint16_t tv=(velocity_value-V_min)/((V_max-V_min)/4096);
    output[2]=(uint16_t)((tv>>4)&0xFF); output[3]&=0x0F; uint8_t lf=(uint8_t)(tv&0xFF); lf=lf<<4; output[3]|=lf;
    output[5]=(uint16_t)((intr_kd>>4)&0xFF); output[6]=(uint16_t)((intr_kd&0x0F)<<4|0x07);
    BYTE vd[8]={(BYTE)output[0],(BYTE)output[1],(BYTE)output[2],(BYTE)output[3],(BYTE)output[4],(BYTE)output[5],(BYTE)output[6],(BYTE)output[7]};
    SendData(send_motor_torque,0x00000001,vd);
  }
  rclcpp::shutdown(); return 0;
}

void Init_Can(void){
  int ret, m_run0=1;
  if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1){ RCLCPP_ERROR_STREAM(g_node_vel->get_logger(),"Open CAN error!"); exit(1); }
  RCLCPP_INFO_STREAM(g_node_vel->get_logger(),"CAN connected!");
  config.AccCode=0x80000008; config.AccMask=0xffffffff; config.Filter=2; config.Mode=0; config.Timing0=0x00; config.Timing1=0x14;
  VCI_ResetCAN(VCI_USBCAN2,0,0); usleep(50000); VCI_ClearBuffer(VCI_USBCAN2,0,0);
  if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1){ RCLCPP_ERROR_STREAM(g_node_vel->get_logger(),"Init CAN error!"); VCI_CloseDevice(VCI_USBCAN2,0); }
  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1){ RCLCPP_ERROR_STREAM(g_node_vel->get_logger(),"Start CAN error!"); VCI_CloseDevice(VCI_USBCAN2,0); }
  RCLCPP_INFO_STREAM(g_node_vel->get_logger(),"CAN started!");
  SendData(config_node,0x00000001,config_motor1); usleep(100000);
  ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
}

void *receive_func(void *param){
  usleep(20000); int reclen=0; VCI_CAN_OBJ rec[2]; int *run=(int*)param; int ind=((*run)>>2); int i,j;
  while(rclcpp::ok()){
    if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,2,10))>=0){
      for(j=0;j<reclen;j++){
        uint32_t rv=rec[j].Data[3],rv_t=rec[j].Data[4],v_r=rv_t&0xF0,t_r=rv_t&0x0F,rt=rec[j].Data[5],current=(t_r<<8)|rt;
        uint32_t rvh=(rv>>4)&0xF,rvl=rv&0xF,realV=(rvh<<8)|(rvl<<4)|(v_r&0xF);
        Output_VelocityValue=(realV*(V_max-V_min)/4096)+V_min;
        Output_TorqueValue=((current*(T_max-T_min)/4096)+T_min)+(Output_VelocityValue-velocity_value)*kd;
      }
    }
  }
  return NULL;
}

void SendData(VCI_CAN_OBJ &h, const int id, const BYTE *d){
  h.ID=id; h.RemoteFlag=0; h.ExternFlag=0; h.DataLen=8; memcpy(h.Data,d,8);
  if(VCI_Transmit(VCI_USBCAN2,0,0,&h,1)<=0) RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_send"),"Error!");
}
