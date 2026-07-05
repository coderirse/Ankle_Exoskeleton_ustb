#include "can_ankle/msg/torque.hpp"
#include "can_ankle/can_ankle_node.h"
#include "can_ankle/controlcan.h"

// 参数宏定义
uint8_t pendingCommand = 0;
const double RETURN_VELOCITY = 8;
double step = 0;
uint8_t byte0;
uint8_t byte1;
uint8_t byte2;
uint8_t byte3;
uint8_t lastCommand;
double user_weight = 0;
int control_mode = 0;
bool slope_file_initialized = false;
std::ofstream slope_file;

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
    BYTE para_Vmode1[8] = {0x23, 0x85, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};
    BYTE para_Vmode2[8] = {0x23, 0x83, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};
    BYTE para_Vmode3[8] = {0x23, 0x84, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};
    BYTE para_Pmode1[8] = {0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};
    BYTE para_Pmode2[8] = {0x23, 0x83, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};
    BYTE para_Pmode3[8] = {0x23, 0x84, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};
};
ConfigNode cfg;

// 使用 std::clamp (C++17)

//状态变量
class state_parameter
{
public:
    bool isForward = false;
    bool need_homing = false;
    bool initialEncoderRecorded = false;
    bool initialMotorRecorded = false;
    bool sendV_mode = true;
    bool sendP_mode = false;
    bool recorded = true;
    bool record = true;
    bool slope_store = false;
    bool RetrunInitial = false;
};
state_parameter ST;

//编码器参数
class encoder_parameter
{
public:
    double Encoder_Value = 0;
    double initialEncoderValue = 0;
    double position_angle = 0;
    const int ENCODER_SAFE_LIMIT = 45;
    double initialMotorPosition_one = NAN;
    double initialMotorPosition_two = NAN;
    double initialMotorPosition = 0;
    int initialMotorPosition_counter = 0;
    double SwingMotorPosition = 0;
};
encoder_parameter Enc;

//支撑相扭矩驱动参数信息
class Torque_parameter
{
public:
    double DRIVE_DURATION =0;
    double DRIVE_DELAY_DURATION = 0;
    double TARGET_TORQUE_RISE_TIME = 0;
    double TARGET_TORQUE_FALL_TIME = 0;
    double TARGET_TORQUE_BASE = 0;
    double TARGET_TORQUE_FLOAT = 0;
    double TARGET_TORQUE_ASSIST = 0;
    double TARGET_TORQUE_EXT = 0;
    double TARGET_TORQUE_PEAK = 0;
    double PRE_TORQUE = 0;
    double CompensatePosition = 0;
    double RealPace = 0;
    double slope = 0;
};
Torque_parameter Tor;

//时间相关函数
class Timer
{
public:
    double time1 = 0;
    double time2 = 0;
    double time3 = 0;
    double time4 = 0;
    double TheorSwingTime = 0;
    Timer() : lastCommandTime(high_resolution_clock::now()),
              lastDtTime(high_resolution_clock::now()),
              lastLoopTime(high_resolution_clock::now()) {}

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

    double getRealTimeDt()
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        double dt = duration_cast<duration<double>>(now - lastDtTime).count();
        lastDtTime = now;
        return dt > 1e-6 ? dt : 1e-6;
    }
private:
    time_point<high_resolution_clock> lastCommandTime;
    time_point<high_resolution_clock> lastDtTime;
    time_point<high_resolution_clock> lastLoopTime;
    mutable mutex mutex_;
};
Timer timer;

class IMU_parameter
{
    public:
        double imu_orientation_w = 0;
        double imu_orientation_x = 0;
        double imu_orientation_y = 0;
        double imu_orientation_z = 0;
        double imu_AngelVelocity_x = 0;
        double imu_AngelVelocity_y = 0;
        double imu_AngelVelocity_z = 0;
        double imu_LinerAcceleration_x = 0;
        double imu_LinerAcceleration_y = 0;
        double imu_LinerAcceleration_z = 0;
        double linear_velocity_x = 0;
        double linear_velocity_y = 0;
        double linear_velocity_z = 0;
        double current_linear_velocity_y = 0;
    };
IMU_parameter imu;

enum DriveMode {VELOCITY_MODE, TORQUE_MODE, PRE_TORQUE_MODE, TORQUE_DRIVE_MODE , STAND_MODE};
DriveMode currentDriveMode = STAND_MODE;

struct PIDController
{
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prev_setpoint;
    double prev_measurement;

    PIDController(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), integral(0.0),
          prev_setpoint(0.0), prev_measurement(0.0) {}

    double compute(double setpoint, double measurement, double dt) {
        if (dt <= 1e-6) dt = 1e-6;
        double error = setpoint - measurement;
        double delta_setpoint = setpoint - prev_setpoint;
        double delta_measurement = measurement - prev_measurement;
        double derivative = (delta_setpoint / dt) - (delta_measurement / dt);
        integral += error * dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        output = std::clamp(output, V_min, V_max);
        prev_setpoint = setpoint;
        prev_measurement = measurement;
        return output;
    }

    void resetIntegral()
    {
      if (!TORQUE_MODE) { integral = 0.0; }
    }
    void updateParamsBasedOnSpeed(double speed)
    {
        if (speed < 2) { Kp = 20; Ki = 0.003; Kd = 0.2; }
        else if (speed >= 2 && speed < 3) { Kp =30; Ki = 0.004; Kd = 0.3; }
        else if (speed >= 3 && speed < 4) { Kp = 40; Ki = 0.005; Kd = 0.4; }
        else if (speed >= 4 && speed < 5) { Kp = 50; Ki = 0.006; Kd = 0.5; }
        else if (speed > 5) { Kp = 60; Ki = 0.007; Kd = 0.6; }
    }
};

PIDController pid(2 , 0.001 , 0.1);

struct PositionController
{
    double K_theta;
    double K_d;
    double prev_theta_m;
    time_point<high_resolution_clock> prev_time;
    PositionController(double kt, double kd) : K_theta(kt), K_d(kd), prev_theta_m(0), prev_time(high_resolution_clock::now()) {}
};
PositionController pos_controller(2 , 0.02);

struct StartPositionController
{
    double K_theta;
    double K_d;
    double prev_theta_m;
    time_point<high_resolution_clock> prev_time;
    StartPositionController(double kt, double kd) : K_theta(kt), K_d(kd), prev_theta_m(0), prev_time(high_resolution_clock::now()) {}
};
StartPositionController startpos_controller(3 , 0.03);

rclcpp::Node::SharedPtr g_node;

void signalHandler(int signum)
{
  cout << "启动归零程序..." << endl;
  ST.need_homing = true;
}

void AdaptiveSpeed(double x)
{
  const double a1 = 1.45022;
  const double b1 = 0.4695;
  const double c1 = 0.06345;
  const double K = 5;
  double i1 = (x - c1) / a1;
  double j1 = - log(i1) / b1;
  Tor.RealPace = j1;
  if (0 <= Tor.RealPace && Tor.RealPace <=  6)
  {
    if (0 <= Tor.RealPace && Tor.RealPace <=  2.1)
    {
      Tor.DRIVE_DURATION = 1.2 * (-0.1908 * Tor.RealPace + 0.8061);
      Tor.TARGET_TORQUE_EXT = (Tor.RealPace - 2.1) * K / 2.1;
    }
    else if(Tor.RealPace > 2.1 && Tor.RealPace <= 4.8)
    {
      Tor.DRIVE_DURATION = 1.2 * (0.03754 * Tor.RealPace * Tor.RealPace - 0.3 * Tor.RealPace + 0.87);
      Tor.TARGET_TORQUE_EXT = 0;
    }
    else if(Tor.RealPace > 4.8 && Tor.RealPace <= 6)
    {
      Tor.DRIVE_DURATION = 0.2949216  * 1.2;
      Tor.TARGET_TORQUE_EXT = (Tor.RealPace - 4.8) * K / 1.2;
    }
  }
  else
  {
    if (Tor.RealPace > 6) { Tor.DRIVE_DURATION = 0.2949216 * 1.3; Tor.TARGET_TORQUE_EXT = K; }
    else { Tor.DRIVE_DURATION = 0.8; Tor.TARGET_TORQUE_EXT = K; }
  }
  double R = 0.035 * Tor.RealPace  + 0.5075;
  Tor.TARGET_TORQUE_RISE_TIME = Tor.DRIVE_DURATION * R ;
  R = std::clamp(R , 0.4 , 0.7);
  Tor.TARGET_TORQUE_FALL_TIME = Tor.DRIVE_DURATION - Tor.TARGET_TORQUE_RISE_TIME;
  Tor.DRIVE_DURATION = std::clamp(Tor.DRIVE_DURATION , min_duration , max_duration);
  if(ST.recorded) { cout << "time2:" << x << "realpace:" << Tor.RealPace <<"drive time:" << Tor.DRIVE_DURATION  << "R value:" << R << endl; ST.recorded = false; }
  const double a3 = 1.2536604201;
  const double b3 = 0.5741810332;
  const double c3 = 0.4461945834;
  timer.TheorSwingTime = a3 * exp(-b3 * Tor.RealPace) + c3;
  pid.updateParamsBasedOnSpeed(Tor.RealPace);
}

void processHoming()
{
    if (ST.need_homing)
    {
        double return_velocity = (Enc.initialEncoderValue > Enc.Encoder_Value) ? RETURN_VELOCITY : -RETURN_VELOCITY;
        speed_to_command(return_velocity);
        BYTE return_data[5] = {0x03, byte0, byte1, byte2, byte3};
        SendData_five(send_motor_torque, 0x00000353, return_data);
        usleep(50000);
        while(abs(Enc.Encoder_Value - Enc.initialEncoderValue) < 2 && rclcpp::ok())
        {
            SendData_five(config_node, 0x00000353, cfg.para_V);
            usleep(200000);
            cout << "关闭节点" << endl;
            SendData(config_node, 0x00000253, cfg.para_motor2);
            usleep(200000);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            rclcpp::shutdown();
            exit(0);
        }
    }
}

void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double receive_angle = msg->data;
    if(receive_angle > 300) receive_angle -= 360;
    Enc.Encoder_Value = receive_angle;
    if(!ST.initialEncoderRecorded)
    {
        Enc.initialEncoderValue = Enc.Encoder_Value;
        RCLCPP_INFO(g_node->get_logger(), "Initial encoder position: %.2f\n", Enc.initialEncoderValue);
        ST.initialEncoderRecorded = true;
    }
    if(abs(Enc.Encoder_Value) > Enc.ENCODER_SAFE_LIMIT)
    {
        RCLCPP_ERROR(g_node->get_logger(), "Encoder value %.2f exceeds safety limit!", Enc.Encoder_Value);
        signalHandler(SIGINT);
    }
    Enc.position_angle = Enc.Encoder_Value;
}

void torqueCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  float forcevalue = msg->data;
  SensorTorque = forcevalue * 0.12 ;
  if (forcevalue > 300) { ST.need_homing = true; }
}

void one_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg) { timer.time1 = msg->data; }
void two_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg) { timer.time2 = msg->data; }
void three_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg) { timer.time3 = msg->data; }
void swing_timeCallback(const std_msgs::msg::Float64::SharedPtr msg) { timer.time4 = msg->data; }

void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  imu.linear_velocity_y = msg->linear.y;
  imu.current_linear_velocity_y = -imu.linear_velocity_y;
}

void extractImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu.imu_orientation_w = msg->orientation.w;
    imu.imu_orientation_x = msg->orientation.x;
    imu.imu_orientation_y = msg->orientation.y;
    imu.imu_orientation_z = msg->orientation.z;
    imu.imu_AngelVelocity_x = msg->angular_velocity.x;
    imu.imu_AngelVelocity_y = msg->angular_velocity.y;
    imu.imu_AngelVelocity_z = msg->angular_velocity.z;
    imu.imu_LinerAcceleration_x = msg->linear_acceleration.x;
    imu.imu_LinerAcceleration_y = msg->linear_acceleration.y;
    imu.imu_LinerAcceleration_z = msg->linear_acceleration.z;
}

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) { extractImuData(msg); }

void updateCompensateTorque()
{
    if (isnan(Enc.initialMotorPosition_one)) return;
    if (isnan(Enc.initialMotorPosition_two)) return;
    bool even_cycle = (Enc.initialMotorPosition_counter / 2) % 2 == 0;
    double a = Enc.initialMotorPosition_two - Enc.initialMotorPosition_one;
    double b = Enc.initialMotorPosition_one - Enc.initialMotorPosition_two;
    if (even_cycle)
    {
        Tor.CompensatePosition = a;
        double angleDiff = Output_Angle - Enc.initialMotorPosition_one;
        double TransV = angleDiff * 2;
        speed_to_command(TransV);
        BYTE velocity_data[5] = {0x03, byte0, byte1, byte2, byte3};
        SendData_five(config_node, 0x00000353, velocity_data);
    }
    else
    {
        Tor.CompensatePosition = b;
        double angleDiff = Output_Angle - Enc.initialMotorPosition_two;
        double TransV = angleDiff * 2;
        speed_to_command(TransV);
        BYTE velocity_data[5] = {0x03, byte0, byte1, byte2, byte3};
        SendData_five(config_node, 0x00000353, velocity_data);
    }
    RCLCPP_INFO(g_node->get_logger(), "补偿角度： %f", Tor.CompensatePosition);
}

void StandSafty()
{
  if(timer.time2 > 5) { currentDriveMode = STAND_MODE; signal(SIGINT, signalHandler); }
  else { currentDriveMode = TORQUE_DRIVE_MODE; }
}

vector<uint8_t> speed_to_command(double speed)
{
    double rpm = speed / 6.0;
    int32_t raw_value = static_cast<int32_t>(rpm * (65536.0 / 0.15));
    byte0 = static_cast<uint8_t>(raw_value & 0xFF);
    byte1 = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
    byte2 = static_cast<uint8_t>((raw_value >> 16) & 0xFF);
    byte3 = static_cast<uint8_t>((raw_value >> 24) & 0xFF);
    return {byte0, byte1, byte2, byte3};
}

void sendTorTransVelCommand(double y)
{
    torque_value = y;
    double dt = timer.getRealTimeDt();
    adjusted_velocity = pid.compute(torque_value, SensorTorque, dt);
    velocity_value = adjusted_velocity;
    speed_to_command(velocity_value);
    velocity_output[1] = byte0;
    velocity_output[2] = byte1;
    velocity_output[3] = byte2;
    velocity_output[4] = byte3;
    BYTE velocity_data[5] = {velocity_output[0],velocity_output[1],velocity_output[2],velocity_output[3],velocity_output[4]};
    SendData_five(send_motor_torque, 0x00000353, velocity_data);
}

void sendVelocityCommand(double y)
{
  velocity_value = y;
  speed_to_command(velocity_value);
  velocity_output[1] = byte0;
  velocity_output[2] = byte1;
  velocity_output[3] = byte2;
  velocity_output[4] = byte3;
  BYTE velocity_data[5] = {velocity_output[0],velocity_output[1],velocity_output[2],velocity_output[3],velocity_output[4]};
  SendData_five(send_motor_torque, 0x00000353, velocity_data);
}

void storeSlope()
{
    double orientationZ = 0;
    if(!ST.slope_store) { orientationZ = imu.imu_orientation_z; ST.slope_store = true; }
    if (orientationZ > 180) orientationZ -= 360;
    Tor.slope = 0 - orientationZ;
    const double maxTorque = 5;
    const double minTorque = -5;
    Tor.TARGET_TORQUE_ASSIST = 8 / M_PI * atan(Tor.slope / 2);
    Tor.TARGET_TORQUE_ASSIST = std::clamp(Tor.TARGET_TORQUE_ASSIST, minTorque, maxTorque);
    if (slope_file.is_open()) { slope_file << Tor.slope << "\n"; slope_file.flush(); }
}

void initializeSlopeFile()
{
    if (!slope_file_initialized)
    {
        std::string home_path = getenv("HOME") ? getenv("HOME") : ".";
        std::string file_path = home_path + "/Slope.txt";
        slope_file.open(file_path.c_str(), std::ios::out | std::ios::trunc);
        if (slope_file.is_open()) { slope_file << "坡度值\nSlope (degrees)\n"; slope_file.flush(); slope_file_initialized = true; }
    }
}

bool checkCommandSequence(uint8_t currentCommand)
{
    if (lastCommand == 0) return true;
    switch (currentCommand)
    {
        case 0x41: return true;
        case 0x42: if (lastCommand != 0x41) { RCLCPP_ERROR(g_node->get_logger(), "顺序错误：0x42不应在0x%02X之后", lastCommand); return false; } break;
        case 0x43: if (lastCommand != 0x42) { RCLCPP_ERROR(g_node->get_logger(), "顺序错误：0x43不应在0x%02X之后", lastCommand); return false; } break;
        case 0x44: if (lastCommand != 0x43) { RCLCPP_ERROR(g_node->get_logger(), "顺序错误：0x44不应在0x%02X之后", lastCommand); return false; } break;
        default: RCLCPP_ERROR(g_node->get_logger(), "未知指令：0x%02X", currentCommand); return false;
    }
    return true;
}

void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    uint8_t command = msg->data;
    if(!checkCommandSequence(command))
    {
        ST.need_homing = true;
        currentDriveMode = STAND_MODE;
        RCLCPP_ERROR(g_node->get_logger(), "安全限制触发：指令顺序错误！触发归零程序。");
        return;
    }
    switch (command)
    {
      case 0x41: currentDriveMode = TORQUE_MODE; break;
      case 0x42: storeSlope(); currentDriveMode = PRE_TORQUE_MODE; break;
      case 0x43: Enc.initialMotorPosition_counter++; StandSafty(); timer.startNewTiming(); break;
      case 0x44:
        if(ST.isForward) { pendingCommand = 0x44; }
        else { currentDriveMode = VELOCITY_MODE; }
        break;
    }
    lastCommand = command;
}

void getUserWeight()
{
    cout << "请输入用户体重(kg,范围40-90): " << endl;
    cin >> user_weight;
    user_weight = std::clamp(user_weight, 40.0, 90.0);
    Tor.TARGET_TORQUE_BASE = user_weight * 0.3;
    cout << "用户体重：" << user_weight << "kg" << endl;
}

double updateTorquePeak()
{
    double RealSwingTime = timer.time4;
    step = (RealSwingTime - timer.TheorSwingTime) * Tor.RealPace * 5;
    Tor.TARGET_TORQUE_FLOAT = step;
    Tor.TARGET_TORQUE_FLOAT = std::clamp(Tor.TARGET_TORQUE_FLOAT, -5.0, 5.0);
    Tor.TARGET_TORQUE_PEAK = Tor.TARGET_TORQUE_BASE + Tor.TARGET_TORQUE_EXT + Tor.TARGET_TORQUE_ASSIST + Tor.TARGET_TORQUE_FLOAT;
    Tor.TARGET_TORQUE_PEAK = std::clamp(Tor.TARGET_TORQUE_PEAK, T_min, T_max);
    if(ST.record) { printf("assist torque: %.2fNM, float torque: %.2fNM, extra torque: %.2fNM, peak: %.2fNM\n", Tor.TARGET_TORQUE_ASSIST, Tor.TARGET_TORQUE_FLOAT, Tor.TARGET_TORQUE_EXT, Tor.TARGET_TORQUE_PEAK); ST.record = false; }
    return Tor.TARGET_TORQUE_PEAK;
}

void getControlMode()
{
    cout << "请输入模式1/2: " << endl;
    int input;
    while (!(cin >> input) || (input != 1 && input != 2)) { cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n'); cout << "输入无效，请重新输入模式1/2: " << endl; }
    control_mode = input;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("can_ankle_node");
  g_node = node;

  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info", 10);
  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>("command_topic", 10, commandCallback);
  auto supportime_one_sub = node->create_subscription<std_msgs::msg::Float64>("one_support_time", 10, one_support_timeCallback);
  auto supportime_two_sub = node->create_subscription<std_msgs::msg::Float64>("two_support_time", 10, two_support_timeCallback);
  auto supportime_three_sub = node->create_subscription<std_msgs::msg::Float64>("three_support_time", 10, three_support_timeCallback);
  auto swingtime_sub = node->create_subscription<std_msgs::msg::Float64>("swing_time", 10, swing_timeCallback);
  auto encoder_sub = node->create_subscription<std_msgs::msg::Float64>("angle", 10, encoderCallback);
  auto forcesensor_sub = node->create_subscription<std_msgs::msg::Float32>("Force", 10, torqueCallback);
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imuCallback);
  auto twist_sub = node->create_subscription<geometry_msgs::msg::Twist>("/system_speed", 10, twistCallback);

  getControlMode();
  getUserWeight();
  pid.resetIntegral();

  Init_Can();
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));
  signal(SIGINT, signalHandler);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
    processHoming();

    velocity_output[0] = 0x03;
    velocity_output[1] = 0x00;
    velocity_output[2] = 0x00;
    velocity_output[3] = 0x00;
    velocity_output[4] = 0x00;

    double y = 0.0;

    if (currentDriveMode == TORQUE_MODE)
    {
        ST.isForward = true;
        ST.record = true;
        if(!ST.initialMotorRecorded)
        {
          if (Enc.initialMotorPosition_counter % 2 == 1)
          { Enc.initialMotorPosition_one = Output_Angle; Enc.initialMotorPosition = Enc.initialMotorPosition_one; cout << "支撑相起始电机角度1已存储" <<endl; ST.initialMotorRecorded = true; }
          else
          { Enc.initialMotorPosition_two = Output_Angle; Enc.initialMotorPosition = Enc.initialMotorPosition_two; cout << "支撑相起始电机角度2已存储" <<endl; ST.initialMotorRecorded = true; }
        }
        if (!isnan(Enc.initialMotorPosition_one) && !isnan(Enc.initialMotorPosition_two))
        {
          if (ST.RetrunInitial)
          {
            y = calculateSupportPosition();
            sendVelocityCommand(y);
            if(Output_Angle - Enc.initialMotorPosition < 1) { ST.RetrunInitial = false; y = 0; sendVelocityCommand(y); }
          }
        }
    }
    else if (currentDriveMode == TORQUE_DRIVE_MODE)
    {
        AdaptiveSpeed(timer.time2);
        double elapsed = timer.getElapsedTime();
        if (control_mode == 1) { y = calculateTargetTorque(); }
        else { y = TargetTorque(); }
        y = std::clamp(y, T_min, T_max);
        adjusted_torque = y;
        sendTorTransVelCommand(y);
        if (elapsed > Tor.DRIVE_DURATION)
        {
            currentDriveMode = STAND_MODE;
            ST.isForward = false;
            if(pendingCommand == 0x44) { currentDriveMode = VELOCITY_MODE; pendingCommand = 0; }
        }
    }
    else if (currentDriveMode == VELOCITY_MODE)
    {
      ST.RetrunInitial = true;
      ST.initialMotorRecorded = false;
      ST.slope_store = false;
      ST.recorded = true;
      rclcpp::sleep_for(std::chrono::milliseconds(15));
      y = calculateTargetPosition();
      sendVelocityCommand(y);
    }
    else if (currentDriveMode == STAND_MODE)
    {
        SendData_five(send_motor_torque, 0x00000353, cfg.para_V);
    }

    auto torque_msg = can_ankle::msg::Torque();
    torque_msg.torque_value = adjusted_torque;
    torque_msg.force_sensortorque = SensorTorque;
    torque_msg.velocity_value = velocity_value;
    torque_msg.return_velocity = Output_VelocityValue;
    torque_msg.return_torque_value = Output_TorqueValue;
    torque_pub->publish(torque_msg);
  }
  if (slope_file.is_open()) { slope_file.close(); }
  rclcpp::shutdown();
  return 0;
}

void sendMSG()
{
    SendData(config_node, 0x00000653, cfg.para1); cout << "指令1发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para2); cout << "指令2发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para3); cout << "指令3发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para4); cout << "指令4发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para5); cout << "指令5发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para6); cout << "指令6发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para7); cout << "指令7发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para8); cout << "指令8发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para9); cout << "指令9发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para10); cout << "指令10发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para11); cout << "指令11发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para12); cout << "指令12发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para13); cout << "指令13发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para14); cout << "指令14发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para15); cout << "指令15发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para16); cout << "指令16发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para17); cout << "指令17发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para18); cout << "指令18发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para19); cout << "指令19发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para20); cout << "指令20发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para21); cout << "指令21发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para22); cout << "指令22发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para23); cout << "指令23发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para24); cout << "指令24发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para25); cout << "指令25发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para26); cout << "指令26发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para27); cout << "指令27发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para28); cout << "指令28发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para29); cout << "指令29发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para30); cout << "指令30发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para31); cout << "指令31发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para32); cout << "指令32发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para33); cout << "指令33发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para34); cout << "指令34发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para35); cout << "指令35发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para36); cout << "指令36发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para37); cout << "指令37发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para38); cout << "指令38发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000653, cfg.para39); cout << "指令39发送完毕" << endl; usleep(200000);
    SendData(config_node, 0x00000000, cfg.para40); cout << "指令40发送完毕" << endl;
}

void motor_on_V()
{
  SendData_five(config_node, 0x00000353, cfg.para_V); cout << "速度模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3); usleep(200000);
  SendData(config_node, 0x00000653, cfg.para_Vmode1); usleep(200000);
  SendData(config_node, 0x00000653, cfg.para_Vmode2); usleep(200000);
  SendData(config_node, 0x00000653, cfg.para_Vmode3); usleep(200000);
  cout << "使能完毕" << endl;
}

void motor_on_P()
{
  SendData_five(config_node, 0x00000353, cfg.para_P); cout << "位置模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3); usleep(200000);
  cout << "使能完毕" << endl;
}

void motor_on_T()
{
  SendData_five(config_node, 0x00000353, cfg.para_T); cout << "扭矩模式发送完毕" << endl;
  SendData(config_node, 0x00000253, cfg.para_motor1); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor2); usleep(200000);
  SendData(config_node, 0x00000253, cfg.para_motor3); usleep(200000);
  cout << "使能完毕" << endl;
}

void Init_Can(void)
{
  int ret;
  int m_run0 = 1;

  if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1)
  {
    RCLCPP_ERROR_STREAM(g_node->get_logger(), "Open CAN deivice error!");
    exit(1);
  }
  RCLCPP_INFO_STREAM(g_node->get_logger(), "CAN connected!");

  config.AccCode = 0x80000008;
  config.AccMask = 0xffffffff;
  config.Filter = 1;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x14; // 1M波特率

  VCI_ResetCAN(VCI_USBCAN2, 0, 0);
  usleep(50000);
  VCI_ClearBuffer(VCI_USBCAN2, 0, 0);
  if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
  {
    RCLCPP_ERROR_STREAM(g_node->get_logger(), "Init CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
  {
    RCLCPP_ERROR_STREAM(g_node->get_logger(), "Start CAN error!");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  RCLCPP_INFO_STREAM(g_node->get_logger(), "CAN started!");
  usleep(100000);
  ret = pthread_create(&threadid, NULL, receive_func, &m_run0);
  sendMSG();
  motor_on_V();
}

void *receive_func(void *param)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("can_recv"), "Start CAN recieve....");
  usleep(20000);
  int reclen = 0;
  VCI_CAN_OBJ rec[200];
  int *run = (int *)param;
  int ind=((*run)>>2);
  int i,j;
  while (rclcpp::ok())
  {
    if ((reclen = VCI_Receive(VCI_USBCAN2, 0 , ind , rec, 200 , 0)) >= 0)
    {
      for (j = 0; j < reclen; j++)
      {
          if (rec[j].ID == 0x000001D3)
          {
            uint8_t rt1 = rec[j].Data[2];
            uint8_t rt2 = rec[j].Data[3];
            uint8_t rp1 = rec[j].Data[4];
            uint8_t rp2 = rec[j].Data[5];
            uint8_t rp3 = rec[j].Data[6];
            uint8_t rp4 = rec[j].Data[7];
            uint16_t combined_torque = (rt2 << 8) | rt1;
            uint32_t combined_position = (rp4 << 24) | (rp3 << 16) | (rp2 << 8) | rp1;
            int16_t torque_output = static_cast<int16_t>(combined_torque);
            Output_TorqueValue = torque_output * (0.12 / 0x0010) * 0.001;
            int32_t position_output = static_cast<int32_t>(combined_position);
            const double MOTOR_CIRCLE = 0x00040000;
            const int GEAR_RATIO = 100;
            Output_Angle = static_cast<double>(position_output) * 360.0 / (MOTOR_CIRCLE * GEAR_RATIO);
          }
          else if (rec[j].ID == 0x000003D3)
          {
            uint8_t rv1 = rec[j].Data[1];
            uint8_t rv2 = rec[j].Data[2];
            uint8_t rv3 = rec[j].Data[3];
            uint8_t rv4 = rec[j].Data[4];
            uint32_t combined_value = (rv4 <<24) | (rv3 <<16) | (rv2 <<8) | rv1;
            int32_t velocity_output_val = static_cast<int32_t>(combined_value);
            const double RPM_SCALE = 0.6 / 0x00040000;
            double speed_rpm = RPM_SCALE * velocity_output_val;
            double speed_deg_per_sec = 6.0 * speed_rpm;
            Output_VelocityValue = speed_deg_per_sec;
          }
      }
    }
  }
  return NULL;
}

void SendData(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = 8;
  for (int i = 0; i < handle_obj.DataLen; i++) { handle_obj.Data[i] = data[i]; }
  if (VCI_Transmit(VCI_USBCAN2, 0, 0, &handle_obj, 1) <= 0) { RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_send"), "Error transmitting!"); }
}

void SendData_five(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = 5;
  for (int i = 0; i < handle_obj.DataLen; i++) { handle_obj.Data[i] = data[i]; }
  if (VCI_Transmit(VCI_USBCAN2, 0, 0, &handle_obj, 1) <= 0) { RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_send"), "Error transmitting!"); }
}

void SendData_two(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data)
{
  handle_obj.ID = id;
  handle_obj.RemoteFlag = 0;
  handle_obj.ExternFlag = 0;
  handle_obj.DataLen = 2;
  for (int i = 0; i < handle_obj.DataLen; i++) { handle_obj.Data[i] = data[i]; }
  if (VCI_Transmit(VCI_USBCAN2, 0, 0, &handle_obj, 1) <= 0) { RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_send"), "Error transmitting!"); }
}

double calculateTargetTorque()
{
    double elapsed = timer.getElapsedTime();
    double torque = 0.0;
    Tor.TARGET_TORQUE_PEAK = updateTorquePeak();
    double rise_phase_start = Tor.TARGET_TORQUE_RISE_TIME;
    double fall_phase_end = Tor.TARGET_TORQUE_RISE_TIME + Tor.TARGET_TORQUE_FALL_TIME;
    if (elapsed < rise_phase_start) { double phase = M_PI * elapsed / Tor.TARGET_TORQUE_RISE_TIME; torque = (1 - cos(phase)) * (Tor.TARGET_TORQUE_PEAK) / 2; }
    else if (elapsed < fall_phase_end) { double t_fall = elapsed - Tor.TARGET_TORQUE_RISE_TIME; double phase = M_PI * t_fall / Tor.TARGET_TORQUE_FALL_TIME; torque = (1 + cos(phase)) / 2 * Tor.TARGET_TORQUE_PEAK; }
    return torque;
}

double TargetTorque()
{
    double elapsed = timer.getElapsedTime();
    double torque = 0.0;
    Tor.TARGET_TORQUE_PEAK = Tor.TARGET_TORQUE_BASE;
    double rise_phase = 0.18;
    double fall_phase = 0.21;
    if (elapsed < rise_phase) { double phase = M_PI * elapsed / rise_phase; torque = (1 - cos(phase)) / 2 * Tor.TARGET_TORQUE_PEAK; }
    else if (elapsed < rise_phase + fall_phase) { double t_fall = elapsed - rise_phase; double phase = M_PI * t_fall / fall_phase; torque = (1 + cos(phase)) * (Tor.TARGET_TORQUE_PEAK) / 2; }
    return torque;
}

double calculateTargetPosition()
{
    auto now = high_resolution_clock::now();
    double theta_m = Enc.position_angle;
    double theta_d = Enc.initialEncoderValue;
    double dt = duration_cast<duration<double>>(now - pos_controller.prev_time).count();
    if (dt < 1e-6) dt = 1e-6;
    double theta_m_dot = (theta_m - pos_controller.prev_theta_m) / dt;
    double forcesensor = SensorTorque / 0.12;
    double theta_d_dot = 0;
    if (forcesensor > 0) { if(theta_d < theta_m) { theta_d_dot = pos_controller.K_theta * (theta_d - theta_m) * forcesensor * 1.0; } }
    theta_d_dot = std::clamp(theta_d_dot, V_min, V_max);
    pos_controller.prev_theta_m = theta_m;
    pos_controller.prev_time = now;
    return theta_d_dot;
}

double calculateSupportPosition()
{
    auto now = high_resolution_clock::now();
    double theta_m = Output_Angle;
    double theta_d = (Enc.initialMotorPosition_counter % 2 == 1) ? Enc.initialMotorPosition_two : Enc.initialMotorPosition_one;
    double dt = duration_cast<duration<double>>(now - startpos_controller.prev_time).count();
    if (dt < 1e-6) dt = 1e-6;
    double theta_m_dot = (theta_m - startpos_controller.prev_theta_m) / dt;
    double theta_d_dot = startpos_controller.K_theta * (theta_d - theta_m) + startpos_controller.K_d * theta_m_dot;
    theta_d_dot = std::clamp(theta_d_dot, V_min, V_max);
    startpos_controller.prev_theta_m = theta_m;
    startpos_controller.prev_time = now;
    return theta_d_dot;
}
