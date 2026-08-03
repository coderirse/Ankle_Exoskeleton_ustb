#include "can_ankle/msg/torque.hpp"
#include "can_ankle/can_ankle_node.h"
// controlcan.h replaced by can subprocess in header

// 参数宏定义
uint8_t pendingCommand = 0;      // 等待执行的指令
const double RETURN_VELOCITY = 8;        // 归零速度(度/s)
double step = 0;
uint8_t byte0;
uint8_t byte1;
uint8_t byte2;
uint8_t byte3;
uint8_t lastCommand;
double user_weight = 0;//用户体重
int control_mode = 0;//控制模式
bool slope_file_initialized = false;
std::ofstream slope_file;

// 2026-07-29 台架参数 (由ROS参数覆盖)
double MOTOR_DIR = 1.0;          // 电机方向: +1收线拉紧, -1放线
double FORCE_LIMIT = 5.0;        // 力限值(/Force尺度, 超限立即停)
double FORCE_EMERGENCY = 35.0;   // 力传感器硬限值 kg, 超限急停锁存(防鲍登线绷断)
double TORQUE_PER_FORCE = 6.0;   // Nm per /Force单位 (台架杠杆比标定)
double PRELOAD_SPEED = 10.0;     // 预紧爬速 deg/s
double PRELOAD_FORCE = 0.2;      // 预紧目标力 (旧参数, 已被min/max取代)
double PRELOAD_FORCE_MIN = 0.5;  // 预紧目标力下限 kg (站立预紧完成后应在此范围)
double PRELOAD_FORCE_MAX = 2.0;  // 预紧目标力上限 kg
double PRELOAD_TIMEOUT = 4.0;    // 预紧超时 s
double STAND_CONFIRM_TIME = 2.0; // 初始化条件全部满足所需持续时间 s
double LEVEL_PITCH_LIMIT = 15.0; // 脚面水平俯仰角容差 deg
double LEVEL_ENCODER_TARGET = 0.0;  // 脚面水平时的编码器目标角度 deg
double LEVEL_ENCODER_LIMIT = 10.0;  // 编码器角度容差 deg
double INIT_TIMEOUT = 30.0;      // 初始化对准总超时 s (超时告警后继续)
double MAX_SPEED = 180.0;        // 速度限幅 deg/s (替换V_max)
float  g_force_value = 0.0;      // 原始力传感器值(/Force)

// 急停锁存: 力超过 FORCE_EMERGENCY 后只发零速, 需重启恢复
bool emergency_stop = false;

// 足底开关原始状态跟踪(用于站立确认, 不受指令顺序校验影响)
uint8_t last_switch_command = 0;                            // 最近一次收到的开关指令
std::chrono::steady_clock::time_point last_switch_time;     // 收到该指令的时刻

// IMU数据跟踪
bool imu_received = false;
std::chrono::steady_clock::time_point last_imu_time;

// 前向声明
void send_speed(double deg_s);

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
    BYTE para_Vmode1[8] = {0x23, 0x85, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};//位置转速度模式
    BYTE para_Vmode2[8] = {0x23, 0x83, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};
    BYTE para_Vmode3[8] = {0x23, 0x84, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x3F};
    BYTE para_Pmode1[8] = {0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};//速度转位置模式
    BYTE para_Pmode2[8] = {0x23, 0x83, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};
    BYTE para_Pmode3[8] = {0x23, 0x84, 0x60, 0x00, 0x00, 0x00, 0x0F, 0x00};
};
ConfigNode cfg;

// using std::clamp

//状态变量
class state_parameter
{
public:
    bool isForward = false; // 用于标记是否正转
    bool need_homing = false; // 归零状态标志
    bool initialEncoderRecorded = false; //起始位置记录
    bool initialMotorRecorded = false; //支撑相起始位置记录
    bool sendV_mode = true; //是否为速度模式
    bool sendP_mode = false; //是否为位置模式
    bool recorded = true; //记录驱动时间参数
    bool record = true; //记录峰值扭矩参数
    bool slope_store = false; //记录坡度参数
    bool RetrunInitial = false; //进入支撑相返回初始位置记录
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
    double initialMotorPosition_one = NAN;  // 单数次支撑相开始时电机的位置
    double initialMotorPosition_two = NAN;  // 双数次支撑相开始时电机的位置
    double initialMotorPosition = 0;  //支撑相开始时电机位置
    int initialMotorPosition_counter = 0;   // 支撑相计数器
    double SwingMotorPosition = 0;     //摆动相电机位置
};
encoder_parameter Enc;

//支撑相扭矩驱动参数信息
class Torque_parameter
{
public:
    double DRIVE_DURATION =0; // 驱动持续时间(支撑相阶段3时间)
    double DRIVE_DELAY_DURATION = 0; // 延时时间(支撑相阶段2)
    double TARGET_TORQUE_RISE_TIME = 0; // 上升时间
    double TARGET_TORQUE_FALL_TIME = 0; // 下降时间
    double TARGET_TORQUE_BASE = 0; // 基准扭矩（根据体重）
    double TARGET_TORQUE_FLOAT = 0; // 浮动扭矩（根据步长）
    double TARGET_TORQUE_ASSIST = 0; //辅助扭矩（根据坡度）
    double TARGET_TORQUE_EXT = 0; //额外扭矩（根据速度）
    double TARGET_TORQUE_PEAK = 0; // 总峰值扭矩
    double PRE_TORQUE = 0; // 预张紧力
    double CompensatePosition = 0; //补偿速度
    double RealPace = 0; //解算出的真实步速
    double slope = 0;//坡度
};
Torque_parameter Tor;

//时间相关函数
class Timer 
{
public:
    double time1 = 0; // 支撑相1时间
    double time2 = 0; // 支撑相2时间
    double time3 = 0; // 支撑相3时间
    double time4 = 0; // 摆动相时间
    double TheorSwingTime = 0; //理论摆动相时间
    Timer() : lastCommandTime(high_resolution_clock::now()), 
              lastDtTime(high_resolution_clock::now()),
              lastLoopTime(high_resolution_clock::now()) {}

    // 重置计时起点
    void startNewTiming() 
    {
        lock_guard<mutex> lock(mutex_);
        lastCommandTime = high_resolution_clock::now();
    }

    // 获取自上次startNewTiming的 elapsed 时间
    double getElapsedTime() 
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        return duration_cast<duration<double>>(now - lastCommandTime).count();
    }

    // 实时dt（每次调用更新时间戳，用于高频控制）
    double getRealTimeDt() 
    {
        lock_guard<mutex> lock(mutex_);
        auto now = high_resolution_clock::now();
        double dt = duration_cast<duration<double>>(now - lastDtTime).count();
        lastDtTime = now;
        return dt > 1e-6 ? dt : 1e-6;
    }
private:
    time_point<high_resolution_clock> lastCommandTime;  // 全局计时起点
    time_point<high_resolution_clock> lastDtTime;       // 实时dt计时起点
    time_point<high_resolution_clock> lastLoopTime;     // 主循环dt计时起点
    mutable mutex mutex_;
};
Timer timer;

//IMU相关信息
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
        double linear_velocity_x = 0; //x轴线速度
        double linear_velocity_y = 0; //y轴线速度
        double linear_velocity_z = 0; //z轴线速度
        double current_linear_velocity_y = 0;///y轴线速度取反
    };
IMU_parameter imu;

//状态机模式
enum DriveMode {VELOCITY_MODE, TORQUE_MODE, PRE_TORQUE_MODE, TORQUE_DRIVE_MODE , STAND_MODE};
DriveMode currentDriveMode = STAND_MODE;

// PID跟踪控制器（扭矩模式）
struct PIDController 
{
    double Kp;       // 扭矩误差比例增益
    double Ki;       // 积分增益
    double Kd;       // 微分增益
    double integral; // 积分累积项
    double prev_setpoint;    // 上一循环目标扭矩
    double prev_measurement; // 上一循环真实扭矩

    PIDController(double kp, double ki, double kd) 
        : Kp(kp), Ki(ki), Kd(kd), integral(0.0), 
          prev_setpoint(0.0), prev_measurement(0.0) {}

    double compute(double setpoint, double measurement, double dt) {
        if (dt <= 1e-6) dt = 1e-6;  // 防止除零

        // 计算误差
        double error = setpoint - measurement;

        // 计算微分项：(Δ目标扭矩/dt - Δ真实扭矩/dt)
        double delta_setpoint = setpoint - prev_setpoint;
        double delta_measurement = measurement - prev_measurement;
        double derivative = (delta_setpoint / dt) - (delta_measurement / dt);

        // 积分项累加（带抗饱和限制）
        integral += error * dt;

        // 计算总输出
        double output = Kp * error + Ki * integral + Kd * derivative;
        output = std::clamp(output, V_min, V_max);  // 输出限幅

        // 更新历史值
        prev_setpoint = setpoint;
        prev_measurement = measurement;

        return output;
    }

    void resetIntegral() 
    {
      if (!TORQUE_MODE)
      {
        integral = 0.0;  // 模式切换时重置积分项
      }  
    }
    void updateParamsBasedOnSpeed(double speed) 
    {
        if (speed < 2) 
        {
            Kp = 20;
            Ki = 0.003;
            Kd = 0.2;
        } 
        else if (speed >= 2 && speed < 3) 
        {
            Kp =30;
            Ki = 0.004;
            Kd = 0.3;
        } 
        else if (speed >= 3 && speed < 4) 
        {
            Kp = 40;
            Ki = 0.005;
            Kd = 0.4;
        } 
        else if (speed >= 4 && speed < 5) 
        {
            Kp = 50;
            Ki = 0.006;
            Kd = 0.5;
        } 
        else if (speed > 5) 
        {
            Kp = 60;
            Ki = 0.007;
            Kd = 0.6;
        }
    }
};

// 初始化PD参数
PIDController pid(2 , 0.001 , 0.1);  
 
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
PositionController pos_controller(2 , 0.02);

//位置补偿
struct StartPositionController 
{
    double K_theta;  // 位置比例增益
    double K_d;      // 速度阻尼增益
    double prev_theta_m; // 上一时刻的测量位置
    time_point<high_resolution_clock> prev_time; // 上一时刻的时间戳

    StartPositionController(double kt, double kd) : K_theta(kt), K_d(kd), prev_theta_m(0), prev_time(high_resolution_clock::now()) {}
};

//补偿参数
StartPositionController startpos_controller(3 , 0.03);

//关闭节点
void signalHandler(int signum)
{
  cout << "启动归零程序..." << endl;
  ST.need_homing = true; // 设置标志位
  (void)signum;
  can_close();
}

//实时速度解算函数(自适应步速)
void AdaptiveSpeed(double x)
{
  //输入支撑相时间2
  const double a1 = 1.45022;
  const double b1 = 0.4695;
  const double c1 = 0.06345;
  const double K = 5;
  double i1 = (x - c1) / a1;
  double j1 = - log(i1) / b1;
  Tor.RealPace = j1; //解算实时速度
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
    if (Tor.RealPace > 6)
    {
      Tor.DRIVE_DURATION = 0.2949216 * 1.3;
      Tor.TARGET_TORQUE_EXT = K;
    }
   else
    {
      Tor.DRIVE_DURATION = 0.8;
      Tor.TARGET_TORQUE_EXT = K;
    }
  }
  double R = 0.035 * Tor.RealPace  + 0.5075; //上升时间所占比例
  Tor.TARGET_TORQUE_RISE_TIME = Tor.DRIVE_DURATION * R ;
  R = std::clamp(R , 0.4 , 0.7);
  Tor.TARGET_TORQUE_FALL_TIME = Tor.DRIVE_DURATION - Tor.TARGET_TORQUE_RISE_TIME;
  //根据实际情况调整支撑相3的参数
  Tor.DRIVE_DURATION = std::clamp(Tor.DRIVE_DURATION , min_duration , max_duration);
  if(ST.recorded)
  {
    cout << "time2:" << x << "realpace:" << Tor.RealPace <<"drive time:" << Tor.DRIVE_DURATION  << "R value:" << R << endl;
    ST.recorded = false;
  }
  const double a3 = 1.2536604201;
  const double b3 = 0.5741810332;
  const double c3 = 0.4461945834;
  timer.TheorSwingTime = a3 * exp(-b3 * Tor.RealPace) + c3;
  //更新pid参数
  pid.updateParamsBasedOnSpeed(Tor.RealPace);
}

// 退出归零函数
void processHoming() 
{
    bool is_homing = false;
    if (ST.need_homing && !is_homing) 
    {
        is_homing = true;
        // 发送归零指令
        double return_velocity = (Enc.initialEncoderValue > Enc.Encoder_Value) ? RETURN_VELOCITY : -RETURN_VELOCITY;
        send_speed(return_velocity);
        usleep(50000);
        while(abs(Enc.Encoder_Value - Enc.initialEncoderValue) < 2 && rclcpp::ok()) 
        {   // 关闭流程
            send_speed(0);
            usleep(200000);
            cout << "关闭节点" << endl;
            BYTE sd[] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
            SendData(config_node, 0x00000653, sd); // 停止电机
            usleep(200000);
            rclcpp::shutdown();
            exit(0);
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
  g_force_value = msg->data;
  SensorTorque = g_force_value * TORQUE_PER_FORCE;  // Nm (台架杠杆比)
  // 硬限值急停: 超过35kg立即停机并锁存, 防鲍登线绷断
  if (fabs(g_force_value) >= FORCE_EMERGENCY)
  {
    emergency_stop = true;
    currentDriveMode = STAND_MODE;
    send_speed(0.0);
    RCLCPP_ERROR(rclcpp::get_logger("ankle"), "急停! 力 %.2f kg 超过硬限值 %.1f kg, 电机已锁停, 请检查系统后重启",
                 g_force_value, FORCE_EMERGENCY);
    return;
  }
  // 力保护: 超限立即停电机回STAND
  if (fabs(g_force_value) >= FORCE_LIMIT)
  {
    currentDriveMode = STAND_MODE;
    send_speed(0.0);
    RCLCPP_WARN(rclcpp::get_logger("ankle"), "力超限! %.2f >= %.2f, 电机停止", g_force_value, FORCE_LIMIT);
  }
}

//支撑相阶段1时间回调函数
void one_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    timer.time1 = msg->data;
}

//支撑相阶段2时间回调函数
void two_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    timer.time2 = msg->data;
}

//支撑相阶段3时间回调函数
void three_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    timer.time3 = msg->data;
}

//摆动相时间回调函数
void swing_timeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    timer.time4 = msg->data;
}

//imu线速度回调函数
void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  imu.linear_velocity_y = msg->linear.y;
  imu.current_linear_velocity_y = -imu.linear_velocity_y;
}

//imu数据存储函数
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

//imu角速度和线加速度回调函数
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    extractImuData(msg);
    imu_received = true;
    last_imu_time = std::chrono::steady_clock::now();
}

//补偿位置速度输出函数
void updateCompensateTorque()
{
    if (isnan(Enc.initialMotorPosition_one))return;
    if (isnan(Enc.initialMotorPosition_two)) return;
    const double ANGLE_TOLERANCE = 0.5;
    double angleDiff = 0;
    double TransV = 0;

    // 根据计数器判断比较规则
    bool even_cycle = (Enc.initialMotorPosition_counter / 2) % 2 == 0;
    double a = Enc.initialMotorPosition_two - Enc.initialMotorPosition_one;
    double b = Enc.initialMotorPosition_one - Enc.initialMotorPosition_two;
    if (even_cycle) 
    {  //偶数次
        Tor.CompensatePosition = a;
        angleDiff = Output_Angle - Enc.initialMotorPosition_one;
        TransV = angleDiff  * 2;
        send_speed(TransV);
    } 
    else 
    {  //奇数次
        Tor.CompensatePosition = b;
        angleDiff = Output_Angle - Enc.initialMotorPosition_two;
        TransV = angleDiff *2;
        send_speed(TransV);
    }

    RCLCPP_INFO(rclcpp::get_logger("ankle"), "补偿角度： %f", Tor.CompensatePosition);
}

// 站立模式函数
void StandSafty()
{
  if(timer.time2 > 5)
  {
    currentDriveMode = STAND_MODE;
    signal(SIGINT, signalHandler);
  }
  else
  {
    currentDriveMode = TORQUE_DRIVE_MODE;
  }
}

//速度模式指令转换函数
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

// 2026-07-29 新电机SDO速度指令: 0x60FF, uu/s = deg/s × 600 × MOTOR_DIR
void send_speed(double deg_s)
{
    int32_t uu = (int32_t)(deg_s * MOTOR_DIR * 600.0);
    BYTE d[8] = {0x23, 0xFF, 0x60, 0x00,
                 (BYTE)(uu & 0xFF), (BYTE)((uu >> 8) & 0xFF),
                 (BYTE)((uu >> 16) & 0xFF), (BYTE)((uu >> 24) & 0xFF)};
    SendData(config_node, 0x00000653, d);
}

//发送扭矩跟踪转换电机速度指令函数
void sendTorTransVelCommand(double y)
{
    torque_value = y;
    double dt = timer.getRealTimeDt(); // 从Timer获取dt
    adjusted_velocity = pid.compute(torque_value, SensorTorque, dt); //PID控制
    velocity_value = adjusted_velocity;//发送的速度值
    send_speed(velocity_value);
}

//直接发送速度指令函数
void sendVelocityCommand(double y) 
{
  velocity_value = y;
  send_speed(velocity_value);
}

//坡度储存函数
void storeSlope() 
{
    double orientationZ = 0;
    if(!ST.slope_store)
    {
      orientationZ = imu.imu_orientation_z;
      ST.slope_store = true;
    }
    if (orientationZ > 180) 
    {
        orientationZ -= 360;
    }
    double supportSlope = orientationZ;
    Tor.slope = 0 - supportSlope;

    // 根据坡度分配辅助扭矩
    const double maxTorque = 5;
    const double minTorque = -5;
    Tor.TARGET_TORQUE_ASSIST = 8 / M_PI * atan(Tor.slope / 2) ;
    Tor.TARGET_TORQUE_ASSIST = std::clamp(Tor.TARGET_TORQUE_ASSIST, minTorque, maxTorque);
    cout << "Slope: " << Tor.slope << endl;
    if (slope_file.is_open())
    {
        slope_file << Tor.slope << "\n";
        slope_file.flush(); // 立即写入
        RCLCPP_INFO(rclcpp::get_logger("ankle"), "坡度值已写入文件: %.2f", Tor.slope);
    }
}

//坡度储存函数
void initializeSlopeFile()
{
    if (!slope_file_initialized)
    {
        // 使用绝对路径确保文件在正确位置创建
        std::string home_path = getenv("HOME") ? getenv("HOME") : ".";
        std::string file_path = home_path + "/Slope.txt";
        
        // 打开文件，如果文件不存在则创建，每次运行覆盖旧内容
        slope_file.open(file_path.c_str(), std::ios::out | std::ios::trunc);
        
        if (slope_file.is_open())
        {
            slope_file << "坡度值\n";
            slope_file << "Slope (degrees)\n";
            slope_file.flush();
            slope_file_initialized = true;
            RCLCPP_INFO(rclcpp::get_logger("ankle"), "path: %s", file_path.c_str());
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("ankle"), "无法打开坡度文件！路径: %s", file_path.c_str());
        }
    }
}

//步态紊乱安全模式函数（更新版）
bool checkCommandSequence(uint8_t currentCommand) 
{
    // 如果是第一次接收到指令，直接通过
    if (lastCommand == 0) 
    {
        return true;
    }
    
    switch (currentCommand) 
    {
        case 0x41:
            // 0x41应该紧跟在0x44后面
                return true;
            break;
        case 0x42:
            if (lastCommand != 0x41) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("ankle"), "指令顺序错误：0x42不应在0x%02X之后！", lastCommand);
                return false;
            }
            break;
        case 0x43:
            if (lastCommand != 0x42) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("ankle"), "指令顺序错误：0x43不应在0x%02X之后！", lastCommand);
                return false;
            }
            break;
        case 0x44:
            if (lastCommand != 0x43) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("ankle"), "指令顺序错误：0x44不应在0x%02X之后！", lastCommand);
                return false;
            }
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("ankle"), "未知指令：0x%02X", currentCommand);
            return false;
            break;
    }
    return true;
}

//脚底开关指令接收函数
void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    uint8_t command = msg->data;

    // 记录原始开关状态与时间(站立确认用, 不受顺序校验影响)
    if (command != last_switch_command)
    {
        last_switch_command = command;
        last_switch_time = std::chrono::steady_clock::now();
    }

    // 添加安全检查，如果指令顺序错误，立即触发归零并返回
    if(!checkCommandSequence(command))
    {
        // 设置归零标志
        ST.need_homing = true;
        // 设置为站立模式（停止驱动）
        currentDriveMode = STAND_MODE;
        RCLCPP_ERROR(rclcpp::get_logger("ankle"), "安全限制触发：指令顺序错误！触发归零程序。");
        return; // 直接返回，不处理当前指令
    }
    
    switch (command) 
    {
      case 0x41:
        currentDriveMode = TORQUE_MODE;
        break;
      case 0x42:
        storeSlope();
        currentDriveMode = PRE_TORQUE_MODE;
        break;
      case 0x43:
        Enc.initialMotorPosition_counter++;
        StandSafty();
        timer.startNewTiming();
        break;
      case 0x44:
        if(ST.isForward)
        {
            pendingCommand = 0x44;
        }
        else
        {
            currentDriveMode = VELOCITY_MODE;
        }
        break;
    }
    lastCommand = command;
}

//体重转换峰值扭矩函数
void getUserWeight() 
{
    cout << "请输入用户体重(kg,范围40-90): " << endl;
    cin >> user_weight;
    // 限制体重范围并计算基准扭矩
    user_weight = std::clamp(user_weight, 40.0, 90.0);
    Tor.TARGET_TORQUE_BASE = user_weight * 0.3;
    cout << "用户体重：" << user_weight << "kg" << endl;
}

//动态调整峰值扭矩函数
double updateTorquePeak() 
{
    double RealSwingTime = timer.time4;
    double a = RealSwingTime - timer.TheorSwingTime;
    // 动态调整扭矩
    if (a > 0) 
    {
        step = (RealSwingTime - timer.TheorSwingTime) * Tor.RealPace * 5;
    } 
    else 
    {
        step = (RealSwingTime - timer.TheorSwingTime) * Tor.RealPace * 5;
    }
    Tor.TARGET_TORQUE_FLOAT = step;
    Tor.TARGET_TORQUE_FLOAT = std::clamp(Tor.TARGET_TORQUE_FLOAT , -5.0 , 5.0);
    Tor.TARGET_TORQUE_PEAK = Tor.TARGET_TORQUE_BASE + Tor.TARGET_TORQUE_EXT + Tor.TARGET_TORQUE_ASSIST + Tor.TARGET_TORQUE_FLOAT;
    Tor.TARGET_TORQUE_PEAK = std::clamp(Tor.TARGET_TORQUE_PEAK , T_min , T_max);
    // 输出调试信息
    if(ST.record)
    {
      printf("assist torque: %.2fNM,float torque: %.2fNM", Tor.TARGET_TORQUE_ASSIST, Tor.TARGET_TORQUE_FLOAT);
      printf("extra torque: %.2fNM,final peak torque: %.2fNM", Tor.TARGET_TORQUE_EXT, Tor.TARGET_TORQUE_PEAK);
      cout <<" "<< endl;
      ST.record = false;
    }
    
    return Tor.TARGET_TORQUE_PEAK;
}

// 模式选择输入函数
void getControlMode()
{
    cout << "请输入模式1/2: " << endl;
    int input;
    // 循环验证输入合法性（必须是1或2）
    while (!(cin >> input) || (input != 1 && input != 2))
    {
        cin.clear();  // 清除输入错误状态
        // 忽略缓冲区中剩余的无效输入
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "输入无效，请重新输入模式1/2: " << endl;
    }
    control_mode = input;
    cout << "已选择模式: " << control_mode << endl;
}

// IMU四元数转俯仰角 pitch = asin(2*(w*y - x*z)), 返回角度deg
double getImuPitchDeg()
{
    double w = imu.imu_orientation_w, x = imu.imu_orientation_x;
    double y = imu.imu_orientation_y, z = imu.imu_orientation_z;
    double sinp = std::clamp(2.0 * (w * y - x * z), -1.0, 1.0);
    return asin(sinp) * 180.0 / M_PI;
}

// 初始化对准:
//   力控闭环维持力传感器在 [PRELOAD_FORCE_MIN, PRELOAD_FORCE_MAX] kg
//   (线松→收线, 线紧→放线, 跟随脚放平自动调整)
//   当 开关闭合(0x42) + 力在范围 + IMU水平 + 编码器在范围 全部满足
//   并持续 STAND_CONFIRM_TIME 秒 → 初始化完成
// 返回 false 表示被急停/关闭/超时中断
bool runInitialAlignment(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "初始化对准: 维持力 %.1f~%.1f kg, 等待 [开关闭合+力正常+IMU水平+编码器正常] 持续 %.1f s...",
        PRELOAD_FORCE_MIN, PRELOAD_FORCE_MAX, STAND_CONFIRM_TIME);

    auto t_start = std::chrono::steady_clock::now();
    auto t_ok = std::chrono::steady_clock::now();
    bool all_ok_prev = false;
    bool warned_no_imu = false;
    int status_div = 0;

    while (rclcpp::ok() && !emergency_stop)
    {
        rclcpp::spin_some(node);

        // ── 力控: 维持力传感器在目标区间 ──
        if (g_force_value < PRELOAD_FORCE_MIN)
            send_speed(PRELOAD_SPEED);        // 线太松 → 收线
        else if (g_force_value > PRELOAD_FORCE_MAX)
            send_speed(-PRELOAD_SPEED);       // 线太紧 → 放线
        else
            send_speed(0.0);

        // ── 各项条件检查 ──
        bool switch_ok  = (last_switch_command == 0x42);   // 双脚开关同时闭合
        bool force_ok   = (g_force_value >= PRELOAD_FORCE_MIN && g_force_value <= PRELOAD_FORCE_MAX);
        bool enc_ok     = (fabs(Enc.Encoder_Value - LEVEL_ENCODER_TARGET) <= LEVEL_ENCODER_LIMIT);

        double pitch_deg = 0.0;
        bool imu_ok = true;
        if (imu_received)
        {
            pitch_deg = getImuPitchDeg();
            imu_ok = (fabs(pitch_deg) <= LEVEL_PITCH_LIMIT);
        }
        else if (!warned_no_imu)
        {
            RCLCPP_WARN(node->get_logger(), "无IMU数据, 水平检测仅使用编码器");
            warned_no_imu = true;
        }

        bool all_ok = switch_ok && force_ok && enc_ok && imu_ok;

        // ── 全条件持续时间判定 ──
        if (all_ok)
        {
            if (!all_ok_prev) { t_ok = std::chrono::steady_clock::now(); all_ok_prev = true; }
            double held = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_ok).count() / 1000.0;
            if (held >= STAND_CONFIRM_TIME)
            {
                send_speed(0.0);
                RCLCPP_INFO(node->get_logger(),
                    "初始化完成: force=%.2f kg, pitch=%.2f deg, encoder=%.2f deg, 稳定保持 %.1f s",
                    g_force_value, pitch_deg, Enc.Encoder_Value, held);
                printf("========================================\n");
                printf("  初始化完成! 力=%.2f kg  俯仰=%.2f deg\n", g_force_value, pitch_deg);
                printf("  编码器=%.2f deg  稳定保持 %.1f s\n", Enc.Encoder_Value, held);
                printf("========================================\n");
                return true;
            }
        }
        else
        {
            all_ok_prev = false;
        }

        // 每0.5s打印一次各条件状态, 便于现场判断卡在哪一项
        if (++status_div >= 100)
        {
            status_div = 0;
            RCLCPP_INFO(node->get_logger(),
                "对准中: 开关[%s] 力[%s %.2f] 编码器[%s %.2f] IMU[%s %.2f]",
                switch_ok ? "OK" : "等待", force_ok ? "OK" : "调整", g_force_value,
                enc_ok ? "OK" : "偏差", Enc.Encoder_Value,
                imu_received ? (imu_ok ? "OK" : "倾斜") : "无数据", pitch_deg);
        }

        // 总超时保护
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t_start).count() / 1000.0;
        if (elapsed > INIT_TIMEOUT)
        {
            send_speed(0.0);
            RCLCPP_WARN(node->get_logger(), "初始化对准超时 %.0f s, 放弃等待继续运行", INIT_TIMEOUT);
            return false;
        }

        usleep(5000);
    }
    send_speed(0.0);
    return false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ankle_node");
  auto torque_pub = node->create_publisher<can_ankle::msg::Torque>("torque_info", 10); //发布各项信息
  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>("command_topic", 10, commandCallback);
  auto supportime_one_sub = node->create_subscription<std_msgs::msg::Float64>("one_support_time", 10, one_support_timeCallback);
  auto supportime_two_sub = node->create_subscription<std_msgs::msg::Float64>("two_support_time", 10, two_support_timeCallback);
  auto supportime_three_sub = node->create_subscription<std_msgs::msg::Float64>("three_support_time", 10, three_support_timeCallback);
  auto swingtime_sub = node->create_subscription<std_msgs::msg::Float64>("swing_time", 10, swing_timeCallback);
  auto encoder_sub = node->create_subscription<std_msgs::msg::Float64>("angle", 10, encoderCallback);
  auto forcesensor_sub = node->create_subscription<std_msgs::msg::Float32>("Force", 10, torqueCallback);
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imuCallback);
  auto twist_sub = node->create_subscription<geometry_msgs::msg::Twist>("/system_speed", 10, twistCallback);
  //initializeSlopeFile();
  // 2026-07-29: 用ROS参数替代cin交互输入
  node->declare_parameter<int>("control_mode", 1);
  node->declare_parameter<double>("user_weight", 60.0);
  node->declare_parameter<double>("force_limit", 5.0);
  node->declare_parameter<double>("torque_per_force", 6.0);
  node->declare_parameter<double>("motor_dir", 1.0);
  node->declare_parameter<double>("preload_speed", 10.0);
  node->declare_parameter<double>("preload_force", 0.2);
  node->declare_parameter<double>("preload_timeout", 4.0);
  node->declare_parameter<double>("max_speed", 180.0);
  node->declare_parameter<double>("force_emergency", 35.0);
  node->declare_parameter<double>("preload_force_min", 0.5);
  node->declare_parameter<double>("preload_force_max", 2.0);
  node->declare_parameter<double>("stand_confirm_time", 2.0);
  node->declare_parameter<double>("level_pitch_limit", 15.0);
  node->declare_parameter<double>("level_encoder_target", 0.0);
  node->declare_parameter<double>("level_encoder_limit", 10.0);
  node->declare_parameter<double>("init_timeout", 30.0);
  control_mode = node->get_parameter("control_mode").as_int();
  user_weight = node->get_parameter("user_weight").as_double();
  user_weight = std::clamp(user_weight, 40.0, 90.0);
  FORCE_LIMIT = node->get_parameter("force_limit").as_double();
  TORQUE_PER_FORCE = node->get_parameter("torque_per_force").as_double();
  MOTOR_DIR = node->get_parameter("motor_dir").as_double();
  PRELOAD_SPEED = node->get_parameter("preload_speed").as_double();
  PRELOAD_FORCE = node->get_parameter("preload_force").as_double();
  PRELOAD_TIMEOUT = node->get_parameter("preload_timeout").as_double();
  MAX_SPEED = node->get_parameter("max_speed").as_double();
  FORCE_EMERGENCY = node->get_parameter("force_emergency").as_double();
  PRELOAD_FORCE_MIN = node->get_parameter("preload_force_min").as_double();
  PRELOAD_FORCE_MAX = node->get_parameter("preload_force_max").as_double();
  STAND_CONFIRM_TIME = node->get_parameter("stand_confirm_time").as_double();
  LEVEL_PITCH_LIMIT = node->get_parameter("level_pitch_limit").as_double();
  LEVEL_ENCODER_TARGET = node->get_parameter("level_encoder_target").as_double();
  LEVEL_ENCODER_LIMIT = node->get_parameter("level_encoder_limit").as_double();
  INIT_TIMEOUT = node->get_parameter("init_timeout").as_double();
  V_max = MAX_SPEED; V_min = -MAX_SPEED;
  Tor.TARGET_TORQUE_BASE = user_weight * 0.3;
  RCLCPP_INFO(node->get_logger(), "控制参数: mode=%d 体重=%.1f 力限=%.1f 急停限=%.1fkg t_per_f=%.2f motor_dir=%.0f max_speed=%.1f",
              control_mode, user_weight, FORCE_LIMIT, FORCE_EMERGENCY, TORQUE_PER_FORCE, MOTOR_DIR, MAX_SPEED);

  pid.resetIntegral();//清除pd积分项

  //初始化can节点 (TCP连接can_bridge.py, 已完成使能)
  Init_Can();

  // 2026-08-03: 初始化对准 — 力控维持0.5~2.0kg随脚放平收放线,
  // 开关闭合+力正常+IMU水平+编码器正常 全部满足持续2s后初始化完毕
  {
    bool init_ok = runInitialAlignment(node);
    if (!init_ok && emergency_stop)
    {
      RCLCPP_ERROR(node->get_logger(), "初始化期间触发急停, 不进入正常控制");
    }
    else if (!init_ok)
    {
      RCLCPP_WARN(node->get_logger(), "初始化未完全确认 (超时/中断), 继续运行");
    }
  }

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));  // 200Hz
  signal(SIGINT, signalHandler);
  RCLCPP_INFO(node->get_logger(), "进入主循环, 初始模式=STAND_MODE");
  int loop_count = 0;
  //发送指令
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();

    processHoming();//是否归零

    double y = 0.0;

    // 急停锁存: 力超硬限值后所有状态失效, 只发零速
    if (emergency_stop)
    {
        send_speed(0.0);
    }
    // 状态机核心
    else if (currentDriveMode == TORQUE_MODE)
    {
        ST.isForward = true;
        ST.record = true;
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
          if (ST.RetrunInitial)
          {
            y = calculateSupportPosition();
            sendVelocityCommand(y);
            if(Output_Angle - Enc.initialMotorPosition < 1)
            {
              ST.RetrunInitial = false;
              y = 0;
              sendVelocityCommand(y);
            }
          }
        }
    }

    /*else if (currentDriveMode ==  PRE_TORQUE_MODE)
    {
      updateTorquePeak();
      Tor.TARGET_TORQUE_PEAK = updateTorquePeak();
      double elapsed = timer.getElapsedTime();
      Tor.PRE_TORQUE = elapsed * Tor.TARGET_TORQUE_PEAK / 3;
      adjusted_torque = Tor.PRE_TORQUE;
      sendVelocityCommand(adjusted_torque);
    }*/

    else if (currentDriveMode == TORQUE_DRIVE_MODE) 
    {
        AdaptiveSpeed(timer.time2);
        double elapsed = timer.getElapsedTime();
        if (control_mode == 1) 
        {
          y = calculateTargetTorque();  // 模式1
        } 
        else 
        {
          y = TargetTorque();    // 模式2
        }
        y = std::clamp(y, T_min, T_max);
        adjusted_torque = y;
        sendTorTransVelCommand(y);
        if (elapsed  > Tor.DRIVE_DURATION) 
        {
            currentDriveMode = STAND_MODE;
            ST.isForward = false;
            if(pendingCommand == 0x44)
            {
                currentDriveMode = VELOCITY_MODE;
                pendingCommand = 0;
            }
        }
    }
      
    else if (currentDriveMode == VELOCITY_MODE) 
    {
      ST.RetrunInitial = true;
      ST.initialMotorRecorded = false;
      ST.slope_store = false;
      ST.recorded = true;
      rclcpp::sleep_for(std::chrono::milliseconds((int)(0.015*1000)));
      y = calculateTargetPosition();
      sendVelocityCommand(y);
    }
    else if (currentDriveMode == STAND_MODE) 
    {
        send_speed(0.0);
    }
    //发布消息
    can_ankle::msg::Torque torque_msg;
    torque_msg.torque_value = adjusted_torque;
    torque_msg.force_sensortorque = SensorTorque;
    torque_msg.velocity_value = velocity_value;
    torque_msg.return_velocity = Output_VelocityValue;
    torque_msg.return_torque_value = Output_TorqueValue;
    torque_pub->publish(torque_msg);
    if (++loop_count % 200 == 1) { // 每秒打印一次
      RCLCPP_INFO(node->get_logger(), "主循环运行中 count=%d mode=%d force=%.2f",
                  loop_count, (int)currentDriveMode, SensorTorque);
    }
  }
  if (slope_file.is_open())
  {
      slope_file.close();
      RCLCPP_INFO(rclcpp::get_logger("ankle"), "坡度文件已关闭");
  }
}

//发送canopen配置指令函数
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

//速度模式函数
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
  SendData(config_node, 0x00000653, cfg.para_Vmode1);
  usleep(200000);
  SendData(config_node, 0x00000653, cfg.para_Vmode2);
  usleep(200000);
  SendData(config_node, 0x00000653, cfg.para_Vmode3);
  usleep(200000);
  cout << "使能完毕" << endl;
}

//位置模式函数
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
}

//扭矩模式函数
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
}

//can线程启动函数

//线程接收函数

//扭矩曲线生成函数
double calculateTargetTorque() 
{
    double elapsed = timer.getElapsedTime();
    double torque = 0.0;
    Tor.TARGET_TORQUE_PEAK = updateTorquePeak();
    // 阶段时间计算
    double rise_phase_start = Tor.TARGET_TORQUE_RISE_TIME;
    double fall_phase_end =Tor.TARGET_TORQUE_RISE_TIME + Tor.TARGET_TORQUE_FALL_TIME;

    // 阶段1：上升阶段
    if (elapsed < rise_phase_start) 
    {
        double t_rise = elapsed;
        double phase = M_PI * t_rise / Tor.TARGET_TORQUE_RISE_TIME;
        torque = (1 - cos(phase)) * (Tor.TARGET_TORQUE_PEAK ) / 2;
    } 
    // 阶段2：下降阶段
    else if (elapsed < fall_phase_end) 
    {
        double t_fall =  elapsed - Tor.TARGET_TORQUE_RISE_TIME;
        double phase = M_PI * t_fall / Tor.TARGET_TORQUE_FALL_TIME;
        torque = (1 + cos(phase)) / 2 * Tor.TARGET_TORQUE_PEAK;
    }

    return torque;
}

//定参数扭矩曲线生成函数
double TargetTorque() 
{
    double elapsed = timer.getElapsedTime();
    double torque = 0.0;
    Tor.TARGET_TORQUE_PEAK = Tor.TARGET_TORQUE_BASE;
    // 阶段时间计算
    double rise_phase = 0.18;
    double fall_phase =0.21;

    // 阶段1：上升阶段
    if (elapsed < rise_phase) 
    {
        double t_rise = elapsed;
        double phase = M_PI * t_rise / rise_phase;
        torque = (1 - cos(phase)) / 2 * Tor.TARGET_TORQUE_PEAK;
    } 
    // 阶段2：下降阶段
    else if (elapsed < rise_phase + fall_phase) 
    {
        double t_fall =  elapsed - rise_phase;
        double phase = M_PI * t_fall / fall_phase;
        torque = (1 + cos(phase))* (Tor.TARGET_TORQUE_PEAK ) / 2;
    }

    return torque;
}

//摆动相实时跟踪函数
double calculateTargetPosition() 
{
    // 获取当前时间和位置
    auto now = high_resolution_clock::now();
    double theta_m = Enc.position_angle; // 当前编码器测量位置
    double theta_d = Enc.initialEncoderValue; // 期望位置(编码器启动时的位置)

    // 计算时间差dt（单位：秒）
    double dt = duration_cast<duration<double>>(now - pos_controller.prev_time).count();
    if (dt < 1e-6) dt = 1e-6; // 防止除以零

    // 计算速度：当前速度 = (当前位置 - 上一时刻位置) / dt
    double theta_m_dot = (theta_m - pos_controller.prev_theta_m) / dt;
    //将拉力传感器与摆动相位置结合
    double forcesensor = SensorTorque / 0.12;
    double theta_d_dot = 0;
    if (forcesensor > 0)
    {
      if(theta_d < theta_m)
      {
        theta_d_dot = pos_controller.K_theta * (theta_d - theta_m) * forcesensor * 1.0;
      }
      else
      {
         theta_d_dot = 0;
      }
    }
    else
    {
      theta_d_dot = 0;
    }

    // 根据公式计算期望速度
    
    theta_d_dot = std::clamp(theta_d_dot , V_min , V_max);

    // 更新上一时刻的数据
    pos_controller.prev_theta_m = theta_m;
    pos_controller.prev_time = now;

    // 返回控制量（期望速度）
    return theta_d_dot;
}

//进入支撑相进行位置补偿
double calculateSupportPosition() 
{
    // 获取当前时间和位置
    auto now = high_resolution_clock::now();
    double theta_m = Output_Angle; // 当前测量位置
    double theta_d = 0;
    if (Enc.initialMotorPosition_counter % 2 == 1) 
    { // 奇数次
      theta_d = Enc.initialMotorPosition_two; // 期望位置(电机双数次启动时的位置)
    }
    else
    {
      theta_d = Enc.initialMotorPosition_one; // 期望位置(电机单数次启动时的位置)
    }
    // 计算时间差dt（单位：秒）
    double dt = duration_cast<duration<double>>(now -  startpos_controller.prev_time).count();
    if (dt < 1e-6) dt = 1e-6; // 防止除以零

    // 计算速度：当前速度 = (当前位置 - 上一时刻位置) / dt
    double theta_m_dot = (theta_m -  startpos_controller.prev_theta_m) / dt;

    // 根据公式计算期望速度
    double theta_d_dot = startpos_controller.K_theta * (theta_d - theta_m) +  startpos_controller.K_d * theta_m_dot;

    theta_d_dot = std::clamp(theta_d_dot , V_min , V_max);

    // 更新上一时刻的数据
    startpos_controller.prev_theta_m = theta_m;
    startpos_controller.prev_time = now;

    // 返回控制量（期望速度）
    return theta_d_dot;
}
