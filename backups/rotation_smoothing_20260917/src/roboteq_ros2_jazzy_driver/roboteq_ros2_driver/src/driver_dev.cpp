#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"


#include <chrono> 
#include <functional> 
#include <memory>     
#include <string>     

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>
#include <cstdlib>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

// dependencies for ROS
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))


// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG




// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish (TODO: write custom roboteq messages to support other reportable data from MC

// #define _ODOM_SENSORS

// Define following to enable service for returning covariance
// #define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

serial::Serial controller;
uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
Roboteq::Roboteq() : Node("roboteq_ros2_driver")
// initialize parameters and variables
{
    pub_odom_tf = this->declare_parameter("pub_odom_tf", true);
    odom_frame = this->declare_parameter("odom_frame", "odom");
    base_frame = this->declare_parameter("base_frame", "base_footprint");
    cmdvel_topic = this->declare_parameter("cmdvel_topic", "cmd_vel");
    odom_topic = this->declare_parameter("odom_topic", "odom");
    port = this->declare_parameter("port", "/dev/roboteq");
    baud = this->declare_parameter("baud", 115200);
    open_loop = this->declare_parameter("open_loop", true);
    wheel_circumference = this->declare_parameter("wheel_circumference", 0.877); //タイヤの円周(m)
    track_width = this->declare_parameter("track_width", 0.40); //トレッド幅(m)
    max_amps = this->declare_parameter("max_amps", 12.4);
    max_rpm = this->declare_parameter("max_rpm", 2500);
    gear_ratio = this->declare_parameter("gear_ratio", 1.0); //ギア比
    pulse = this->declare_parameter("pulse", 229); //一周あたりのパルス数
    max_speed = this->declare_parameter("max_speed", 0.5); //最大速度
    // odom streaming/publish tuning
    odom_stream_interval_ms = this->declare_parameter("odom_stream_interval_ms", 50);
    odom_publish_hz = this->declare_parameter("odom_publish_hz", 50.0);
    // Speed scale factor to calibrate actual robot speed to match commanded speed
    // If robot moves faster than commanded, decrease this value (e.g., 0.625 for 60% faster)
    // If robot moves slower than commanded, increase this value (e.g., 1.2 for 20% slower)
    speed_scale = this->declare_parameter("speed_scale", 1.0);
    kp_soft = this->declare_parameter("kp_soft", 0.0);
    min_speed_threshold = this->declare_parameter("min_speed_threshold", 0.10);
    encoder_sign_r = this->declare_parameter("encoder_sign_r", -1.0);
    encoder_sign_l = this->declare_parameter("encoder_sign_l", 1.0);

    starttime = 0;
    hstimer = 0;
    mstimer = 0;
    odom_idx = 0;
    odom_encoder_toss = 5;
    odom_encoder_left = 0;
    odom_encoder_right = 0;
    odom_x = 0.0;
    odom_y = 0.0;
    odom_yaw = 0.0;
    odom_last_x = 0.0;
    odom_last_y = 0.0;
    odom_last_yaw = 0.0;
    odom_last_time = 0;
    right_rpm_command = 0.0;
    left_rpm_command = 0.0;
    linear_x = 0.0;
    angular_z = 0.0;
    running_ = true;

    //odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg = nav_msgs::msg::Odometry();

    serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // タイムアウトを短く設定
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);
    // connect to serial port
    connect();
    // configure motor controller
    cmdvel_setup();
    odom_setup();
//
//  odom publisher
//
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);

    RCLCPP_INFO(this->get_logger(), "Odometry: wheel_circumference=%.4f pulse=%d gear_ratio=%.3f speed_scale=%.4f",
                wheel_circumference, pulse, gear_ratio, speed_scale);
//
// cmd_vel subscriber
//

    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1000,         // QoS history depth
        std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));

    stop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "stop",
        10,
        std::bind(&Roboteq::bumper_callback, this, std::placeholders::_1)
    );
    
    using namespace std::chrono_literals;
    // set odometry publishing loop timer at 10Hz
    timer_ = this->create_wall_timer(10ms,std::bind(&Roboteq::run, this));
    // enable modifying params at run-time
    /*    
    using namespace std::chrono_literals;
    */
    param_update_timer = this->create_wall_timer(1000ms, std::bind(&Roboteq::update_parameters, this));

    // Start reading only after the serial connection, controller setup,
    // odometry stream setup, publisher, and shared state are ready.
    odom_thread_ = std::thread(&Roboteq::odom_loop, this);
}

void Roboteq::update_parameters()
{
    this->get_parameter("pub_odom_tf", pub_odom_tf);
    this->get_parameter("odom_frame", odom_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("cmdvel_topic", cmdvel_topic);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("port", port);
    this->get_parameter("baud", baud);
    this->get_parameter("open_loop", open_loop);
    double new_wheel_circumference = wheel_circumference;
    double new_track_width = track_width;
    int new_max_rpm = max_rpm;
    this->get_parameter("wheel_circumference", new_wheel_circumference);
    this->get_parameter("track_width", new_track_width);
    this->get_parameter("max_amps", max_amps);
    this->get_parameter("max_rpm", new_max_rpm);
    float new_gear_ratio = gear_ratio;
    int new_pulse = pulse;
    double new_encoder_sign_r = encoder_sign_r;
    double new_encoder_sign_l = encoder_sign_l;
    this->get_parameter("gear_ratio", new_gear_ratio);
    this->get_parameter("pulse", new_pulse);
    this->get_parameter("max_speed", max_speed);
    // read odom timing params
    int new_stream_ms = odom_stream_interval_ms;
    this->get_parameter("odom_stream_interval_ms", new_stream_ms);
    this->get_parameter("odom_publish_hz", odom_publish_hz);
    this->get_parameter("speed_scale", speed_scale);
    this->get_parameter("kp_soft", kp_soft);
    this->get_parameter("min_speed_threshold", min_speed_threshold);
    this->get_parameter("encoder_sign_r", new_encoder_sign_r);
    this->get_parameter("encoder_sign_l", new_encoder_sign_l);
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        gear_ratio = new_gear_ratio;
        pulse = new_pulse;
        encoder_sign_r = new_encoder_sign_r;
        encoder_sign_l = new_encoder_sign_l;
        wheel_circumference = new_wheel_circumference;
        track_width = new_track_width;
        max_rpm = new_max_rpm;
    }
    // If the stream interval changed while running, re-send the stream
    // configuration to the device so it starts using the new rate sooner.
    bool stream_interval_changed = false;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (new_stream_ms != odom_stream_interval_ms) {
            odom_stream_interval_ms = new_stream_ms;
            stream_interval_changed = true;
        }
    }
    if (stream_interval_changed) {
        try {
            odom_stream();
            last_sent_odom_stream_ms = new_stream_ms;
            RCLCPP_INFO(this->get_logger(), "Updated odom stream interval to %d ms and re-sent stream command", new_stream_ms);
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to re-send odom stream config: " << e.what());
        }
    }
}

void Roboteq::connect() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
    
    // 接続前に閉じておく
    if (controller.isOpen()) {
        try {
            controller.close();
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Error closing port: " << e.what());
        }
    }
    
    int retry_count = 0;
    const int max_retries = 5;
    
    while (retry_count < max_retries) {
        try {
            controller.open();
            if (controller.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                
                // シリアルポートの設定を最適化
                controller.setFlowcontrol(serial::flowcontrol_none);  // フロー制御を無効化
                controller.setBytesize(serial::eightbits);  // 8ビット
                controller.setParity(serial::parity_none);  // パリティなし
                controller.setStopbits(serial::stopbits_one);  // ストップビット1
                
                return;
            }
        } catch (const serial::IOException &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Serial::IOException: " << e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Exception: " << e.what());
        }
        
        retry_count++;
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to open serial port, retry " << retry_count << " of " << max_retries << "...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port after maximum retries");
}

void Roboteq::bumper_callback(const std_msgs::msg::Bool::SharedPtr stop_msg)
{
    if (stop_msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Bumper!!!");
        safe_serial_write("!EX 1\r");
    }

    else
    {
        RCLCPP_INFO(this->get_logger(), "Restart!!!");
        // Releasing emergency stop. Reset command values to 0 to prevent the controller
        // from resuming previous non-zero velocities from its memory.
        safe_serial_write("!G 1 0\r");
        safe_serial_write("!G 2 0\r");
        safe_serial_write("!MG\r");
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            linear_x = 0.0;
            angular_z = 0.0;
            right_rpm_command = 0.0;
            left_rpm_command = 0.0;
        }
    }
}

// シリアルポートへの安全な書き込み関数を追加
bool Roboteq::safe_serial_write(const std::string &cmd) {
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Cannot write to closed port. Attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
            return false;
        }
    }
    
    try {
        controller.write(cmd);
        return true;
    } catch (const serial::IOException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial write failed: " << e.what());
        return false;
    } catch (const serial::PortNotOpenedException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Port not opened: " << e.what());
        return false;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown error during serial write: " << e.what());
        return false;
    }
}

void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    // 車輪速度の計算 (m/s)
    // 右モーター(M1): 正(+)の指令で前進
    // 左モーター(M2): 負(-)の指令で前進
    float right_speed = (twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0);
    float left_speed = (twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0) * -1;

    const float command_linear_x = twist_msg->linear.x;
    const float command_angular_z = twist_msg->angular.z;

    float min_speed_thresh = (min_speed_threshold > 0.0) ? min_speed_threshold : 0.10f;
    constexpr float EPSILON = 1e-6f;
    float scale = (speed_scale > 0.0) ? speed_scale : 1.0;
    float min_threshold = min_speed_thresh * scale;

    // speed_scaleを先に適用する。
    // これにより、不感帯（デッドバンド）の判定を実際にモーターへ送る最終的な指令値ベースで行える。
    right_speed *= scale;
    left_speed *= scale;

    // 指令値がゼロでない場合のみ、低速時の底上げ処理を行う
    if ((std::abs(command_linear_x) > EPSILON) || (std::abs(command_angular_z) > EPSILON))
    {
        float max_abs_speed = std::max(std::abs(right_speed), std::abs(left_speed));
        
        // 最大速度がしきい値（スケーリング済み）未満かつゼロでない場合、比率を維持したまま底上げ
        if (max_abs_speed > EPSILON && max_abs_speed < min_threshold)
        {
            float boost_factor = min_threshold / max_abs_speed;
            right_speed *= boost_factor;
            left_speed *= boost_factor;
        }
        // 指令はあるが計算上の車輪速度がほぼゼロになる場合（例：低速旋回のみなど）
        else if (max_abs_speed < EPSILON)
        {
            if (command_linear_x > EPSILON) {
                right_speed = min_threshold;
                left_speed = -min_threshold;
            } else if (command_linear_x < -EPSILON) {
                right_speed = -min_threshold;
                left_speed = min_threshold;
            } else {
                // 旋回指令のみの場合
                if (command_angular_z > 0) {
                    right_speed = min_threshold;
                    left_speed = min_threshold;
                } else {
                    right_speed = -min_threshold;
                    left_speed = -min_threshold;
                }
            }
        }
    }

    std::stringstream left_cmd;
    std::stringstream right_cmd;
    
    float rpm_r = right_speed / wheel_circumference * 60.0;
    float rpm_l = left_speed / wheel_circumference * 60.0;

    // モーター出力指令 (0-1000 スケール)
    int32_t right_power = rpm_r / max_rpm * 1000.0;
    int32_t left_power = rpm_l / max_rpm * 1000.0;
    
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        linear_x = command_linear_x;
        angular_z = command_angular_z;
        right_rpm_command = rpm_r;
        left_rpm_command = rpm_l;
    }
    
    right_cmd << "!G 1 " << right_power << "\r";
    left_cmd << "!G 2 " << left_power << "\r";
    // モーターコントローラへコマンドを送信
    #ifndef _CMDVEL_FORCE_RUN
        safe_serial_write(right_cmd.str());
        safe_serial_write(left_cmd.str());
        try {
            if (controller.isOpen()) {
                controller.flush();
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_callback flush: " << e.what());
        }
        
        // 送信したモーターコマンドをログ出力
        // RCLCPP_INFO(this->get_logger(), "cmdvel: linear=%.3f angular=%.3f -> R=%d L=%d (%s)",
        //             linear_x, angular_z, right_power_or_rpm, left_power_or_rpm,
        //             open_loop ? "power" : "rpm");
    #endif
}
void Roboteq::cmdvel_setup()
{
    // stop motors
    safe_serial_write("!G 1 0\r");
    safe_serial_write("!G 2 0\r");
    safe_serial_write("!S 1 0\r");
    safe_serial_write("!S 2 0\r");
    
    if (!controller.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open, cannot complete setup");
        return;
    }
    
    try {
        controller.flush();

        // disable echo
        safe_serial_write("^ECHOF 1\r");
        controller.flush();

        // enable watchdog timer (1000 ms)
        safe_serial_write("^RWD 1000\r");

        // set motor operating mode (0 for open-loop speed)
        safe_serial_write("^MMOD 1 0\r");
        safe_serial_write("^MMOD 2 0\r");

        // set motor amps limit (A * 10)
        std::stringstream right_ampcmd;
        std::stringstream left_ampcmd;
        right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
        left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
        safe_serial_write(right_ampcmd.str());
        safe_serial_write(left_ampcmd.str());

        // set max speed (rpm) for relative speed commands
        std::stringstream right_rpmcmd;
        std::stringstream left_rpmcmd;
        right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
        left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
        safe_serial_write(right_rpmcmd.str());
        safe_serial_write(left_rpmcmd.str());

        // set max acceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MAC 1 20000\r");
        safe_serial_write("^MAC 2 20000\r");

        // set max deceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MDEC 1 20000\r");
        safe_serial_write("^MDEC 2 20000\r");

        // set open-loop acceleration and deceleration ramps (value in 0.1s to go from 0 to 100% power)
        safe_serial_write("^MXACC 1 12\r");
        safe_serial_write("^MXACC 2 12\r");
        safe_serial_write("^MXDEC 1 12\r");
        safe_serial_write("^MXDEC 2 12\r");

        // set PID parameters (gain * 10)
        safe_serial_write("^KP 1 0\r");
        safe_serial_write("^KP 2 0\r");
        safe_serial_write("^KI 1 0\r");
        safe_serial_write("^KI 2 0\r");
        safe_serial_write("^KD 1 0\r");
        safe_serial_write("^KD 2 0\r");

        // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
        safe_serial_write("^EMOD 1 18\r");
        safe_serial_write("^EMOD 2 34\r");

        // set encoder counts (ppr)
        std::stringstream right_enccmd;
        std::stringstream left_enccmd;
        right_enccmd << "^EPPR 1 " << pulse << "\r";
        left_enccmd << "^EPPR 2 " << pulse << "\r";
        safe_serial_write(right_enccmd.str());
        safe_serial_write(left_enccmd.str());

        controller.flush();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_setup: " << e.what());
    }
}

void Roboteq::cmdvel_loop()
{
}

void Roboteq::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
    if (open_loop)
    {
    controller.write("!G 1 100\r");
    controller.write("!G 2 100\r");
    }
    else
    {
    std::stringstream right_cmd;
    std::stringstream left_cmd;
    right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
    left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
    controller.write(right_cmd.str());
    controller.write(left_cmd.str());
    }
    controller.flush();
#endif
}


void Roboteq::odom_setup()
{
    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    // Initialize covariance regardless of whether TF publication is enabled.
    // TODO: Tune these existing covariance values from measured robot data.
    for (size_t i = 0; i < 36; i++)
    {
        odom_msg.pose.covariance[i] = 0;
        odom_msg.twist.covariance[i] = 0;
    }

    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1000000;
    odom_msg.pose.covariance[21] = 1000000;
    odom_msg.pose.covariance[28] = 1000000;
    odom_msg.pose.covariance[35] = 1000;

    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[14] = 1000000;
    odom_msg.twist.covariance[21] = 1000000;
    odom_msg.twist.covariance[28] = 1000000;
    odom_msg.twist.covariance[35] = 1000;

    // Configure encoder streaming once after serial connection, independent
    // of whether this node is responsible for publishing odom -> base TF.
    odom_stream();
    last_sent_odom_stream_ms = odom_stream_interval_ms;
    odom_last_time = millis();
#ifdef _ODOM_SENSORS
    current_last_time = millis();
#endif
}

// Odom msg streams

void Roboteq::odom_stream()
{

#ifdef _ODOM_SENSORS
    // start encoder and current output (30 hz)
    // doubling frequency since one value is output at each cycle
    //  controller.write("# C_?CR_?BA_# 17\r");
    // start encoder, current and voltage output (30 hz)
    // tripling frequency since one value is output at each cycle
    safe_serial_write("# C_?CB_?BA_?V_# 11\r");
#else
    // use configured streaming interval (ms) - Roboteq expects the interval
    // in ms in this command (e.g. 50 -> 20Hz). Make sure value is sane.
    int stream_ms = 50;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        stream_ms = odom_stream_interval_ms;
    }
    if (stream_ms <= 0) stream_ms = 50;
    std::stringstream ss;
    ss << "# C_?CB_# " << stream_ms << "\r";
    safe_serial_write(ss.str());
    
    // オプション: より小さなデータフォーマットを使用
    // 注: お使いのRoboteqコントローラがこのコマンドをサポートしているか確認してください
    // safe_serial_write("# _?CR_# 500\r");  // 生カウンタデータ (より小さなデータサイズ)
#endif
    
    try {
        if (controller.isOpen()) {
            controller.flush();
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in odom_stream flush: " << e.what());
    }
}

int64_t Roboteq::encoder_count_delta(int32_t current, int32_t previous)
{
    constexpr int64_t encoder_modulus = (int64_t{1} << 32);
    int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
    if (delta > std::numeric_limits<int32_t>::max()) {
        delta -= encoder_modulus;
    } else if (delta < std::numeric_limits<int32_t>::min()) {
        delta += encoder_modulus;
    }
    return delta;
}

void Roboteq::process_encoder_sample(
    int32_t right_count,
    int32_t left_count,
    std::chrono::steady_clock::time_point stamp)
{
    constexpr double target_window_seconds = 0.100;
    constexpr double minimum_window_seconds = 0.075;
    constexpr double maximum_window_seconds = 0.150;
    constexpr double history_retention_seconds = 0.300;
    constexpr double rpm_safety_factor = 1.5;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_encoder_right = right_count;
    odom_encoder_left = left_count;

    if (!encoder_initialized_) {
        encoder_initialized_ = true;
        previous_encoder_right_ = right_count;
        previous_encoder_left_ = left_count;
        last_encoder_receive_time_ = stamp;
        encoder_history_.push_back({stamp, right_count, left_count});
        velocity_estimate_valid_ = false;
        return;
    }

    const int64_t raw_right_delta = encoder_count_delta(right_count, previous_encoder_right_);
    const int64_t raw_left_delta = encoder_count_delta(left_count, previous_encoder_left_);
    previous_encoder_right_ = right_count;
    previous_encoder_left_ = left_count;

    const double gear = (gear_ratio > 0.0f) ? gear_ratio : 1.0;
    const double counts_per_wheel_revolution = static_cast<double>(pulse) * gear;
    if (counts_per_wheel_revolution <= 0.0) {
        encoder_history_.clear();
        velocity_estimate_valid_ = false;
        last_encoder_receive_time_ = stamp;
        return;
    }

    // The host timestamp belongs to the serial read batch. Packets parsed from
    // one read therefore intentionally share a timestamp. Use the configured
    // stream period as a conservative floor only for rejecting a single
    // impossible count jump; it is never used to calculate velocity.
    double receive_gap_seconds =
        std::chrono::duration<double>(stamp - last_encoder_receive_time_).count();
    const double nominal_validation_seconds =
        std::max(0.020, static_cast<double>(odom_stream_interval_ms) / 1000.0 * 2.0);
    receive_gap_seconds = std::max(receive_gap_seconds, nominal_validation_seconds);
    receive_gap_seconds = std::min(receive_gap_seconds, 0.500);
    const double physical_rpm_limit = std::max(1.0, static_cast<double>(max_rpm) * rpm_safety_factor);
    const double max_counts_per_sample =
        physical_rpm_limit * counts_per_wheel_revolution / 60.0 * receive_gap_seconds + 8.0;

    last_encoder_receive_time_ = stamp;
    if (std::abs(static_cast<double>(raw_right_delta)) > max_counts_per_sample ||
        std::abs(static_cast<double>(raw_left_delta)) > max_counts_per_sample) {
        // Re-synchronize on the received absolute counts. Do not integrate or
        // estimate velocity from a physically impossible jump.
        encoder_history_.clear();
        encoder_history_.push_back({stamp, right_count, left_count});
        velocity_estimate_valid_ = false;
        return;
    }

    const double right_revolutions =
        encoder_sign_r * static_cast<double>(raw_right_delta) / counts_per_wheel_revolution;
    const double left_revolutions =
        encoder_sign_l * static_cast<double>(raw_left_delta) / counts_per_wheel_revolution;
    cumulative_wheel_revolutions_right_ += right_revolutions;
    cumulative_wheel_revolutions_left_ += left_revolutions;

    // Position is integrated when absolute encoder updates arrive. Its timing
    // is independent of the ROS publish timer.
    const double linear_distance =
        (right_revolutions + left_revolutions) * wheel_circumference / 2.0;
    const double angular_distance = track_width > 1e-9
        ? (right_revolutions - left_revolutions) * wheel_circumference / track_width
        : 0.0;
    const double mid_yaw = odom_yaw + angular_distance / 2.0;
    odom_x += linear_distance * std::cos(mid_yaw);
    odom_y += linear_distance * std::sin(mid_yaw);
    odom_yaw = NORMALIZE(odom_yaw + angular_distance);
    odom_last_x = odom_x;
    odom_last_y = odom_y;
    odom_last_yaw = odom_yaw;

    // Coalesce all encoder records returned by one serial read. This avoids
    // assigning artificial microsecond-scale intervals to batched packets.
    if (!encoder_history_.empty() && encoder_history_.back().stamp == stamp) {
        encoder_history_.back() = {stamp, right_count, left_count};
    } else {
        encoder_history_.push_back({stamp, right_count, left_count});
    }

    while (encoder_history_.size() > 2 &&
           std::chrono::duration<double>(stamp - encoder_history_.front().stamp).count() >
               history_retention_seconds) {
        encoder_history_.pop_front();
    }

    const EncoderSample *window_start = nullptr;
    double best_window_error = std::numeric_limits<double>::max();
    double selected_window_seconds = 0.0;
    for (const auto &sample : encoder_history_) {
        const double age_seconds = std::chrono::duration<double>(stamp - sample.stamp).count();
        if (age_seconds < minimum_window_seconds || age_seconds > maximum_window_seconds) {
            continue;
        }
        const double window_error = std::abs(age_seconds - target_window_seconds);
        if (window_error < best_window_error) {
            best_window_error = window_error;
            selected_window_seconds = age_seconds;
            window_start = &sample;
        }
    }

    if (window_start == nullptr || selected_window_seconds <= 0.0) {
        velocity_estimate_valid_ = false;
        return;
    }

    const int64_t window_right_counts = encoder_count_delta(right_count, window_start->right_count);
    const int64_t window_left_counts = encoder_count_delta(left_count, window_start->left_count);
    const double right_rpm = encoder_sign_r * static_cast<double>(window_right_counts) /
        counts_per_wheel_revolution / selected_window_seconds * 60.0;
    const double left_rpm = encoder_sign_l * static_cast<double>(window_left_counts) /
        counts_per_wheel_revolution / selected_window_seconds * 60.0;

    if (std::abs(right_rpm) > physical_rpm_limit || std::abs(left_rpm) > physical_rpm_limit) {
        velocity_estimate_valid_ = false;
        return;
    }

    // The 100 ms window itself suppresses encoder quantization. No EMA is
    // applied here, avoiding another source of lag; any future filter must run
    // only after the physical plausibility checks above.
    estimated_wheel_rpm_right_ = right_rpm;
    estimated_wheel_rpm_left_ = left_rpm;
    const double right_wheel_velocity = right_rpm * wheel_circumference / 60.0;
    const double left_wheel_velocity = left_rpm * wheel_circumference / 60.0;
    estimated_linear_velocity_ = (right_wheel_velocity + left_wheel_velocity) / 2.0;
    estimated_angular_velocity_ = track_width > 1e-9
        ? (right_wheel_velocity - left_wheel_velocity) / track_width
        : 0.0;
    velocity_window_seconds_ = selected_window_seconds;
    velocity_estimate_valid_ = true;
}

void Roboteq::odom_loop()
{
    const size_t buffer_size = 1024;  // バッファサイズを大きく
    uint8_t buffer[buffer_size];
    size_t bytes_read = 0;
    std::string line_buffer;
    
    // データ処理のパフォーマンス計測用
    uint32_t last_diagnostic_time = millis();
    size_t total_bytes = 0;
    int packet_count = 0;
    
    // ストリーミング設定再送用のタイミング変数（odom_publishとは別）
    uint32_t stream_last_time = millis();
    
    while (running_) {
        // ストリーミング設定の再送間隔を30秒に長く
        uint32_t nowtime = millis();
        if (DELTAT(nowtime, stream_last_time) >= 30000) {
            odom_stream();
            stream_last_time = nowtime;
            
            // データレート診断を表示
            float elapsed_secs = DELTAT(nowtime, last_diagnostic_time) / 1000.0f;
            if (elapsed_secs > 0 && packet_count > 0) {
                float bytes_per_sec = total_bytes / elapsed_secs;
                RCLCPP_INFO(this->get_logger(), "Odometry data rate: %.2f KB/s, packets: %d", 
                            bytes_per_sec / 1024.0f, packet_count);
                total_bytes = 0;
                packet_count = 0;
                last_diagnostic_time = nowtime;
            }
        }

        if (controller.available()) {
            try {
                // 一度に複数バイトを読み取る
                bytes_read = controller.read(buffer, buffer_size);
                const auto batch_stamp = std::chrono::steady_clock::now();
                total_bytes += bytes_read;
                
                // 読み取ったデータをより効率的に処理
                for (size_t i = 0; i < bytes_read; i++) {
                    char ch = buffer[i];
                    
                    if (ch == '\r') {
                        packet_count++;
                        
                        // 行の終わり - データを処理
                        if (line_buffer.length() >= 3 && 
                            line_buffer[0] == 'C' && 
                            line_buffer[1] == 'B' && 
                            line_buffer[2] == '=') {
                                
                            if (odom_encoder_toss > 0) {
                                --odom_encoder_toss;
                            } else {
                                // データを解析 - より効率的な実装
                                size_t delimiter_pos = line_buffer.find(':', 3);
                                if (delimiter_pos != std::string::npos) {
                                    try {
                                        // stoi()は例外を投げる可能性があるため、try内で実行
                                        std::string right_str = line_buffer.substr(3, delimiter_pos - 3);
                                        std::string left_str = line_buffer.substr(delimiter_pos + 1);
                                        
                                        int32_t right_val = std::stoi(right_str);
                                        int32_t left_val = std::stoi(left_str);
                                        process_encoder_sample(right_val, left_val, batch_stamp);

                                    }
                                    catch (const std::exception& e) {
                                        // データ解析エラーの無視 - ログに記録しない（頻度が高すぎるため）
                                    }
                                }
                            }
                        }
                        
                        // バッファをクリア
                        line_buffer.clear();
                    } else if (line_buffer.length() < 64) {  // バッファオーバーフロー防止
                        line_buffer += ch;
                    }
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exception reading serial data: " << e.what());
            }
        }
        // odom_publish()はメインスレッド(run関数)から呼び出す
        // 別スレッドからROS2パブリッシャーを呼ぶとクラッシュするため
        // Encoder state is copied by odom_publish() under odom_mutex_.

        // スリープ時間を短縮（CPUの空回しを防ぎつつも、応答性を保つ）
        std::this_thread::sleep_for(std::chrono::microseconds(100));  // 0.1 ms
    }
}

void Roboteq::odom_publish()
{
    static rclcpp::Time last_publish_time(0, 0, RCL_ROS_TIME);
    static bool first_publish = true;
    rclcpp::Time current_time = this->get_clock()->now();
    
    if (first_publish) {
        first_publish = false;
        last_publish_time = current_time;
    }
    
    // Odometryメッセージの発行頻度を制限（リソース節約のため）
    // Use configured publish hz as a maximum. Default was approx 50Hz.
    double publish_interval = 0.02; // default 50Hz
    if (odom_publish_hz > 1e-6) {
        publish_interval = 1.0 / odom_publish_hz;
    }
    if ((current_time - last_publish_time).seconds() < publish_interval) {
        return;
    }
    
    last_publish_time = current_time;

    // Retain publish-period timing only for the existing P-controller feedback
    // path. Odometry pose and twist do not use this dt.
    static rclcpp::Time last_odom_time(0, 0, RCL_ROS_TIME);
    static bool first_odom = true;
    float dt = 0.02f; // default fallback
    if (first_odom) {
        first_odom = false;
        last_odom_time = current_time;
    } else {
        dt = (current_time - last_odom_time).seconds();
        // Skip if dt is invalid or too large (indicates data gap)
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.02f; // fallback to ~50Hz
        }
    }
    last_odom_time = current_time;

    float odom_x_snapshot = 0.0f;
    float odom_y_snapshot = 0.0f;
    float odom_yaw_snapshot = 0.0f;
    double cumulative_right_snapshot = 0.0;
    double cumulative_left_snapshot = 0.0;
    double estimated_right_rpm_snapshot = 0.0;
    double estimated_left_rpm_snapshot = 0.0;
    double estimated_linear_snapshot = 0.0;
    double estimated_angular_snapshot = 0.0;
    double velocity_window_snapshot = 0.0;
    bool velocity_valid_snapshot = false;
    bool encoder_initialized_snapshot = false;
    std::chrono::steady_clock::time_point last_encoder_receive_snapshot;

    // The receive thread owns all calculations. The publish side only copies
    // its completed state while holding the mutex.
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_snapshot = odom_x;
        odom_y_snapshot = odom_y;
        odom_yaw_snapshot = odom_yaw;
        cumulative_right_snapshot = cumulative_wheel_revolutions_right_;
        cumulative_left_snapshot = cumulative_wheel_revolutions_left_;
        estimated_right_rpm_snapshot = estimated_wheel_rpm_right_;
        estimated_left_rpm_snapshot = estimated_wheel_rpm_left_;
        estimated_linear_snapshot = estimated_linear_velocity_;
        estimated_angular_snapshot = estimated_angular_velocity_;
        velocity_window_snapshot = velocity_window_seconds_;
        velocity_valid_snapshot = velocity_estimate_valid_;
        encoder_initialized_snapshot = encoder_initialized_;
        last_encoder_receive_snapshot = last_encoder_receive_time_;
    }

    if (encoder_initialized_snapshot) {
        const double encoder_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_encoder_receive_snapshot).count();
        if (encoder_age > 0.300) {
            velocity_valid_snapshot = false;
        }
    }

    static bool p_feedback_initialized = false;
    static double previous_cumulative_right = 0.0;
    static double previous_cumulative_left = 0.0;
    double legacy_p_right_delta = 0.0;
    double legacy_p_left_delta = 0.0;
    if (p_feedback_initialized) {
        legacy_p_right_delta = cumulative_right_snapshot - previous_cumulative_right;
        legacy_p_left_delta = cumulative_left_snapshot - previous_cumulative_left;
    } else {
        p_feedback_initialized = true;
    }
    previous_cumulative_right = cumulative_right_snapshot;
    previous_cumulative_left = cumulative_left_snapshot;

    float right_rpm_command_snapshot = 0.0f;
    float left_rpm_command_snapshot = 0.0f;
    float linear_x_snapshot = 0.0f;
    float angular_z_snapshot = 0.0f;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        right_rpm_command_snapshot = right_rpm_command;
        left_rpm_command_snapshot = left_rpm_command;
        linear_x_snapshot = linear_x;
        angular_z_snapshot = angular_z;
    }

    // Keep the existing P-controller feedback semantics in this step. It now
    // derives a publish-to-publish delta from monotonic cumulative state rather
    // than racing on a shared accumulator. The new 100 ms RPM estimate is
    // deliberately not connected to motor output yet.
    const float legacy_actual_rpm_r = static_cast<float>(legacy_p_right_delta / dt * 60.0);
    const float legacy_actual_rpm_l = static_cast<float>(-legacy_p_left_delta / dt * 60.0);
    if (!open_loop && kp_soft > 0.001) {
        float error_r = right_rpm_command_snapshot - legacy_actual_rpm_r;
        float error_l = left_rpm_command_snapshot - legacy_actual_rpm_l;
        
        int32_t power_r = static_cast<int32_t>((right_rpm_command_snapshot / max_rpm * 1000.0f) + (kp_soft * error_r));
        int32_t power_l = static_cast<int32_t>((left_rpm_command_snapshot / max_rpm * 1000.0f) + (kp_soft * error_l));
        
        power_r = std::max(-1000, std::min(1000, power_r));
        power_l = std::max(-1000, std::min(1000, power_l));
        
        if (std::abs(right_rpm_command_snapshot) > 0.1f || std::abs(left_rpm_command_snapshot) > 0.1f) {
            std::stringstream r_cmd, l_cmd;
            r_cmd << "!G 1 " << power_r << "\r";
            l_cmd << "!G 2 " << power_l << "\r";
            safe_serial_write(r_cmd.str());
            safe_serial_write(l_cmd.str());
        }
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    // convert yaw to quat
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, odom_yaw_snapshot);
    geometry_msgs::msg::Quaternion quat;
    quat.x = tf2_quat.x();
    quat.y = tf2_quat.y();
    quat.z = tf2_quat.z();
    quat.w = tf2_quat.w();

    //update odom msg

    if(pub_odom_tf)
    {
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = base_frame;
        tf_msg.transform.translation.x = odom_x_snapshot;
        tf_msg.transform.translation.y = odom_y_snapshot;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = quat;
        odom_baselink_transform_->sendTransform(tf_msg);
    }

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;
    odom_msg.pose.pose.position.x = odom_x_snapshot;
    odom_msg.pose.pose.position.y = odom_y_snapshot;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;

    // Twist comes only from the accepted encoder-count difference over the
    // approximately 100 ms steady-clock window. ROS publish dt is not used.
    odom_msg.twist.twist.linear.x = velocity_valid_snapshot ? estimated_linear_snapshot : 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = velocity_valid_snapshot ? estimated_angular_snapshot : 0.0;

    odom_pub->publish(odom_msg);

    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Encoder velocity: valid=%d window=%.1fms wheel_rpm(R=%.2f L=%.2f) "
        "odom(lin=%.3f ang=%.3f) cmd(lin=%.3f ang=%.3f) legacy_p_rpm(R=%.2f L=%.2f)",
        velocity_valid_snapshot, velocity_window_snapshot * 1000.0,
        estimated_right_rpm_snapshot, estimated_left_rpm_snapshot,
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z,
        linear_x_snapshot, angular_z_snapshot, legacy_actual_rpm_r, legacy_actual_rpm_l);
}

int Roboteq::run() {
    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;
        
    try {
        cmdvel_loop();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_loop: " << e.what());
    }
    
    // オドメトリをメインスレッドからパブリッシュ
    try {
        odom_publish();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in odom_publish: " << e.what());
    }
    
    // デバイスの接続状態をチェック
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Device disconnected, attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
        }
    }

    try {
        cmdvel_run();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_run: " << e.what());
    }
    
    return 0;
}


Roboteq::~Roboteq()
{
    running_ = false;
    if (odom_thread_.joinable()){
        odom_thread_.join();
    }

    if (controller.isOpen()){
        controller.close();
    }
}

} // end of namespace

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    printf("stop");
    rclcpp::shutdown();
    return 0;
}
