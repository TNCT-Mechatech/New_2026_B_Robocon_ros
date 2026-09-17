#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "SerialBridge.hpp"
#include "LinuxHardwareSerial.hpp"

#include <chrono>
#include <cstdint>
#include <cmath>


//============================================================
// シリアルポート
//============================================================

#define SERIAL_PATH_1 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0" // Leonardo
#define SERIAL_PATH_2 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0" // Mega


//============================================================
// 最大値
//============================================================

// ジョイスティック最大値
#define MAX_VALUE (255 * 0.8)

// 十字キーの値
// ジョイスティックより小さい値
#define DPAD_VALUE (MAX_VALUE * 0.8)


//============================================================
// 十字キーの番号
//============================================================
//
// buttons:
// 0  A
// 1  B
// 2  X
// 3  Y
// 4  LB(L1)
// 5  RB(R1)
// 6  G
// 7  S
// 8  不明
// 9  左ジョイスティック押し込み
// 10 右ジョイスティック押し込み
// 11 上
// 12 下
// 13 左
// 14 右
//
//============================================================

#define DPAD_UP     11
#define DPAD_DOWN   12
#define DPAD_LEFT   13
#define DPAD_RIGHT  14


//============================================================
// Leonardoに送るデータ
//============================================================
//
// 足回り
// joyX
// joyY
// joyRot
//
// 射出
// buttonL1
// buttonY
//
// 合計 8Byte
//
//============================================================

typedef struct
{
    int16_t joyX;
    int16_t joyY;
    int16_t joyRot;

    uint8_t buttonL1;
    uint8_t buttonY;

} JoyData_t;


//============================================================
// Leonardoから返ってくるデータ
//============================================================
//
// buttonL1
// buttonY
// limitShootUP
// limitShootDOWN
//
// 合計 4Byte
//
//============================================================

typedef struct
{
    uint8_t buttonL1;
    uint8_t buttonY;

    uint8_t limitShootUP;
    uint8_t limitShootDOWN;

} ResponseData_t;


//============================================================
// Megaへ送るボタンデータ
//============================================================

typedef struct
{
    uint8_t buttonB;
    uint8_t buttonA;
    uint8_t buttonX;

} MEGA_t;


//============================================================
// Megaから返ってくるデータ
//============================================================

typedef struct
{
    uint8_t buttonB;
    uint8_t buttonA;
    uint8_t buttonX;

} MEGA_Response_t;


//============================================================
// SerialArduinoNode
//============================================================

class SerialArduinoNode : public rclcpp::Node
{
public:

    SerialArduinoNode()
        : Node("serial_arduino")
    {
        //====================================================
        // シリアル初期化
        //====================================================

        init_serial(SERIAL_PATH_1);


        //====================================================
        // Controller1
        //====================================================

        joy1_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/controller/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy1_callback,
                    this,
                    std::placeholders::_1));


        //====================================================
        // 20msタイマー
        //====================================================

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SerialArduinoNode::timer_callback,
                    this));


        //====================================================
        // 3秒待機
        //====================================================

        start_time_ =
            std::chrono::steady_clock::now();


        RCLCPP_INFO(
            get_logger(),
            "Waiting 3 seconds before TX...");
    }


    //========================================================
    // Destructor
    //========================================================

    ~SerialArduinoNode()
    {
        delete bridge1_;
        delete serial1_;

        delete bridge2_;
        delete serial2_;
    }


private:

    //========================================================
    // Serial
    //========================================================

    LinuxHardwareSerial *serial1_ = nullptr;
    LinuxHardwareSerial *serial2_ = nullptr;

    SerialBridge *bridge1_ = nullptr;
    SerialBridge *bridge2_ = nullptr;


    //========================================================
    // Message
    //========================================================

    sb::Message<JoyData_t> joy_msg_;

    sb::Message<ResponseData_t> response_msg_;

    sb::Message<MEGA_t> mega_msg_;

    sb::Message<MEGA_Response_t> mega_response_msg_;


    //========================================================
    // Controller1
    //========================================================

    JoyData_t joy1_data_ = {
        0,
        0,
        0,
        0,
        0
    };


    //========================================================
    // Controller状態
    //========================================================

    bool controller1_connected_ = false;


    //========================================================
    // ROS2
    //========================================================

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;

    rclcpp::TimerBase::SharedPtr timer_;


    //========================================================
    // 3秒待機
    //========================================================

    std::chrono::steady_clock::time_point start_time_;


    //========================================================
    // Serial初期化
    //========================================================

    bool init_serial(
        const std::string &port)
    {
        //====================================================
        // Leonardo
        //====================================================

        serial1_ =
            new LinuxHardwareSerial(
                port.c_str(),
                B230400);

        bridge1_ =
            new SerialBridge(serial1_);


        // Pi → Leonardo
        bridge1_->add_frame(
            0,
            &joy_msg_);


        // Leonardo → Pi
        bridge1_->add_frame(
            1,
            &response_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: Leonardo");


        //====================================================
        // Mega
        //====================================================

        serial2_ =
            new LinuxHardwareSerial(
                SERIAL_PATH_2,
                B115200);

        bridge2_ =
            new SerialBridge(serial2_);


        // Pi → Mega
        bridge2_->add_frame(
            0,
            &mega_msg_);


        // Mega → Pi
        bridge2_->add_frame(
            1,
            &mega_response_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: MEGA");


        return true;
    }


    //========================================================
    // Controller1
    //========================================================

    void joy1_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!controller1_connected_)
    {
        controller1_connected_ = true;
        RCLCPP_INFO(get_logger(), "Controller1 Connected!");
    }

    if (msg->axes.size() < 3)
        return;

    if (msg->buttons.size() < 15)
        return;


    //==================================================
    // スティック入力
    //==================================================

    int16_t stickX =
        static_cast<int16_t>(-msg->axes[0] * MAX_VALUE);

    int16_t stickY =
        static_cast<int16_t>(-msg->axes[1] * MAX_VALUE);

    int16_t stickRot =
        static_cast<int16_t>(-msg->axes[2] * MAX_VALUE);


    //==================================================
    // スティックのデッドゾーン
    //==================================================

    const int STICK_THRESHOLD = 20;

    if (std::abs(stickX) <= STICK_THRESHOLD)
        stickX = 0;

    if (std::abs(stickY) <= STICK_THRESHOLD)
        stickY = 0;

    if (std::abs(stickRot) <= STICK_THRESHOLD)
        stickRot = 0;


    //==================================================
    // 十字キー
    //
    // buttons[11] = 上
    // buttons[12] = 下
    // buttons[13] = 左
    // buttons[14] = 右
    //==================================================

    int16_t dpadX = 0;
    int16_t dpadY = 0;

    bool dpadUp =
        static_cast<bool>(msg->buttons[11]);

    bool dpadDown =
        static_cast<bool>(msg->buttons[12]);

    bool dpadLeft =
        static_cast<bool>(msg->buttons[13]);

    bool dpadRight =
        static_cast<bool>(msg->buttons[14]);


    //==================================================
    // 十字キー入力
    //==================================================

    if (dpadUp && !dpadDown)
    {
        dpadY = DPAD_VALUE;
    }
    else if (dpadDown && !dpadUp)
    {
        dpadY = -DPAD_VALUE;
    }

    if (dpadLeft && !dpadRight)
    {
        dpadX = -DPAD_VALUE;
    }
    else if (dpadRight && !dpadLeft)
    {
        dpadX = DPAD_VALUE;
    }


    //==================================================
    // 入力状態確認
    //==================================================

    bool stickMoving =
        (stickX != 0) ||
        (stickY != 0) ||
        (stickRot != 0);

    bool dpadMoving =
        (dpadX != 0) ||
        (dpadY != 0);


    //==================================================
    // スティックと十字キーの選択
    //
    // 両方操作
    // → 停止
    //
    // スティックのみ
    // → スティック
    //
    // 十字キーのみ
    // → 十字キー
    //
    // どちらも操作していない
    // → 停止
    //==================================================

    if (stickMoving && dpadMoving)
    {
        joy1_data_.joyX = 0;
        joy1_data_.joyY = 0;
        joy1_data_.joyRot = 0;
    }
    else if (stickMoving)
    {
        joy1_data_.joyX = stickX;
        joy1_data_.joyY = stickY;
        joy1_data_.joyRot = stickRot;
    }
    else if (dpadMoving)
    {
        joy1_data_.joyX = dpadX;
        joy1_data_.joyY = dpadY;
        joy1_data_.joyRot = 0;
    }
    else
    {
        joy1_data_.joyX = 0;
        joy1_data_.joyY = 0;
        joy1_data_.joyRot = 0;
    }


    //==================================================
    // ボタン
    //
    // buttons[0] = A
    // buttons[1] = B
    // buttons[2] = X
    // buttons[3] = Y
    // buttons[4] = L1
    //==================================================

    joy1_data_.buttonL1 =
        static_cast<uint8_t>(msg->buttons[4]);

    joy1_data_.buttonY =
        static_cast<uint8_t>(msg->buttons[3]);


    //==================================================
    // MEGA
    //
    // buttons[0] = A
    // buttons[1] = B
    // buttons[2] = X
    //==================================================

    mega_msg_.data.buttonA =
        static_cast<uint8_t>(msg->buttons[0]);

    mega_msg_.data.buttonB =
        static_cast<uint8_t>(msg->buttons[1]);

    mega_msg_.data.buttonX =
        static_cast<uint8_t>(msg->buttons[2]);
}


    //========================================================
    // Timer
    //========================================================

    void timer_callback()
    {
        //====================================================
        // 起動後3秒待つ
        //====================================================

        auto now =
            std::chrono::steady_clock::now();

        auto elapsed =
            std::chrono::duration_cast<
                std::chrono::seconds>(
                now - start_time_).count();

        if (elapsed < 3)
            return;


        //====================================================
        // Leonardoから受信
        //====================================================

        bridge1_->update();


        //====================================================
        // Megaから受信
        //====================================================

        bridge2_->update();


        //====================================================
        // Leonardoへ送信
        //====================================================

        joy_msg_.data =
            joy1_data_;

        bridge1_->write(0);


        //====================================================
        // Megaへ送信
        //====================================================

        int tx =
            bridge2_->write(0);


        //====================================================
        // 5秒ごとに状態表示
        //====================================================

        static auto last_status_print =
            std::chrono::steady_clock::now();

        if (
            std::chrono::duration_cast<
                std::chrono::seconds>(
                now - last_status_print).count() >= 5)
        {
            last_status_print = now;


            //================================================
            // Leonardo
            //================================================

            RCLCPP_INFO(
                get_logger(),
                "Leonardo TX | X=%d Y=%d Rot=%d L1=%d Y=%d",
                joy_msg_.data.joyX,
                joy_msg_.data.joyY,
                joy_msg_.data.joyRot,
                joy_msg_.data.buttonL1,
                joy_msg_.data.buttonY);


            //================================================
            // Mega
            //================================================

            RCLCPP_INFO(
                get_logger(),
                "MEGA TX=%d | A=%d B=%d X=%d",
                tx,
                mega_msg_.data.buttonA,
                mega_msg_.data.buttonB,
                mega_msg_.data.buttonX);


            //================================================
            // Controller
            //================================================

            RCLCPP_INFO(
                get_logger(),
                "Controller1: %s",
                controller1_connected_
                    ? "Connected"
                    : "Disconnected");
        }
    }
   
};


//============================================================
// main
//============================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            SerialArduinoNode>());

    rclcpp::shutdown();

    return 0;
}