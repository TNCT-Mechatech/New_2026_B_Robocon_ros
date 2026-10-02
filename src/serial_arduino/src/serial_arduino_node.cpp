#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "SerialBridge.hpp"
#include "LinuxHardwareSerial.hpp"

#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>


//==================================================
// シリアルポート
//==================================================

#define SERIAL_PATH_1 \
    "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0"

#define SERIAL_PATH_2 \
    "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0"


//==================================================
// ジョイスティック最大値
//==================================================

#define MAX_VALUE (255 * 0.8)
#define DPAD_VALUE (MAX_VALUE * 0.7)


//==================================================
// Raspberry Pi → Leonardo
//==================================================

typedef struct
{
    int16_t joyX;
    int16_t joyY;
    int16_t joyRot;

    uint8_t buttonGear;
    uint8_t buttonSol;

} JoyData_t;


//==================================================
// Leonardo → Raspberry Pi
//==================================================

typedef struct
{
} ResponseData_t;


//==================================================
// Raspberry Pi → MEGA
//==================================================

typedef struct
{
    uint8_t buttonA;
    uint8_t buttonB;
    uint8_t buttonX;

    uint8_t buttonCollectROT;
    uint8_t buttonCollectHand;
    uint8_t buttonCollectBack;

    uint8_t buttonCamUP;
    uint8_t buttonCamDOWN;

    uint8_t buttonRollGo;
    uint8_t buttonRollBack;

    uint8_t buttonLookUp;
    uint8_t buttonLookDown;


} MEGA_t;


//==================================================
// MEGA → Raspberry Pi
//==================================================

typedef struct
{
    uint8_t buttonA;
    uint8_t buttonB;
    uint8_t buttonX;

} MEGA_Response_t;


//==================================================
// ROS2 Node
//==================================================

class SerialArduinoNode : public rclcpp::Node
{
public:

    SerialArduinoNode()
        : Node("serial_arduino")
    {
        init_serial(SERIAL_PATH_1);

        //==========================================
        // Controller 1
        //==========================================

        joy1_sub_ =
            this->create_subscription<sensor_msgs::msg::Joy>(
                "/controller/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy1_callback,
                    this,
                    std::placeholders::_1));


        //==========================================
        // Controller 2
        //==========================================

        joy2_sub_ =
            this->create_subscription<sensor_msgs::msg::Joy>(
                "/cont2/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy2_callback,
                    this,
                    std::placeholders::_1));


        //==========================================
        // 20ms周期
        //==========================================

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SerialArduinoNode::timer_callback,
                    this));


        //==========================================
        // 起動時間
        //==========================================

        start_time_ =
            std::chrono::steady_clock::now();

        last_controller1_time_ =
            start_time_;

        last_controller2_time_ =
            start_time_;


        RCLCPP_INFO(
            get_logger(),
            "Waiting 3 seconds before TX...");
    }


    ~SerialArduinoNode()
    {
        delete bridge1_;
        delete serial1_;

        delete bridge2_;
        delete serial2_;
    }


private:

    //==================================================
    // シリアル
    //==================================================

    LinuxHardwareSerial *serial1_ = nullptr;
    LinuxHardwareSerial *serial2_ = nullptr;

    SerialBridge *bridge1_ = nullptr;
    SerialBridge *bridge2_ = nullptr;


    //==================================================
    // SerialBridge Message
    //==================================================

    sb::Message<JoyData_t> joy_msg_{};
    sb::Message<ResponseData_t> response_msg_{};

    sb::Message<MEGA_t> mega_msg_{};
    sb::Message<MEGA_Response_t> mega_response_msg_{};


    //==================================================
    // コントローラー入力
    //==================================================

    JoyData_t joy1_data_ = {};


    //==================================================
    // コントローラー接続状態
    //==================================================

    bool controller1_connected_ = false;
    bool controller2_connected_ = false;


    //==================================================
    // 最後の入力受信
    //==================================================

    std::chrono::steady_clock::time_point
        last_controller1_time_;

    std::chrono::steady_clock::time_point
        last_controller2_time_;


    //==================================================
    // 未接続表示用
    //==================================================

    std::chrono::steady_clock::time_point
        last_controller1_warning_time_;

    std::chrono::steady_clock::time_point
        last_controller2_warning_time_;


    //==================================================
    // ROS
    //==================================================

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
        joy1_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
        joy2_sub_;

    rclcpp::TimerBase::SharedPtr timer_;


    //==================================================
    // 起動時間
    //==================================================

    std::chrono::steady_clock::time_point start_time_;


    //==================================================
    // シリアル初期化
    //==================================================

    bool init_serial(const std::string &port)
    {
        //==========================================
        // Leonardo
        //==========================================

        serial1_ =
            new LinuxHardwareSerial(
                port.c_str(),
                B230400);

        bridge1_ =
            new SerialBridge(serial1_);

        bridge1_->add_frame(
            0,
            &joy_msg_);

        bridge1_->add_frame(
            1,
            &response_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: Leonardo");


        //==========================================
        // MEGA
        //==========================================

        serial2_ =
            new LinuxHardwareSerial(
                SERIAL_PATH_2,
                B115200);

        bridge2_ =
            new SerialBridge(serial2_);

        bridge2_->add_frame(
            0,
            &mega_msg_);

        bridge2_->add_frame(
            1,
            &mega_response_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: MEGA");

        return true;
    }


    //==================================================
    // Controller 1
    //==================================================

    void joy1_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        //==========================================
        // 最終受信を更新
        //==========================================

        last_controller1_time_ =
            std::chrono::steady_clock::now();


        //==========================================
        // 接続状態
        //==========================================

        if (!controller1_connected_)
        {
            controller1_connected_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Controller1 Connected!");
        }


        //==========================================
        // 入力数確認
        //==========================================

        if (msg->axes.size() < 3)
            return;

        if (msg->buttons.size() < 15)
            return;


        //==========================================
        // スティック
        //==========================================

        int16_t stickX =
            static_cast<int16_t>(
                -msg->axes[0] * MAX_VALUE);

        int16_t stickY =
            static_cast<int16_t>(
                -msg->axes[1] * MAX_VALUE);

        int16_t stickRot =
            static_cast<int16_t>(
                -msg->axes[2] * MAX_VALUE);


        //==========================================
        // デッドゾーン
        //==========================================

        const int STICK_THRESHOLD = 20;

        if (std::abs(stickX) <= STICK_THRESHOLD)
            stickX = 0;

        if (std::abs(stickY) <= STICK_THRESHOLD)
            stickY = 0;

        if (std::abs(stickRot) <= STICK_THRESHOLD)
            stickRot = 0;


        //==========================================
        // 十字キー
        //==========================================

        int16_t dpadX = 0;
        int16_t dpadY = 0;

        bool dpadUp =
            msg->buttons[11];

        bool dpadDown =
            msg->buttons[12];

        bool dpadLeft =
            msg->buttons[13];

        bool dpadRight =
            msg->buttons[14];


        if (dpadUp && !dpadDown)
            dpadY = DPAD_VALUE;

        else if (dpadDown && !dpadUp)
            dpadY = -DPAD_VALUE;


        if (dpadLeft && !dpadRight)
            dpadX = -DPAD_VALUE;

        else if (dpadRight && !dpadLeft)
            dpadX = DPAD_VALUE;


        //==========================================
        // スティック・十字キー切り替え
        //==========================================

        bool stickMoving =
            (stickX != 0) ||
            (stickY != 0) ||
            (stickRot != 0);

        bool dpadMoving =
            (dpadX != 0) ||
            (dpadY != 0);


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


        //==========================================
        // Controller 1 → MEGA
        //==========================================

        mega_msg_.data.buttonA =
            static_cast<uint8_t>(
                msg->buttons[0]);

        mega_msg_.data.buttonB =
            static_cast<uint8_t>(
                msg->buttons[1]);

        mega_msg_.data.buttonX =
            static_cast<uint8_t>(
                msg->buttons[2]);

        mega_msg_.data.buttonCollectROT =
            static_cast<uint8_t>(
                msg->buttons[5]);

        mega_msg_.data.buttonCollectHand =
            static_cast<uint8_t>(
                msg->buttons[7]);
                
        mega_msg_.data.buttonCollectBack =
            static_cast<uint8_t>(
                msg->buttons[4]);
    }


    //==================================================
    // Controller 2
    //==================================================

    void joy2_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        //==========================================
        // 最終受信を更新
        //==========================================

        last_controller2_time_ =
            std::chrono::steady_clock::now();


        //==========================================
        // 接続状態
        //==========================================

        if (!controller2_connected_)
        {
            controller2_connected_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Controller2 Connected!");
        }


        //==========================================
        // 入力数確認
        //==========================================

        if (msg->buttons.size() < 15)
            return;


        //==========================================
        // Controller 2 → Leonardo
        //==========================================

        joy1_data_.buttonGear =
            static_cast<uint8_t>(
                msg->buttons[0]);

        joy1_data_.buttonSol =
            static_cast<uint8_t>(
                msg->buttons[1]);


        //==========================================
        // Controller 2 → MEGA
        //==========================================

        mega_msg_.data.buttonCamUP =
            static_cast<uint8_t>(
                msg->buttons[11]);

        mega_msg_.data.buttonCamDOWN =
            static_cast<uint8_t>(
                msg->buttons[12]);

        mega_msg_.data.buttonRollGo =
            static_cast<uint8_t>(
                msg->buttons[4]);

        mega_msg_.data.buttonRollBack =
            static_cast<uint8_t>(
                msg->buttons[5]);

        mega_msg_.data.buttonLookUp =
            static_cast<uint8_t>(
                msg->buttons[13]);

        mega_msg_.data.buttonLookDown =
            static_cast<uint8_t>(
                msg->buttons[14]);
    }


    //==================================================
    // タイマー
    //==================================================

    void timer_callback()
    {
        auto now =
            std::chrono::steady_clock::now();


        //==========================================
        // 起動後3秒待つ
        //==========================================

        auto elapsed =
            std::chrono::duration_cast<
                std::chrono::seconds>(
                    now - start_time_)
                .count();

        if (elapsed < 3)
            return;


        //==========================================
        // Controller 1 接続確認
        //==========================================

        auto controller1_elapsed =
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    now - last_controller1_time_)
                .count();


        if (controller1_elapsed >= 1000)
        {
            if (controller1_connected_)
            {
                controller1_connected_ = false;

                RCLCPP_WARN(
                    get_logger(),
                    "Controller1 Disconnected!");

                last_controller1_warning_time_ =
                    now;
            }
            else
            {
                auto warning_elapsed =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            now -
                            last_controller1_warning_time_)
                        .count();

                if (warning_elapsed >= 1000)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Controller1 is not connected.");

                    last_controller1_warning_time_ =
                        now;
                }
            }
        }


        //==========================================
        // Controller 2 接続確認
        //==========================================

        auto controller2_elapsed =
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    now - last_controller2_time_)
                .count();


        if (controller2_elapsed >= 1000)
        {
            if (controller2_connected_)
            {
                controller2_connected_ = false;

                RCLCPP_WARN(
                    get_logger(),
                    "Controller2 Disconnected!");

                last_controller2_warning_time_ =
                    now;
            }
            else
            {
                auto warning_elapsed =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            now -
                            last_controller2_warning_time_)
                        .count();

                if (warning_elapsed >= 1000)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Controller2 is not connected.");

                    last_controller2_warning_time_ =
                        now;
                }
            }
        }


        //==========================================
        // SerialBridge更新
        //==========================================

        bridge2_->update();
        bridge1_->update();


        //==========================================
        // Leonardoへ送信
        //==========================================

        joy_msg_.data =
            joy1_data_;

        bridge1_->write(0);


        //==========================================
        // MEGAへ送信
        //==========================================

        bridge2_->write(0);

        static int debug_count = 0;

        debug_count++;

        if (debug_count >= 50)
        {
            RCLCPP_INFO(
            get_logger(),
            "MEGA TX: A=%d B=%d X=%d ROT=%d HAND=%d CAMUP=%d CAMDOWN=%d ROLL=%d BACK=%d LOOKUP=%d LOOKDOWN=%d",
        
            mega_msg_.data.buttonA,
            mega_msg_.data.buttonB,
            mega_msg_.data.buttonX,
            mega_msg_.data.buttonCollectROT,
            mega_msg_.data.buttonCollectHand,
            mega_msg_.data.buttonCamUP,
            mega_msg_.data.buttonCamDOWN,
            mega_msg_.data.buttonRollGo,
            mega_msg_.data.buttonRollBack,
            mega_msg_.data.buttonLookUp,
            mega_msg_.data.buttonLookDown
            );

            debug_count = 0;
        }
    }
};


//==================================================
// main
//==================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);


    rclcpp::spin(
        std::make_shared<
            SerialArduinoNode>());


    rclcpp::shutdown();

    return 0;
}