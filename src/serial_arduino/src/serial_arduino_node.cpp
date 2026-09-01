#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "SerialBridge.hpp"
#include "LinuxHardwareSerial.hpp"

#include <mutex>
#include <chrono>
#include <cstdint>

#define SERIAL_PATH_1 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0" // ポート左上 足回りArduino指定
#define SERIAL_PATH_2 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0" // ポート左下 Arduino指定

//============================================================
// Leonardoに送るJoyデータ
//============================================================

typedef struct
{
    int16_t joyX;
    int16_t joyY;
    int16_t joyRot;

} JoyData_t;


//============================================================
// MEGAに送るボタンデータ
//============================================================

typedef struct
{
    uint8_t buttonB;
    uint8_t buttonA;
    uint8_t buttonX;

} MEGA_t;


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
        // Serial初期化
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
        // Controller2
        //====================================================

        joy2_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/cont2/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy2_callback,
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

    LinuxHardwareSerial *serial1_;
    LinuxHardwareSerial *serial2_;

    SerialBridge *bridge1_;
    SerialBridge *bridge2_;


    //========================================================
    // Message
    //========================================================

    sb::Message<JoyData_t> joy_msg_;

    sb::Message<MEGA_t> mega_msg_;


    //========================================================
    // Controller状態
    //========================================================

    bool controller1_connected_ = false;
    bool controller2_connected_ = false;


    //========================================================
    // ROS2
    //========================================================

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy2_sub_;

    rclcpp::TimerBase::SharedPtr timer_;


    //========================================================
    // Serial初期化
    //========================================================

    bool init_serial(const std::string &port)
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

        bridge1_->add_frame(
            0,
            &joy_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: Leonardo");


        //====================================================
        // MEGA
        //====================================================

        serial2_ =
            new LinuxHardwareSerial(
                SERIAL_PATH_2,
                B115200);

        bridge2_ =
            new SerialBridge(serial2_);

        bridge2_->add_frame(
            0,
            &mega_msg_);


        RCLCPP_INFO(
            get_logger(),
            "SerialBridge Connected: MEGA");


        return true;
    }


    //========================================================
    // Controller1
    //========================================================

    void joy1_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!controller1_connected_)
        {
            controller1_connected_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Controller1 Connected!");
        }


        if (msg->axes.size() < 4)
            return;

        if (msg->buttons.size() < 3)
            return;


        //====================================================
        // Leonardoへ送るJoy
        //====================================================

        joy_msg_.data.joyX =
            static_cast<int16_t>(
                -msg->axes[0] * 255);

        joy_msg_.data.joyY =
            static_cast<int16_t>(
                -msg->axes[1] * 255);

        joy_msg_.data.joyRot =
            static_cast<int16_t>(
                -msg->axes[2] * 255);


        //====================================================
        // MEGAへ送るボタン
        //====================================================

        mega_msg_.data.buttonA =
            static_cast<uint8_t>(
                msg->buttons[0]);

        mega_msg_.data.buttonB =
            static_cast<uint8_t>(
                msg->buttons[1]);

        mega_msg_.data.buttonX =
            static_cast<uint8_t>(
                msg->buttons[2]);
    }


    //========================================================
    // Controller2
    //========================================================

    void joy2_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!controller2_connected_)
        {
            controller2_connected_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Controller2 Connected!");
        }


        if (msg->axes.size() < 4)
            return;

        if (msg->buttons.size() < 3)
            return;
    }


    //========================================================
    // Timer
    //========================================================

    void timer_callback()
    {
        //====================================================
        // Leonardoへ送信
        //====================================================

        bridge1_->write(0);


        //====================================================
        // MEGAへ送信
        //====================================================

        int tx =
            bridge2_->write(0);


        //====================================================
        // 5秒ごとに状態表示
        //====================================================

        static auto last_status_print =
            std::chrono::steady_clock::now();

        auto now =
            std::chrono::steady_clock::now();

        if (
            std::chrono::duration_cast<
                std::chrono::seconds>(
                now - last_status_print).count() >= 5)
        {
            last_status_print = now;

            RCLCPP_INFO(
                get_logger(),
                "MEGA TX=%d | A=%d B=%d X=%d",
                tx,
                mega_msg_.data.buttonA,
                mega_msg_.data.buttonB,
                mega_msg_.data.buttonX);


            RCLCPP_INFO(
                get_logger(),
                "Controller1: %s, Controller2: %s",
                controller1_connected_
                    ? "Connected"
                    : "Disconnected",
                controller2_connected_
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