#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "SerialBridge.hpp"
#include "LinuxHardwareSerial.hpp"

#include <mutex>
#include <chrono>
#include <cstdint>

#define SERIAL_PATH_1 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0"//ポート左上足回りArduino指定　つける位置間違えないように
#define SERIAL_PATH_2 "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0"//ポート左下Arduino指定　つける位置間違えないように

typedef struct
{
    int16_t joyX;
    int16_t joyY;
    int16_t joyRot;
} JoyData_t;
//ここでArdinoLeonardoに送るJoyのデータ構造体を定義している。joyX, joyY, joyRotの3つのint16_t型の変数を持つ構造体で、ジョイスティックのX軸、Y軸、回転軸の値を格納するために使用される。

typedef struct
{
    uint8_t buttonB;
    uint8_t buttonA;

    int16_t joy2X;
    int16_t joy2Y;
    int16_t joy2Rot;
} MEGA_t;
//ここでArdinoMEGAに送るButtonのデータ構造体を


class SerialArduinoNode : public rclcpp::Node
{
public:

    SerialArduinoNode()
        : Node("serial_arduino")
          
    {
        init_serial(SERIAL_PATH_1);
        //↑ポートを指定して展開
        joy1_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/controller/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy1_callback,
                    this,
                    std::placeholders::_1));
        //↑"/controller/joy"と指定したスマホコントローラの値の読み取り
        joy2_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/cont2/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy2_callback,
                    this,
                    std::placeholders::_1));
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SerialArduinoNode::timer_callback,
                    this));
    }   //↑20msごとにtimer_callback()を呼び出すタイマーを作成する。これにより、定期的にシリアル通信を行うことができる。

    ~SerialArduinoNode()
    {
            delete bridge1_;
            delete serial1_;

            delete bridge2_;
            delete serial2_;
    }
//↑シリアル通信の終了時に、シリアル通信オブジェクトとブリッジオブジェクトを削除してメモリを解放する。
private:

   LinuxHardwareSerial *serial1_;
   LinuxHardwareSerial *serial2_;

    SerialBridge *bridge1_;
    SerialBridge *bridge2_;

    sb::Message<JoyData_t> joy_msg_;
    sb::Message<MEGA_t> mega_msg_;

    bool controller1_connected_ = false;
    bool controller2_connected_ = false;

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;
    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy2_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //--------------------------------------------------
    // Serial初期化
    //--------------------------------------------------

 
        bool init_serial(const std::string &port)
    {
        serial1_ = new LinuxHardwareSerial(
        port.c_str(),
        B230400
    );
    //↑Leonardo通信速度設定

    bridge1_ = new SerialBridge(serial1_);
    bridge1_->add_frame(0, &joy_msg_);

    RCLCPP_INFO(
        get_logger(),
        "SerialBridge Connected: Leonardo"
    );

    serial2_ = new LinuxHardwareSerial(
        SERIAL_PATH_2,
        B230400
    );
    //↑MEGA通信速度設定

    bridge2_ = new SerialBridge(serial2_);
    bridge2_->add_frame(0, &mega_msg_);

    RCLCPP_INFO(
        get_logger(),
        "SerialBridge Connected: MEGA"
    );

    return true;
    }
    

    //--------------------------------------------------
    // Joy受信
    //--------------------------------------------------

  
    void joy1_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
       if (!controller1_connected_)
    {
        controller1_connected_ = true;
        RCLCPP_INFO(get_logger(), "Controller1 Connected!");
    }
    
    if (msg->axes.size() < 4)
        return;
    if (msg->buttons.size() < 2)
        return;

    joy_msg_.data.joyX =
    static_cast<int16_t>(-msg->axes[0] * 255);

    joy_msg_.data.joyY =
    static_cast<int16_t>(-msg->axes[1] * 255);

    joy_msg_.data.joyRot =
    static_cast<int16_t>(-msg->axes[2] * 255);

    mega_msg_.data.buttonA =
    static_cast<uint8_t>(msg->buttons[0]);

    mega_msg_.data.buttonB = 
    static_cast<uint8_t>(msg->buttons[1]);

   
}
//↑コントローラ1から読み取った値を変更、データ化
    void joy2_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
       if (!controller2_connected_)
    {
        controller2_connected_ = true;
        RCLCPP_INFO(get_logger(), "Controller2 Connected!");
    }
    
    if (msg->axes.size() < 4)
        return;
    if (msg->buttons.size() < 2)
        return;

    mega_msg_.data.joy2X =
    static_cast<int16_t>(-msg->axes[0] * 255);

    mega_msg_.data.joy2Y =
    static_cast<int16_t>(-msg->axes[1] * 255);

    mega_msg_.data.joy2Rot =
    static_cast<int16_t>(-msg->axes[2] * 255);



   
}
     void timer_callback()
    {
    bridge1_->write(0);
     bridge1_->update();
    bridge2_->write(0);
     bridge2_->update();
    }
    //↑実際に送信

   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SerialArduinoNode>());

    rclcpp::shutdown();

    return 0;
}