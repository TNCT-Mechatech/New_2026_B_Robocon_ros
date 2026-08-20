#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "SerialBridge.hpp"
#include "LinuxHardwareSerial.hpp"

#include <mutex>
#include <chrono>
#include <cstdint>

#define SERIAL_PATH "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0"//ポート左上足回りArduino指定　つける位置間違えないように

typedef struct
{
    int16_t joyX;
    int16_t joyY;
    int16_t joyRot;
} JoyData_t;
//ここでArdinoに送るJoyのデータ構造体を定義している。joyX, joyY, joyRotの3つのint16_t型の変数を持つ構造体で、ジョイスティックのX軸、Y軸、回転軸の値を格納するために使用される。



class SerialArduinoNode : public rclcpp::Node
{
public:

    SerialArduinoNode()
        : Node("serial_arduino")
          
    {
        init_serial(SERIAL_PATH);
        //↑ポートを指定して展開
        joy_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/controller/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy_callback,
                    this,
                    std::placeholders::_1));
        //↑"/controller/joy"と指定したスマホコントローラの値の読み取り
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SerialArduinoNode::timer_callback,
                    this));
    }   //↑20msごとにtimer_callback()を呼び出すタイマーを作成する。これにより、定期的にシリアル通信を行うことができる。

    ~SerialArduinoNode()
    {
         delete bridge_;
            delete serial_;
    }
//↑シリアル通信の終了時に、シリアル通信オブジェクトとブリッジオブジェクトを削除してメモリを解放する。
private:

   LinuxHardwareSerial *serial_;
    SerialBridge *bridge_;
   
    sb::Message<JoyData_t> joy_msg_;

    bool controller_connected_ = false;
    

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //--------------------------------------------------
    // Serial初期化
    //--------------------------------------------------

 
        bool init_serial(const std::string &port)
    {
        serial_ = new LinuxHardwareSerial(
        port.c_str(),
        B230400
    );
    //↑通信速度設定

    bridge_ = new SerialBridge(serial_);
    bridge_->add_frame(0, &joy_msg_);

    RCLCPP_INFO(
        get_logger(),
        "SerialBridge Connected"
    );

    return true;
    }
    

    //--------------------------------------------------
    // Joy受信
    //--------------------------------------------------

  
    void joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
       if (!controller_connected_)
    {
        controller_connected_ = true;
        RCLCPP_INFO(get_logger(), "Controller Connected!");
    }
    
    if (msg->axes.size() < 4)
        return;

    joy_msg_.data.joyX =
    static_cast<int16_t>(-msg->axes[0] * 255);

    joy_msg_.data.joyY =
    static_cast<int16_t>(-msg->axes[1] * 255);

    joy_msg_.data.joyRot =
    static_cast<int16_t>(-msg->axes[2] * 255);

}
//↑コントローラから読み取った値を変更、データ化
     void timer_callback()
    {
    bridge_->write(0);
     bridge_->update();
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