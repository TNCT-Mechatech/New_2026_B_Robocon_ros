#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <mutex>
#include <chrono>
#include <cstdint>

class SerialArduinoNode : public rclcpp::Node
{
public:

    SerialArduinoNode()
        : Node("serial_arduino"),
          serial_fd_(-1)
    {
        init_serial("/dev/ttyACM0");

        joy_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Joy>(
                "/controller/joy",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SerialArduinoNode::joy_callback,
                    this,
                    std::placeholders::_1));

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(
                    &SerialArduinoNode::timer_callback,
                    this));
    }

    ~SerialArduinoNode()
    {
        if(serial_fd_ >= 0)
            close(serial_fd_);
    }

private:

    int serial_fd_;

    int16_t joy_x_ = 0;
    int16_t joy_y_ = 0;
    int16_t joy_rot_ = 0;

    bool controller_connected_ = false;

    std::mutex serial_mutex_;

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //--------------------------------------------------
    // Serial初期化
    //--------------------------------------------------

    bool init_serial(const std::string& port)
    {
        serial_fd_ =
            open(
                port.c_str(),
                O_RDWR | O_NOCTTY | O_SYNC);

        if(serial_fd_ < 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot open %s",
                port.c_str());
            return false;
        }

        termios tty{};

        tcgetattr(serial_fd_, &tty);

        cfsetispeed(&tty, B230400);
        cfsetospeed(&tty, B230400);

        tty.c_cflag =
            (tty.c_cflag & ~CSIZE) | CS8;

        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tty.c_cflag |=
            CLOCAL | CREAD;

        tty.c_cflag &=
            ~(PARENB | CSTOPB | CRTSCTS);

        tcsetattr(
            serial_fd_,
            TCSANOW,
            &tty);

        sleep(2);

        RCLCPP_INFO(
            get_logger(),
            "Serial Connected");

        return true;
    }

    //--------------------------------------------------
    // Joy受信
    //--------------------------------------------------

    /*void joy_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Joy callback");

        if(msg->axes.size() < 4)
            return;

        joy_x_ =
            static_cast<int16_t>(
                -msg->axes[0] * 255);

        joy_y_ =
            static_cast<int16_t>(
                msg->axes[1] * 255);

        joy_rot_ =
            static_cast<int16_t>(
                -msg->axes[2] * 255);
    }*/
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

    joy_x_ =
        static_cast<int16_t>(-msg->axes[0] * 255);

    joy_y_ =
        static_cast<int16_t>( -msg->axes[1] * 255);

    joy_rot_ =
        static_cast<int16_t>(-msg->axes[2] * 255);

    /*RCLCPP_INFO(
        get_logger(),
        "X:%d  Y:%d  ROT:%d",
        joy_x_,
        joy_y_,
        joy_rot_);*/
}
        //--------------------------------------------------
    // Timer
    //--------------------------------------------------

    void timer_callback()
    {
        if (serial_fd_ < 0)
            return;

        send_data();
    }

    //--------------------------------------------------
    // 6Byte送信
    //--------------------------------------------------

    void send_data()
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        uint8_t tx_buf[6];

        // joyX
        tx_buf[0] = (joy_x_ >> 8) & 0xFF;
        tx_buf[1] = joy_x_ & 0xFF;

        // joyY
        tx_buf[2] = (joy_y_ >> 8) & 0xFF;
        tx_buf[3] = joy_y_ & 0xFF;

        // joyRot
        tx_buf[4] = (joy_rot_ >> 8) & 0xFF;
        tx_buf[5] = joy_rot_ & 0xFF;

       // RCLCPP_INFO(get_logger(), "before write");
        int ret = write(serial_fd_, tx_buf, 6);
      //  RCLCPP_INFO(get_logger(), "after write");
        //int ret = write(serial_fd_, tx_buf, 6);

        if (ret != 6)
        {
            RCLCPP_WARN(
                get_logger(),
                "Write Failed");
            return;
        }
        
        /*RCLCPP_INFO(
            get_logger(),
            "JOY X:%d  Y:%d  ROT:%d",
            joy_x_,
            joy_y_,
            joy_rot_);*/
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SerialArduinoNode>());

    rclcpp::shutdown();

    return 0;
}