/*
 * Copyright (c) 2020. Mohit Deshpande.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <motion_controller_msgs/WheelEncoders.h>

#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <termios.h>

// [Hz] communications rate for sending cmd_vel & receiving encoders
static constexpr auto MCU_COMMS_RATE = 200;
// topic for command requests
static constexpr auto CMD_VEL_TOPIC = "/cmd_vel";
static constexpr auto WHEEL_ENCODER_TOPIC = "/wheels/encoders";

// size of serial buffer to read encoders
static constexpr auto SERIAL_BUFFER_SIZE = 32;
// size of serial buffer to write (v, w) to MCU
static constexpr auto CMD_VEL_BUFFER_SIZE = 9;

static constexpr auto MCU_PATH = "/dev/ttyACM0";
static constexpr auto MCU_BAUD_RATE = B19200;
// [us] timeout waiting for the MCU to reboot
static constexpr auto MCU_REBOOT_TIMEOUT = 1000*1000;
// magic bytes that are used to signal a read/write for the MCU
static constexpr auto MCU_MAGIC_BYTE_WRITE = 255;
static constexpr auto MCU_MAGIC_BYTE_READ = 254;

geometry_msgs::TwistStamped::ConstPtr cmd_vel_ = nullptr;

int ConfigureMcuConnection() {
    int fd = open(MCU_PATH, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ROS_ERROR_STREAM("Error connecting to MCU (" << errno << ")!!! " << strerror(errno));
    } else {
        ROS_INFO_STREAM("Opened MCU: " << fd);
    }

    // Honestly, I don't fully understand this termios stuff, but hey, it works so...
    struct termios toptions{};
    tcgetattr(fd, &toptions);
    cfsetispeed(&toptions, MCU_BAUD_RATE);
    cfsetospeed(&toptions, MCU_BAUD_RATE);
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no hardware flow control
    toptions.c_cflag &= ~CRTSCTS;
    // enable receiver, ignore status lines
    toptions.c_cflag |= CREAD | CLOCAL;
    // disable input/output flow control, disable restart chars
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    // disable canonical input, disable echo,
    // disable visually erase chars,
    // disable terminal-generated signals
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // disable output processing
    toptions.c_oflag &= ~OPOST;

    // blocks until buffer is full
    toptions.c_cc[VMIN] = SERIAL_BUFFER_SIZE;
    // no minimum time to wait before read returns
    toptions.c_cc[VTIME] = 0;

    // commit the options
    tcsetattr(fd, TCSANOW, &toptions);

    // Wait for the MCU
    usleep(MCU_REBOOT_TIMEOUT);
    // Flush anything already in the serial buffer
    tcflush(fd, TCIFLUSH);

    return fd;
}

void CmdVelSub(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
    cmd_vel_ = cmd_vel;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe(CMD_VEL_TOPIC, 1, &CmdVelSub);
    ros::Publisher encoders_pub = nh.advertise<motion_controller_msgs::WheelEncoders>(WHEEL_ENCODER_TOPIC, 1);
    motion_controller_msgs::WheelEncoders encoders_msg;

    int mcu_fd = ConfigureMcuConnection();

    // set up buffers
    unsigned char mcu_serial_buff[SERIAL_BUFFER_SIZE];
    int32_t encoders[2];

    unsigned char cmd_vel_buff[CMD_VEL_BUFFER_SIZE];
    cmd_vel_buff[0] = MCU_MAGIC_BYTE_WRITE;
    float cmd_vel[2] = {0., 0.};
    bool prev_cmd_halt = true;

    ROS_INFO("Motion controller initialized!");

    ros::Rate rate(MCU_COMMS_RATE);
    while (ros::ok()) {
        /*
         * Command requests to the MCU
         */
        if (cmd_vel_) {
            cmd_vel[0] = cmd_vel_->twist.linear.x;
            cmd_vel[1] = cmd_vel_->twist.angular.z;
        }

        bool halt = (std::abs(cmd_vel[0]) < std::numeric_limits<float>::epsilon() &&
                        std::abs(cmd_vel[1]) < std::numeric_limits<float>::epsilon());
        if (!halt || !prev_cmd_halt) {
            // only print if the command is nonzero
            ROS_INFO_STREAM_THROTTLE(1. / 10, "cmd_vel v: " << cmd_vel[0] << " w: " << cmd_vel[1]);
        }
        // always print a final v=0, w = 0
        if (halt && !prev_cmd_halt) {
            ROS_INFO_STREAM("cmd_vel v: " << cmd_vel[0] << " w: " << cmd_vel[1]);
        }
        prev_cmd_halt = halt;

        // writing cmd_vel into a buffer, AFTER the command byte at cmd_vel_buff[0]
        memcpy(&cmd_vel_buff[1], cmd_vel, sizeof(cmd_vel));
        int bytes_written = write(mcu_fd, cmd_vel_buff, sizeof(cmd_vel_buff));
        if (bytes_written != sizeof(cmd_vel_buff)) {
            ROS_ERROR_STREAM("Expected to write " << sizeof(cmd_vel_buff)
                                                 << " but wrote " << bytes_written);
        }

        /*
         * Reading wheel encoders
         */
        int bytes_read = read(mcu_fd, mcu_serial_buff, SERIAL_BUFFER_SIZE);
        int i = 0;
        while (i < bytes_read) {
            // iterate forward through the buffer until the first signal byte
            while (mcu_serial_buff[i++] != MCU_MAGIC_BYTE_READ && i < bytes_read);

            // we can't read enough data; skip this buffer for now
            // TODO: do something smarter here like reading the remaining bytes
            if (i + sizeof(encoders) >= bytes_read) continue;

            memcpy(encoders, &mcu_serial_buff[i], sizeof(encoders));

            // publish the encoders message
            encoders_msg.header.stamp = ros::Time::now();
            encoders_msg.left = encoders[0];
            encoders_msg.right = encoders[1];
            encoders_pub.publish(encoders_msg);
        }

        // reset buffers
        memset(mcu_serial_buff, 0, SERIAL_BUFFER_SIZE);
        memset(&cmd_vel_buff[1], 0, sizeof(cmd_vel));

        // reset command velocity
        //cmd_vel_.twist.linear.x = 0;
        //cmd_vel_.twist.angular.z = 0;

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}

