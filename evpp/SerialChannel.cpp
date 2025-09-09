// src/SerialChannel.cpp
#include "hv/SerialChannel.h"
#include "hv/hlog.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#endif

namespace hv {

SerialChannel::SerialChannel(EventLoop* loop) : Channel(loop) {
}

SerialChannel::~SerialChannel() {
    close();
}

int SerialChannel::open(const char* device, int baudrate) {
    if (!device) return -1;
    device_ = device;

    int fd = __open_serial(device, baudrate);
    if (fd < 0) {
        hloge("Failed to open serial port: %s", device);
        return -1;
    }

    // 初始化 Channel（关键！）
    Channel::init(fd, Channel::READ); // 监听可读事件
    if (loop_) {
        loop_->addChannel(this); // 注册到事件循环
    }

    fd_ = fd;
    return 0;
}

void SerialChannel::close() {
    if (fd_ != -1) {
        if (loop_) {
            loop_->removeChannel(this);
        }
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialChannel::send(const void* data, int size) {
    if (fd_ == -1) return -1;
    int n = ::write(fd_, data, size);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            hlogw("Serial write error: %s", strerror(errno));
        }
        return -1;
    }
    return n;
}

// ============= Linux 实现 =============
#ifndef _WIN32

int SerialChannel::__open_serial(const char* device, int baudrate) {
    int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        hloge("open %s failed: %s", device, strerror(errno));
        return -1;
    }

    if (__configure_serial(fd, baudrate, 8, 'N', 1) != 0) {
        ::close(fd);
        return -1;
    }

    return fd;
}

int SerialChannel::__configure_serial(int fd, int baudrate, int databits, char parity, int stopbits) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        hloge("tcgetattr failed");
        return -1;
    }

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:  speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200:speed = B115200; break;
        default:    speed = B9600; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 数据位
    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default: tty.c_cflag |= CS8; break;
    }

    // 校验位
    tty.c_cflag &= ~PARENB;
    tty.c_iflag &= ~(INPCK | ISTRIP);
    if (parity == 'O') {
        tty.c_cflag |= PARENB | PARODD;
        tty.c_iflag |= INPCK | ISTRIP;
    } else if (parity == 'E') {
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
        tty.c_iflag |= INPCK | ISTRIP;
    }

    // 停止位
    if (stopbits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        tty.c_cflag &= ~CSTOPB;
    }

    tty.c_cflag |= CREAD | CLOCAL; // 接收使能 + 忽略控制线
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1s 超时

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        hloge("tcsetattr failed");
        return -1;
    }

    return 0;
}

#endif // !_WIN32

// Windows 实现（略，结构类似，用 CreateFile + DCB）
#ifdef _WIN32
// ...
#endif

} // namespace hv
