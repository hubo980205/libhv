// include/hv/SerialChannel.h
#ifndef HV_SERIAL_CHANNEL_H
#define HV_SERIAL_CHANNEL_H

#include "Channel.h"
#include "EventLoop.h"

namespace hv {

class SerialChannel : public Channel {
public:
    SerialChannel(EventLoop* loop = NULL);
    virtual ~SerialChannel();

    // 打开串口设备
    int open(const char* device, int baudrate = 9600);

    // 设置串口参数（数据位、校验、停止位等）
    int setParameters(int baudrate, int databits = 8, char parity = 'N', int stopbits = 1);

    // 发送数据
    int send(const void* data, int size);
    int send(const std::string& str) {
        return send(str.data(), str.size());
    }

    // 关闭串口
    void close();

    // 获取设备名
    const std::string& device() const { return device_; }

private:
    std::string device_;
    int         fd_ = -1;

    // 平台相关实现
    int __open_serial(const char* device, int baudrate);
    int __configure_serial(int fd, int baudrate, int databits, char parity, int stopbits);
};

} // namespace hv

#endif // HV_SERIAL_CHANNEL_H
