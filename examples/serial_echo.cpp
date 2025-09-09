#include "hv/SerialChannel.h"
#include "hv/EventLoop.h"
#include <iostream>

using namespace hv;

void onMessage(Channel* ch, Buffer* buf) {
    printf("Received: %.*s\n", (int)buf->size(), buf->data());
    // 回显
    ch->write(buf);
}

int main() {
    EventLoop loop;

    SerialChannel serial(&loop);
    serial.open("/dev/ttyUSB0", 115200);

    serial.setOnMessage(onMessage);

    loop.run();
    return 0;
}
