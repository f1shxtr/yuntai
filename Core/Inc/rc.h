#ifndef RC_H
#define RC_H

#include <stdint.h>

#define RC_TIMEOUT_MS 1000   // 超过这个判定断开

// 开关状态

class RC_Class
{
public:

    enum SwitchState {UP=1, MID=3, DOWN=2};
    struct
{
    float ch0;
    float ch1;
    float ch2;
    float ch3;
    SwitchState s1;
    SwitchState s2;
}rc;
    uint8_t rx_buf[18];
    uint8_t pData[18];
    volatile uint32_t last_receive_tick; // 上次接收时间
    volatile uint8_t data_ready;
    uint8_t is_connected = 0;
    RC_Class();

    void init();   // 初始化 DMA 接收
    void handle(); // 主循环调用解析数据

    void parse();  // 内部解析函数
};

#endif