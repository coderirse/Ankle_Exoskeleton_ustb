/**
 * 足底双开关采集 — ESP32-S3 版 (正点原子 DNESP32S3)
 * 移植自 STM32 心跳固件 (src/stm_switch/足底双开关/User/main_heartbeat.c)
 *
 * 协议 (与 STM32 版完全一致, ROS 侧无需任何改动):
 *   0x41 = 脚跟着地  0x42 = 全掌接触
 *   0x43 = 脚跟离地  0x44 = 脚尖离地
 *   状态变化立即发送 + 每 ~50ms 心跳重发当前状态
 *
 * 引脚分配 (详见 接线说明.md):
 *   IO4  ← 后脚跟开关 (按下接地, 内部上拉)
 *   IO5  ← 前脚掌开关 (按下接地, 内部上拉)
 *   IO6  → 外接 LED1 (后脚跟指示, 高电平点亮, 串 1kΩ 电阻到地)
 *   IO7  → 外接 LED2 (前脚掌指示, 高电平点亮, 串 1kΩ 电阻到地)
 *   IO1  → 板载红色 LED (低电平点亮):
 *            0x42 双开关闭合=常亮; 0x41/0x43 单开关=快闪; 0x44 摆动=慢闪
 *   输出通道: 原生 USB (USB Serial/JTAG, Type-C 口), 免接线
 *
 * 编译烧录:
 *   source ~/esp/esp-idf/export.sh
 *   idf.py set-target esp32s3
 *   idf.py build flash
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/* ── 引脚 ── */
#define PIN_HEEL        GPIO_NUM_4
#define PIN_TOE         GPIO_NUM_5
#define PIN_LED_HEEL    GPIO_NUM_6    /* 外接, 高电平点亮 */
#define PIN_LED_TOE     GPIO_NUM_7    /* 外接, 高电平点亮 */
#define PIN_LED_BOARD   GPIO_NUM_1    /* 板载红色, 低电平点亮 */

/* ── 参数 ── */
#define DEBOUNCE_MS     20            /* 20ms 消抖 */
#define HEARTBEAT_MS    50            /* 心跳周期 */

/* 2026-08-10: 步态转移图过滤 — 长导线EMI爆发(~0.5s)会产生四状态循环扫掠,
   纯时间消抖挡不住; 但爆发中的跳变(如44→43, 42→44)违反物理步态顺序, 可直接拒绝。
   合法转移: 44→41(脚跟着地) 41→42(全掌) 42→43(跟离) 43→44(尖离), 及相邻回退 */
static int transition_allowed(uint8_t from, uint8_t to)
{
    switch (from) {
    case 0x41: return (to == 0x42 || to == 0x44);
    case 0x42: return (to == 0x41 || to == 0x43);
    case 0x43: return (to == 0x42 || to == 0x44);
    case 0x44: return (to == 0x41);
    }
    return 0;
}

/* 板载 LED 闪烁参数 (1ms 循环计数) */
#define BLINK_FAST_HALF_MS  100       /* 单开关: 5Hz 快闪 */
#define BLINK_SLOW_HALF_MS  500       /* 摆动相: 1Hz 慢闪 */

static int rd_heel(void) { return gpio_get_level(PIN_HEEL) ? 0 : 1; }  /* 按下接地=0 → 返回1 */
static int rd_toe(void)  { return gpio_get_level(PIN_TOE)  ? 0 : 1; }

static uint8_t to_cmd(int h, int t)
{
    if (h && !t) return 0x41;
    if (h &&  t) return 0x42;
    if (!h && t) return 0x43;
    return 0x44;
}

static void send_cmd(uint8_t cmd)
{
    /* stdout 走 USB Serial/JTAG (原生 Type-C 口).
       协议字节为 0x41~0x44, 不含 \n, 不受换行转换影响 */
    fwrite(&cmd, 1, 1, stdout);
    fflush(stdout);
}

static void gpio_setup(void)
{
    /* 开关: 上拉输入 */
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_HEEL) | (1ULL << PIN_TOE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    /* LED: 推挽输出 */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_LED_HEEL) | (1ULL << PIN_LED_TOE) | (1ULL << PIN_LED_BOARD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    gpio_set_level(PIN_LED_HEEL, 0);   /* 外接 LED 灭 (低灭) */
    gpio_set_level(PIN_LED_TOE, 0);
    gpio_set_level(PIN_LED_BOARD, 1);  /* 板载 LED 灭 (高灭) */
}

/* 板载 LED 按当前步态状态做组合指示 (方案1) */
static void board_led_update(uint8_t cmd, uint32_t ms)
{
    switch (cmd) {
    case 0x42:  /* 双开关闭合 (站立): 常亮 */
        gpio_set_level(PIN_LED_BOARD, 0);
        break;
    case 0x41:  /* 单开关: 快闪 */
    case 0x43:
        gpio_set_level(PIN_LED_BOARD, (ms / BLINK_FAST_HALF_MS) % 2 ? 0 : 1);
        break;
    case 0x44:  /* 摆动相 (双开关都松开): 慢闪 */
    default:
        gpio_set_level(PIN_LED_BOARD, (ms / BLINK_SLOW_HALF_MS) % 2 ? 0 : 1);
        break;
    }
}

void app_main(void)
{
    gpio_setup();

    int sh = rd_heel(), st = rd_toe();          /* 消抖后的稳定状态 */
    uint8_t last_sent = to_cmd(sh, st);
    int hc = 0, tc = 0;                          /* 消抖计数 */
    uint32_t heartbeat = 0;
    uint32_t disagree_ms = 0;                    /* 转移图不一致计时 */
    uint32_t dwell42_ms = 0;                     /* 44→42 平放专用持续计时 */
    uint32_t ms = 0;

    send_cmd(last_sent);                         /* 上电立即上报当前状态 */

    while (1) {
        /* 消抖: 连续 DEBOUNCE_MS 次一致才更新 (与 STM32 版一致) */
        int v = rd_heel();
        if (v == sh) hc = 0; else if (++hc >= DEBOUNCE_MS) { hc = 0; sh = v; }
        v = rd_toe();
        if (v == st) tc = 0; else if (++tc >= DEBOUNCE_MS) { tc = 0; st = v; }

        /* 外接 LED: 直接指示对应开关 (高电平点亮) */
        gpio_set_level(PIN_LED_HEEL, sh);
        gpio_set_level(PIN_LED_TOE, st);
        /* 板载 LED: 组合状态指示 */
        board_led_update(last_sent, ms);

        uint8_t cmd = to_cmd(sh, st);
        if (cmd != last_sent) {
            /* 整脚平放时 44→42 是合法的 (双开关同时闭合), 但爆发噪声里 42 也常出现;
               折中: 44→42 需持续 80ms 才接受 (真实平放≥200ms, 噪声段一般<50ms) */
            if (cmd == 0x42 && last_sent == 0x44)
                dwell42_ms++;
            else
                dwell42_ms = 0;

            if (transition_allowed(last_sent, cmd) ||
                (cmd == 0x42 && last_sent == 0x44 && dwell42_ms >= 80)) {
                last_sent = cmd;
                send_cmd(cmd);                       /* 合法状态变化 → 立即发送 */
                heartbeat = 0;
                disagree_ms = 0;
            } else if (++disagree_ms > 500) {
                /* 原始状态与输出持续不一致 500ms → 转移图可能漏了合法路径,
                   强制重同步防锁死 */
                last_sent = cmd;
                send_cmd(cmd);
                heartbeat = 0;
                disagree_ms = 0;
            }
            /* 不合法跳变: 视为 EMI 噪声忽略 */
        } else {
            disagree_ms = 0;
            if (++heartbeat >= HEARTBEAT_MS) {
                heartbeat = 0;
                send_cmd(last_sent);                 /* 心跳重发 */
            }
        }

        ms++;
        /* 1ms 循环节拍 (需 CONFIG_FREERTOS_HZ=1000, 见 sdkconfig.defaults;
           让出 CPU 给 IDLE 任务, 避免任务看门狗触发) */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
