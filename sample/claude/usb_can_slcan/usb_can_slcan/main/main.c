/*
 * USB-CAN adapter (SLCAN compatible) for ESP32-WROOM-32 / ESP-IDF v5.x
 *
 *   PC <-USB-> CP2102N <-UART0-> ESP32 (SLCAN + TWAI) <-> TJA1051T <-> CAN bus
 *
 * Pins:  TWAI_TX=GPIO5  TWAI_RX=GPIO4  RX LED=GPIO16  TX LED=GPIO17  ERR LED=GPIO18
 *
 * Tasks: uart_rx_task (cmd parser) / uart_tx_task / can_task (TWAI RX+TX+alerts)
 *        led_task / config_task (NVS)
 *
 * can_task merges the "can_rx / can_tx / alert" roles so that a single task owns
 * the TWAI driver; this avoids mutex starvation between polling tasks.
 *
 * Extension commands (not part of Lawicel SLCAN):
 *   U6/U7/U8  UART baud 921600/2000000/3000000 (saved to NVS)
 *   A0/A1     auto-start disable/enable (RAM, persisted by W)
 *   W         save bitrate, last mode, auto-start, serial to NVS
 *   n<serial> set serial number (max 16 ASCII chars, persisted by W)
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

/* ---------------- configuration ---------------- */
#define PIN_TWAI_TX     GPIO_NUM_5
#define PIN_TWAI_RX     GPIO_NUM_4
#define PIN_LED_RX      GPIO_NUM_16
#define PIN_LED_TX      GPIO_NUM_17
#define PIN_LED_ERR     GPIO_NUM_18   /* spec rev0.2: moved from GPIO2 (strapping pin) */

#define UART_PORT       UART_NUM_0
#define UART_RX_BUF     1024
#define UART_TXQ_LEN    600           /* ~24KB: replaces the 16KB UART TX buffer */
#define CAN_TXQ_LEN     64
#define CAN_RX_DRV_LEN  256
#define SL_LINE_MAX     40            /* max SLCAN line incl. CR is 31 bytes */

#define HW_VER          "0100"
#define FW_VER          "0100"

#define LED_PULSE_MS    100

#define ALERTS (TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | \
                TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BELOW_ERR_WARN |             \
                TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE |                       \
                TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |                     \
                TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN |             \
                TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR)

#define SL_DEBUG 0   /* 切り分けが終わったら 0 に戻す */

/* SLCAN 'F' status bits */
enum {
    ST_RXFULL  = 0x01,  /* RX queue / UART TX overflow, frame dropped */
    ST_TXFULL  = 0x02,  /* CAN TX queue full / transmit rejected      */
    ST_WARN    = 0x04,
    ST_OVERRUN = 0x08,
    ST_PASSIVE = 0x20,
    ST_ARBLOST = 0x40,
    ST_BUSERR  = 0x80,
};
enum { ERR_NONE = 0, ERR_WARN, ERR_PASSIVE, ERR_BUSOFF };

static const uint32_t UART_BAUDS[] = { 921600, 2000000, 3000000 };   /* U6..U8 */

typedef struct {
    uint8_t bitrate;     /* 0..8 (S0..S8) */
    uint8_t mode;        /* 0 = Normal, 1 = Listen Only */
    uint8_t autostart;
    uint8_t uart_idx;    /* index into UART_BAUDS */
    char    serial[17];
} app_cfg_t;

typedef struct {
    uint8_t len;
    char    d[SL_LINE_MAX - 1];
} line_t;

/* ---------------- state ---------------- */
static app_cfg_t s_cfg = { .bitrate = 4, .mode = 0, .autostart = 0, .uart_idx = 1, .serial = "0001" };

static SemaphoreHandle_t s_can_lock;      /* protects TWAI driver install/uninstall */
static QueueHandle_t     s_uart_txq;      /* line_t   */
static QueueHandle_t     s_can_txq;       /* twai_message_t */
static TaskHandle_t      s_cfg_task;

static volatile bool        s_open;
static volatile bool        s_busoff;
static volatile bool        s_ts_on;
static volatile twai_mode_t s_mode = TWAI_MODE_NORMAL;
static volatile uint32_t    s_err;
static volatile uint32_t    s_rx_until, s_tx_until;
static volatile uint32_t    s_pending_baud;
static uint32_t             s_status;     /* accessed atomically */
static uint32_t             s_acc_code = 0, s_acc_mask = 0xFFFFFFFFu;  /* accept all */

static inline void status_set(uint32_t b) { __atomic_fetch_or(&s_status, b, __ATOMIC_RELAXED); }

/* ---------------- NVS ---------------- */
static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open("slcan", NVS_READONLY, &h) != ESP_OK) return;
    app_cfg_t c;
    size_t sz = sizeof(c);
    if (nvs_get_blob(h, "cfg", &c, &sz) == ESP_OK && sz == sizeof(c) &&
        c.bitrate <= 8 && c.mode <= 1 && c.autostart <= 1 &&
        c.uart_idx < (sizeof(UART_BAUDS) / sizeof(UART_BAUDS[0]))) {
        c.serial[16] = '\0';
        s_cfg = c;
    }   /* on any failure keep defaults (125kbps / Normal / no autostart) */
    nvs_close(h);
}

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open("slcan", NVS_READWRITE, &h) != ESP_OK) return;
    app_cfg_t c = s_cfg;
    nvs_set_blob(h, "cfg", &c, sizeof(c));
    nvs_commit(h);
    nvs_close(h);
}

static void cfg_save_async(void) { if (s_cfg_task) xTaskNotifyGive(s_cfg_task); }

static void config_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        cfg_save();
    }
}

/* ---------------- UART output ---------------- */
static bool uart_put(const char *s, size_t n, TickType_t wait)
{
    line_t l;
    if (n > sizeof(l.d)) return false;
    l.len = (uint8_t)n;
    memcpy(l.d, s, n);
    return xQueueSend(s_uart_txq, &l, wait) == pdTRUE;
}
static void reply_ok(void)  { uart_put("\r", 1, pdMS_TO_TICKS(100)); }
static void reply_err(void) { uart_put("\a", 1, pdMS_TO_TICKS(100)); }
static void reply_str(const char *s) { uart_put(s, strlen(s), pdMS_TO_TICKS(100)); }

static void uart_tx_task(void *arg)
{
    line_t l;
    for (;;) {
        if (xQueueReceive(s_uart_txq, &l, portMAX_DELAY) != pdTRUE) continue;
        uart_write_bytes(UART_PORT, l.d, l.len);
        uint32_t baud = s_pending_baud;
        if (baud) {
            uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
            uart_set_baudrate(UART_PORT, baud);
            s_pending_baud = 0;
        }
    }
}

/* ---------------- helpers ---------------- */
static const char HEXD[] = "0123456789ABCDEF";

static int put_hex(char *p, uint32_t v, int digits)
{
    for (int i = digits - 1; i >= 0; i--) *p++ = HEXD[(v >> (4 * i)) & 0xF];
    return digits;
}
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static bool hex_n(const char *s, int n, uint32_t *out)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        int h = hexval(s[i]);
        if (h < 0) return false;
        v = (v << 4) | (uint32_t)h;
    }
    *out = v;
    return true;
}

/* ---------------- TWAI control ---------------- */
static bool get_timing(int idx, twai_timing_config_t *t)
{
    switch (idx) {
#ifdef TWAI_TIMING_CONFIG_10KBITS   /* defined only when the chip supports BRP > 256 (ESP32 rev.2+) */
    case 0: { twai_timing_config_t x = TWAI_TIMING_CONFIG_10KBITS(); *t = x; return true; }
    case 1: { twai_timing_config_t x = TWAI_TIMING_CONFIG_10KBITS();
              x.brp = 200; x.tseg_1 = 15; x.tseg_2 = 4; x.sjw = 3; *t = x; return true; }  /* 20kbps */
#endif
    case 2: { twai_timing_config_t x = TWAI_TIMING_CONFIG_50KBITS();  *t = x; return true; }
    case 3: { twai_timing_config_t x = TWAI_TIMING_CONFIG_100KBITS(); *t = x; return true; }
    case 4: { twai_timing_config_t x = TWAI_TIMING_CONFIG_125KBITS(); *t = x; return true; }
    case 5: { twai_timing_config_t x = TWAI_TIMING_CONFIG_250KBITS(); *t = x; return true; }
    case 6: { twai_timing_config_t x = TWAI_TIMING_CONFIG_500KBITS(); *t = x; return true; }
    case 7: { twai_timing_config_t x = TWAI_TIMING_CONFIG_800KBITS(); *t = x; return true; }
    case 8: { twai_timing_config_t x = TWAI_TIMING_CONFIG_1MBITS();   *t = x; return true; }
    default: return false;
    }
}

static bool can_open(twai_mode_t mode)
{
    bool ok = false;
    xSemaphoreTake(s_can_lock, portMAX_DELAY);
    if (!s_open) {
        twai_timing_config_t t;
        if (get_timing(s_cfg.bitrate, &t)) {
            twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, mode);
            g.tx_queue_len    = CAN_TXQ_LEN;
            g.rx_queue_len    = CAN_RX_DRV_LEN;
            g.alerts_enabled  = ALERTS;
            twai_filter_config_t f = {
                .acceptance_code = s_acc_code,
                .acceptance_mask = s_acc_mask,
                .single_filter   = true,
            };
            if (twai_driver_install(&g, &t, &f) == ESP_OK) {
                if (twai_start() == ESP_OK) {
                    s_mode = mode;
                    s_busoff = false;
                    s_err = ERR_NONE;
                    xQueueReset(s_can_txq);
                    s_open = true;
                    ok = true;
                } else {
                    twai_driver_uninstall();
                }
            }
        }
    }
    xSemaphoreGive(s_can_lock);
    return ok;
}

static bool can_close(void)
{
    bool ok = false;
    xSemaphoreTake(s_can_lock, portMAX_DELAY);
    if (s_open) {
        twai_status_info_t si;
        if (twai_get_status_info(&si) == ESP_OK) {
            if (si.state == TWAI_STATE_RUNNING) {
                twai_stop();
            } else if (si.state == TWAI_STATE_BUS_OFF) {
                twai_initiate_recovery();
                for (int i = 0; i < 30; i++) {           /* wait up to ~300ms */
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (twai_get_status_info(&si) == ESP_OK && si.state == TWAI_STATE_STOPPED) break;
                }
            }
        }
        if (twai_driver_uninstall() == ESP_OK) {
            s_open = false;
            s_busoff = false;
            s_err = ERR_NONE;
            xQueueReset(s_can_txq);
            ok = true;
        }
    }
    xSemaphoreGive(s_can_lock);
    return ok;
}

/* ---------------- CAN -> UART ---------------- */
static void forward_frame(const twai_message_t *m)
{
    line_t l;
    char *p = l.d;
    *p++ = m->extd ? (m->rtr ? 'R' : 'T') : (m->rtr ? 'r' : 't');
    p += put_hex(p, m->identifier, m->extd ? 8 : 3);
    *p++ = (char)('0' + (m->data_length_code > 8 ? 8 : m->data_length_code));
    if (!m->rtr) {
        for (int i = 0; i < m->data_length_code && i < 8; i++) p += put_hex(p, m->data[i], 2);
    }
    if (s_ts_on) p += put_hex(p, (uint32_t)(esp_timer_get_time() / 1000) & 0xFFFF, 4);
    *p++ = '\r';
    l.len = (uint8_t)(p - l.d);
    if (xQueueSend(s_uart_txq, &l, 0) != pdTRUE) status_set(ST_RXFULL);   /* UART side full: drop */
    s_rx_until = xTaskGetTickCount() + pdMS_TO_TICKS(LED_PULSE_MS);
}

static void handle_alerts(uint32_t al)
{
    if (!al) return;
    if (al & TWAI_ALERT_ABOVE_ERR_WARN) { status_set(ST_WARN); if (s_err < ERR_WARN) s_err = ERR_WARN; }
    if (al & TWAI_ALERT_ERR_PASS)       { status_set(ST_PASSIVE); if (s_err < ERR_BUSOFF) s_err = ERR_PASSIVE; }
    if (al & TWAI_ALERT_ERR_ACTIVE)     { if (s_err < ERR_BUSOFF) s_err = ERR_WARN; }
    if (al & TWAI_ALERT_BELOW_ERR_WARN) { if (s_err < ERR_BUSOFF) s_err = ERR_NONE; }
    if (al & TWAI_ALERT_BUS_OFF)        { s_err = ERR_BUSOFF; s_busoff = true; }   /* no auto recovery: C then O/L */
    if (al & TWAI_ALERT_TX_SUCCESS)     { if (s_mode == TWAI_MODE_NORMAL) s_tx_until = xTaskGetTickCount() + pdMS_TO_TICKS(LED_PULSE_MS); }
    if (al & TWAI_ALERT_RX_QUEUE_FULL)  status_set(ST_RXFULL);
    if (al & TWAI_ALERT_RX_FIFO_OVERRUN) status_set(ST_OVERRUN);
    if (al & TWAI_ALERT_ARB_LOST)       status_set(ST_ARBLOST);
    if (al & TWAI_ALERT_BUS_ERROR)      status_set(ST_BUSERR);
}

static void can_task(void *arg)
{
    for (;;) {
        if (!s_open) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        xSemaphoreTake(s_can_lock, portMAX_DELAY);
        if (s_open) {
            twai_message_t m;
            /* UART -> CAN */
            if (!s_busoff && s_mode == TWAI_MODE_NORMAL) {
                while (xQueueReceive(s_can_txq, &m, 0) == pdTRUE) {
                    if (twai_transmit(&m, 0) != ESP_OK) status_set(ST_TXFULL);
                }
            } else {
                xQueueReset(s_can_txq);
            }
            /* alerts (also acts as a <=2ms wait, woken early by RX_DATA) */
            uint32_t al = 0;
            twai_read_alerts(&al, pdMS_TO_TICKS(2));
            handle_alerts(al);
            /* CAN -> UART */
            for (int i = 0; i < 64 && twai_receive(&m, 0) == ESP_OK; i++) forward_frame(&m);
        }
        xSemaphoreGive(s_can_lock);
    }
}

/* ---------------- SLCAN command parser ---------------- */
static void cmd_transmit(const char *l, size_t n)
{
    char c = l[0];
    bool ext = (c == 'T' || c == 'R');
    bool rtr = (c == 'r' || c == 'R');
    int idlen = ext ? 8 : 3;

    if (!s_open || s_busoff || s_mode != TWAI_MODE_NORMAL) { reply_err(); return; }
    if (n < (size_t)(1 + idlen + 1)) { reply_err(); return; }

    uint32_t id;
    if (!hex_n(l + 1, idlen, &id) || id > (ext ? 0x1FFFFFFFu : 0x7FFu)) { reply_err(); return; }
    int dlc = l[1 + idlen] - '0';
    if (dlc < 0 || dlc > 8) { reply_err(); return; }
    if (n != (size_t)(1 + idlen + 1 + (rtr ? 0 : dlc * 2))) { reply_err(); return; }

    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id;
    m.extd = ext;
    m.rtr = rtr;
    m.data_length_code = (uint8_t)dlc;
    if (!rtr) {
        for (int i = 0; i < dlc; i++) {
            uint32_t b;
            if (!hex_n(l + 2 + idlen + i * 2, 2, &b)) { reply_err(); return; }
            m.data[i] = (uint8_t)b;
        }
    }
    if (xQueueSend(s_can_txq, &m, 0) != pdTRUE) { status_set(ST_TXFULL); reply_err(); return; }
    reply_str(ext ? "Z\r" : "z\r");
}

static void handle_cmd(const char *l, size_t n)
{
    twai_timing_config_t t;
    uint32_t v;
    char buf[SL_LINE_MAX - 1];

    switch (l[0]) {
    case 'S':
        if (n == 2 && l[1] >= '0' && l[1] <= '8' && !s_open && get_timing(l[1] - '0', &t)) {
            s_cfg.bitrate = (uint8_t)(l[1] - '0');
            reply_ok();
        } else reply_err();
        break;
    case 'O':
    case 'L':
        if (n == 1 && can_open(l[0] == 'O' ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY)) {
            s_cfg.mode = (l[0] == 'L');
            reply_ok();
        } else reply_err();
        break;
    case 'C':
        if (n == 1 && can_close()) reply_ok(); else reply_err();
        break;
    case 't': case 'T': case 'r': case 'R':
        cmd_transmit(l, n);
        break;
    case 'F':
        if (n == 1 && s_open) {
            uint32_t f = __atomic_exchange_n(&s_status, 0, __ATOMIC_RELAXED);
            buf[0] = 'F';
            put_hex(buf + 1, f & 0xFF, 2);
            buf[3] = '\r';
            uart_put(buf, 4, pdMS_TO_TICKS(100));
        } else reply_err();
        break;
    case 'Z':
        if (n == 2 && (l[1] == '0' || l[1] == '1')) { s_ts_on = (l[1] == '1'); reply_ok(); } else reply_err();
        break;
    case 'V':
        if (n == 1) reply_str("V" HW_VER "\r"); else reply_err();
        break;
    case 'v':
        if (n == 1) reply_str("v" FW_VER "\r"); else reply_err();
        break;
    case 'N':
        if (n == 1) {
            int k = snprintf(buf, sizeof(buf), "N%s\r", s_cfg.serial);
            uart_put(buf, (size_t)k, pdMS_TO_TICKS(100));
        } else reply_err();
        break;
    case 'M':
    case 'm':
        if (n == 9 && !s_open && hex_n(l + 1, 8, &v)) {
            if (l[0] == 'M') s_acc_code = v; else s_acc_mask = v;
            reply_ok();
        } else reply_err();
        break;
    /* ---- extensions ---- */
    case 'U':
        if (n == 2 && l[1] >= '6' && l[1] <= '8') {
            s_cfg.uart_idx = (uint8_t)(l[1] - '6');
            reply_ok();                                  /* reply first, then switch baud */
            s_pending_baud = UART_BAUDS[s_cfg.uart_idx];
            cfg_save_async();
        } else reply_err();
        break;
    case 'A':
        if (n == 2 && (l[1] == '0' || l[1] == '1')) { s_cfg.autostart = (uint8_t)(l[1] - '0'); reply_ok(); } else reply_err();
        break;
    case 'W':
        if (n == 1) { cfg_save_async(); reply_ok(); } else reply_err();
        break;
    case 'n':
        if (n >= 2 && n - 1 <= 16) {
            for (size_t i = 1; i < n; i++) if (l[i] < 0x21 || l[i] > 0x7E) { reply_err(); return; }
            memset(s_cfg.serial, 0, sizeof(s_cfg.serial));
            memcpy(s_cfg.serial, l + 1, n - 1);
            reply_ok();
        } else reply_err();
        break;
    default:
        reply_err();
        break;
    }
}

static void uart_rx_task(void *arg)
{
    char line[SL_LINE_MAX];
    size_t n = 0;
    bool over = false;
    uint8_t buf[64];

    for (;;) {
        int r = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(10));
        for (int i = 0; i < r; i++) {
            char c = (char)buf[i];
#if SL_DEBUG
            uart_write_bytes(UART_PORT, &c, 1);   /* 受信した文字をそのまま返す */
#endif
            if (c == '\r') {
                if (over) reply_err();               /* overlong line: discarded, resynced at CR */
                else if (n == 0) reply_ok();
                else handle_cmd(line, n);
                n = 0;
                over = false;
            } else if (c == '\n') {
                /* ignore LF */
            } else if (n < sizeof(line)) {
                line[n++] = c;
            } else {
                over = true;
            }
        }
    }
}

/* ---------------- LED ---------------- */
static void led_task(void *arg)
{
    for (;;) {
        uint32_t now = xTaskGetTickCount();
        uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool open = s_open;

        gpio_set_level(PIN_LED_RX, open && (int32_t)(s_rx_until - now) > 0);
        gpio_set_level(PIN_LED_TX, open && s_mode == TWAI_MODE_NORMAL && (int32_t)(s_tx_until - now) > 0);

        int lvl = 0;
        if (open) {
            switch (s_err) {
            case ERR_BUSOFF:   lvl = 1; break;
            case ERR_PASSIVE:  lvl = ((ms / 125) & 1) == 0; break;   /* 4Hz */
            case ERR_WARN:     lvl = ((ms / 500) & 1) == 0; break;   /* 1Hz */
            default: break;
            }
        }
        gpio_set_level(PIN_LED_ERR, lvl);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ---------------- init ---------------- */
static void gpio_init_leds(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED_RX) | (1ULL << PIN_LED_TX) | (1ULL << PIN_LED_ERR),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LED_RX, 0);
    gpio_set_level(PIN_LED_TX, 0);
    gpio_set_level(PIN_LED_ERR, 0);
}

static void uart_init(uint32_t baud)
{
    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
//    esp_log_level_set("*", ESP_LOG_NONE);            /* UART0 is the SLCAN channel */

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    cfg_load();

    gpio_init_leds();
    uart_init(UART_BAUDS[s_cfg.uart_idx]);
//    uart_init(460800);
//    uart_init(921600);
//    uart_init(2000000);

    s_can_lock = xSemaphoreCreateMutex();
    s_uart_txq = xQueueCreate(UART_TXQ_LEN, sizeof(line_t));
    s_can_txq  = xQueueCreate(CAN_TXQ_LEN, sizeof(twai_message_t));
    configASSERT(s_can_lock && s_uart_txq && s_can_txq);

    xTaskCreate(config_task,  "config_task",  4096, NULL, 2, &s_cfg_task);
    xTaskCreate(led_task,     "led_task",     2048, NULL, 3, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 3072, NULL, 5, NULL);
    xTaskCreate(can_task,     "can_task",     4096, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 6, NULL);   /* highest: wins can_lock */

    if (s_cfg.autostart) {
        can_open(s_cfg.mode ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    }

#if SL_DEBUG
    uart_write_bytes(UART_PORT, "SLCAN READY\r\n", 13);
#endif

}
