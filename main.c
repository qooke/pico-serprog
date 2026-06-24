/**
 * Copyright (C) 2021, Mate Kukri <km@mkukri.xyz>
 * Based on "pico-serprog" by Thomas Roth <code@stacksmashing.net>
 * 
 * Licensed under GPLv3
 *
 * Also based on stm32-vserprog:
 *  https://github.com/dword1511/stm32-vserprog
 * 
 */

#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "tusb.h"
#include "serprog.h"

#if defined(PICO_CYW43_SUPPORTED) && PICO_CYW43_SUPPORTED
#include "pico/cyw43_arch.h"
#endif

#define CDC_ITF     0           // USB CDC interface no

#define SPI_IF      spi0        // Which PL022 to use
#define SPI_BAUD    10000000    // Default baudrate (10 MHz)
#define SPI_CS      5
#define SPI_MISO    4
#define SPI_MOSI    3
#define SPI_SCK     2
#define MAX_BUFFER_SIZE 256
#define MAX_OPBUF_SIZE 64
#define SERIAL_BUFFER_SIZE 64
#define LED_ACTIVITY_WINDOW_MS 200
#define LED_ACTIVITY_HOLD_MS 700
#define LED_BLINK_SLOW_MS 450
#define LED_BLINK_FAST_MS 45
#define LED_ACTIVITY_MAX_BPS 262144

// Define a global operation buffer and a pointer to track the current position
uint8_t opbuf[MAX_OPBUF_SIZE];
uint32_t opbuf_pos = 0;

static bool led_ready = false;
static bool led_on = true;
static uint64_t led_window_bytes = 0;
static uint32_t led_blink_interval_ms = LED_BLINK_SLOW_MS;
static absolute_time_t led_activity_until;
static absolute_time_t led_window_started_at;
static absolute_time_t led_next_toggle;

static inline void led_put(bool on)
{
#if defined(PICO_CYW43_SUPPORTED) && PICO_CYW43_SUPPORTED
    if (led_ready)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#elif defined(PICO_DEFAULT_LED_PIN)
    if (led_ready)
        gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
    (void) on;
#endif
}

static inline void led_set(bool on)
{
    if (led_on == on)
        return;

    led_on = on;
    led_put(on);
}

static void led_init(void)
{
#if defined(PICO_CYW43_SUPPORTED) && PICO_CYW43_SUPPORTED
    led_ready = cyw43_arch_init() == 0;
#elif defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    led_ready = true;
#endif

    led_activity_until = get_absolute_time();
    led_window_started_at = get_absolute_time();
    led_next_toggle = make_timeout_time_ms(LED_BLINK_SLOW_MS);

    if (led_ready) {
        led_on = false;
        led_set(true);
    }
}

static inline void led_note_activity(uint32_t bytes)
{
    led_window_bytes += bytes ? bytes : 1;
    led_activity_until = make_timeout_time_ms(LED_ACTIVITY_HOLD_MS);
}

static inline void led_task(void)
{
    if (!led_ready)
        return;

    absolute_time_t now = get_absolute_time();
    int64_t window_us = absolute_time_diff_us(led_window_started_at, now);

    if (window_us >= LED_ACTIVITY_WINDOW_MS * 1000) {
        uint32_t elapsed_ms = window_us / 1000;

        if (led_window_bytes > 0 && elapsed_ms > 0) {
            uint64_t bps = (led_window_bytes * 1000) / elapsed_ms;
            uint64_t capped_bps = bps > LED_ACTIVITY_MAX_BPS ? LED_ACTIVITY_MAX_BPS : bps;
            uint32_t interval_range = LED_BLINK_SLOW_MS - LED_BLINK_FAST_MS;

            led_blink_interval_ms = LED_BLINK_SLOW_MS -
                ((uint32_t)capped_bps * interval_range) / LED_ACTIVITY_MAX_BPS;
        } else if (absolute_time_diff_us(now, led_activity_until) <= 0) {
            led_blink_interval_ms = LED_BLINK_SLOW_MS;
        }

        led_window_bytes = 0;
        led_window_started_at = now;
    }

    if (absolute_time_diff_us(now, led_activity_until) <= 0) {
        led_set(true);
        led_next_toggle = make_timeout_time_ms(led_blink_interval_ms);
        return;
    }

    if (absolute_time_diff_us(now, led_next_toggle) <= 0) {
        led_set(!led_on);
        led_next_toggle = make_timeout_time_ms(led_blink_interval_ms);
    }
}

static void enable_spi(uint baud)
{
    // Setup chip select GPIO
    gpio_init(SPI_CS);
    gpio_put(SPI_CS, 1);
    gpio_set_dir(SPI_CS, GPIO_OUT);

    // Setup PL022
    spi_init(SPI_IF, baud);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK,  GPIO_FUNC_SPI);
}

static void disable_spi()
{
    // Set all pins to SIO inputs
    gpio_init(SPI_CS);
    gpio_init(SPI_MISO);
    gpio_init(SPI_MOSI);
    gpio_init(SPI_SCK);

    // Disable all pulls
    gpio_set_pulls(SPI_CS, 0, 0);
    gpio_set_pulls(SPI_MISO, 0, 0);
    gpio_set_pulls(SPI_MOSI, 0, 0);
    gpio_set_pulls(SPI_SCK, 0, 0);

    // Disable SPI peripheral
    spi_deinit(SPI_IF);
}

static inline void cs_select(uint cs_pin) {
    sleep_us(1); // 1 microsecond delay; adjust as needed
    gpio_put(cs_pin, 0);
    sleep_us(1); // Additional delay after CS is pulled low
}

static inline void cs_deselect(uint cs_pin) {
    sleep_us(1); // Delay before pulling CS high
    gpio_put(cs_pin, 1);
    sleep_us(1); // Additional delay after CS is pulled high
}

static void wait_for_read(void)
{
    do {
        tud_task();
        led_task();
    } while (!tud_cdc_n_available(CDC_ITF));
}

static inline void readbytes_blocking(void *b, uint32_t len)
{
    uint8_t *buf = b;

    while (len) {
        wait_for_read();
        uint32_t r = tud_cdc_n_read(CDC_ITF, buf, len);
        if (r)
            led_note_activity(r);
        buf += r;
        len -= r;
    }
}

static inline uint8_t readbyte_blocking(void)
{
    wait_for_read();
    uint8_t b;
    uint32_t r = tud_cdc_n_read(CDC_ITF, &b, 1);
    if (r)
        led_note_activity(r);
    return b;
}

static void wait_for_write(void)
{
    do {
        tud_task();
        led_task();
    } while (!tud_cdc_n_write_available(CDC_ITF));
}

static inline void sendbytes_blocking(const void *b, uint32_t len)
{
    const uint8_t *buf = b;

    while (len) {
        wait_for_write();
        uint32_t w = tud_cdc_n_write(CDC_ITF, buf, len);
        if (w)
            led_note_activity(w);
        buf += w;
        len -= w;
    }
}

static inline void sendbyte_blocking(uint8_t b)
{
    wait_for_write();
    uint32_t w = tud_cdc_n_write(CDC_ITF, &b, 1);
    if (w)
        led_note_activity(w);
}

static void command_loop(void)
{
    uint baud = spi_get_baudrate(SPI_IF);

    for (;;) {
        switch (readbyte_blocking()) {
        case S_CMD_NOP:
            sendbyte_blocking(S_ACK);
            break;
        case S_CMD_Q_IFACE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking(0x01);
            sendbyte_blocking(0x00);
            break;
        case S_CMD_Q_RDNMAXLEN:
        case S_CMD_Q_WRNMAXLEN:
            {
                sendbyte_blocking(S_ACK);

                // Break down MAX_BUFFER_SIZE into three bytes (24 bits) in little-endian format
                sendbyte_blocking(32 & 0xFF);         // LSB
                sendbyte_blocking((32 >> 8) & 0xFF);  // Middle byte
                sendbyte_blocking((32 >> 16) & 0xFF); // MSB

                break;
            }
        case S_CMD_Q_CMDMAP:
            {
                static const uint32_t cmdmap[8] = {
                      (1 << S_CMD_NOP)       |
                      (1 << S_CMD_Q_IFACE)   |
                      (1 << S_CMD_Q_RDNMAXLEN)   |
                      (1 << S_CMD_Q_WRNMAXLEN)   |
                      (1 << S_CMD_Q_CMDMAP)  |
                      (1 << S_CMD_Q_PGMNAME) |
                      (1 << S_CMD_Q_SERBUF)  |
                      (1 << S_CMD_Q_BUSTYPE) |
                      (1 << S_CMD_SYNCNOP)   |
                      (1 << S_CMD_O_SPIOP)   |
                      (1 << S_CMD_S_BUSTYPE) |
                      (1 << S_CMD_S_SPI_FREQ)|
                      (1 << S_CMD_R_BYTE)|
                      (1 << S_CMD_O_WRITEB)|
                      (1 << S_CMD_O_INIT)|
                      (1 << S_CMD_O_EXEC)
                };

                sendbyte_blocking(S_ACK);
                sendbytes_blocking((uint8_t *) cmdmap, sizeof cmdmap);
                break;
            }
        case S_CMD_Q_PGMNAME:
            {
                static const char progname[16] = "pico-serprog";

                sendbyte_blocking(S_ACK);
                sendbytes_blocking(progname, sizeof progname);
                break;
            }
        case S_CMD_Q_SERBUF:
            {
                sendbyte_blocking(S_ACK);

                // Send the buffer size as a 16-bit little-endian value
                uint16_t bufferSizeLE = SERIAL_BUFFER_SIZE & 0xFFFF;
                sendbyte_blocking((uint8_t)(bufferSizeLE & 0xFF));        // Lower byte
                sendbyte_blocking((uint8_t)((bufferSizeLE >> 8) & 0xFF)); // Upper byte

                break;
            }
        case S_CMD_Q_BUSTYPE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking((1 << 3)); // BUS_SPI
            break;
        case S_CMD_SYNCNOP:
            sendbyte_blocking(S_NAK);
            sendbyte_blocking(S_ACK);
            break;
        case S_CMD_S_BUSTYPE:
            // If SPI is among the requsted bus types we succeed, fail otherwise
            if((uint8_t) readbyte_blocking() & (1 << 3))
                sendbyte_blocking(S_ACK);
            else
                sendbyte_blocking(S_NAK);
            break;
        case S_CMD_O_SPIOP:
            {
                uint32_t slen, rlen;
                readbytes_blocking(&slen, 3); // Read send length
                readbytes_blocking(&rlen, 3); // Read receive length
                slen &= 0x00FFFFFF; // Mask to use only the lower 24 bits
                rlen &= 0x00FFFFFF; // Mask to use only the lower 24 bits

                uint8_t tx_buffer[MAX_BUFFER_SIZE]; // Buffer for transmit data
                uint8_t rx_buffer[MAX_BUFFER_SIZE]; // Buffer for receive data

                // Read data to be sent (if slen > 0)
                if (slen > 0) {
                    readbytes_blocking(tx_buffer, slen);
                }

                // Perform SPI operation
                cs_select(SPI_CS);
                if (slen > 0) {
                    spi_write_blocking(SPI_IF, tx_buffer, slen);
                }
                if (rlen > 0 && rlen < MAX_BUFFER_SIZE ) {
                    spi_read_blocking(SPI_IF, 0, rx_buffer, rlen);
                    // Send ACK followed by received data
                    sendbyte_blocking(S_ACK);
                    if (rlen > 0) {
                        sendbytes_blocking(rx_buffer, rlen);
                    }

                    cs_deselect(SPI_CS);
                    break;
                }

                // Send ACK after handling slen (before reading)
                sendbyte_blocking(S_ACK);

                // Handle receive operation in chunks for large rlen
                uint32_t chunk;
                char buf[128];

                for(uint32_t i = 0; i < rlen; i += chunk) {
                    chunk = MIN(rlen - i, sizeof(buf));
                    spi_read_blocking(SPI_IF, 0, buf, chunk);
                    // Send ACK followed by received data
                    sendbyte_blocking(S_ACK);
                    sendbytes_blocking(buf, rlen);
                }
                cs_deselect(SPI_CS);
                break;
            }
            case S_CMD_S_SPI_FREQ:
            {
                uint32_t want_baud;
                readbytes_blocking(&want_baud, 4);
                if (want_baud) {
                    // Set frequence
                    baud = spi_set_baudrate(SPI_IF, want_baud);
                    // Send back actual value
                    sendbyte_blocking(S_ACK);
                    sendbytes_blocking(&baud, 4);
                } else {
                    // 0 Hz is reserved
                    sendbyte_blocking(S_NAK);
                }
                break;
            }
        case S_CMD_R_BYTE:
            {
                uint32_t addr;
                readbytes_blocking(&addr, 3);
                uint8_t data;

                cs_select(SPI_CS);
                spi_write_blocking(SPI_IF, (uint8_t*)&addr, 3); // Send address
                spi_read_blocking(SPI_IF, 0, &data, 1); // Read one byte
                cs_deselect(SPI_CS);

                sendbyte_blocking(S_ACK);
                sendbyte_blocking(data);
                break;
            }
        case S_CMD_R_NBYTES:
            {
                uint32_t addr, len;
                readbytes_blocking(&addr, 3);
                readbytes_blocking(&len, 3);

                uint8_t buffer[MAX_BUFFER_SIZE]; // Define MAX_BUFFER_SIZE based on your hardware capability

                cs_select(SPI_CS);
                spi_write_blocking(SPI_IF, (uint8_t*)&addr, 3); // Send address

                while (len > 0) {
                    uint32_t chunk_size = (len < MAX_BUFFER_SIZE) ? len : MAX_BUFFER_SIZE;
                    spi_read_blocking(SPI_IF, 0, buffer, chunk_size);
                    sendbytes_blocking(buffer, chunk_size);
                    len -= chunk_size;
                }

                cs_deselect(SPI_CS);

                sendbyte_blocking(S_ACK);
                break;
            }
        case S_CMD_O_WRITEB:
            {
                if (opbuf_pos + 5 > MAX_OPBUF_SIZE) {
                    sendbyte_blocking(S_NAK);
                    break;
                }

                uint32_t addr;
                uint8_t byte;
                readbytes_blocking(&addr, 3);
                byte = readbyte_blocking();

                // Store in operation buffer (assuming format: 1-byte command, 3-byte address, 1-byte data)
                opbuf[opbuf_pos++] = S_CMD_O_WRITEB;
                memcpy(&opbuf[opbuf_pos], &addr, 3);
                opbuf_pos += 3;
                opbuf[opbuf_pos++] = byte;

                sendbyte_blocking(S_ACK);
                break;
            }
        case S_CMD_O_INIT:
            {
                opbuf_pos = 0; // Reset the operation buffer position
                memset(opbuf, 0, MAX_OPBUF_SIZE); // Clear the buffer (optional)
                sendbyte_blocking(S_ACK);
                break;
            }
        case S_CMD_O_EXEC:
            {
                if (opbuf_pos == 0) {
                    sendbyte_blocking(S_NAK);
                    break;
                }

                // Send ACK before handling the operation buffer
                sendbyte_blocking(S_ACK);

                // Handle the operation buffer
                uint32_t i = 0;
                while (i < opbuf_pos) {
                    uint8_t cmd = opbuf[i++];
                    uint32_t addr;
                    uint8_t byte;

                    switch (cmd) {
                    case S_CMD_O_WRITEB:
                        memcpy(&addr, &opbuf[i], 3);
                        i += 3;
                        byte = opbuf[i++];
                        cs_select(SPI_CS);
                        spi_write_blocking(SPI_IF, (uint8_t*)&addr, 3); // Send address
                        spi_write_blocking(SPI_IF, &byte, 1); // Send data
                        cs_deselect(SPI_CS);
                        break;
                    default:
                        sendbyte_blocking(S_NAK);
                        break;
                    }
                }

                // Send ACK after handling the operation buffer
                sendbyte_blocking(S_ACK);
                break;
            }
        default:
            sendbyte_blocking(S_NAK);
            break;
        }

        tud_cdc_n_write_flush(CDC_ITF);
    }
}

int main()
{
    led_init();
    // Setup USB
    tusb_init();
    // Setup PL022 SPI
    enable_spi(SPI_BAUD);

    command_loop();
}
