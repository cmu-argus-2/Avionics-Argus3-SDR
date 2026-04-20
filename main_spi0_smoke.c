#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include <eff/drivers/uart.h>

#define UART_DEV       UART_4
#define UART_PINMUX    PINMUX_4

#define SPI_DEV        SPI_0
#define SPI_PINMUX     PINMUX_0
#define SPI_PINMUX_CFG PINMUX_SPI

#define FRAME_MAGIC       0x49515130u
#define FRAME_VERSION     1u
#define FRAME_DATA_BYTES  2048u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;

static int uart_init_debug(void)
{
    eff_uart_cfg_t cfg = EFF_UART_DEFAULTS;
    if (eff_uart_init(UART_DEV, cfg)) return -1;
    if (eff_pinmux_set(UART_PINMUX, PINMUX_UART)) return -1;
    return 0;
}

static void uart_log(const char *s)
{
    eff_uart_puts(UART_DEV, s);
}

static void uart_log_hex32(const char *prefix, uint32_t x)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "%s0x%08lx\r\n", prefix, (unsigned long)x);
    eff_uart_puts(UART_DEV, buf);
}

static void uart_log_u32_2(const char *prefix, uint32_t a, uint32_t b)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "%s%lu %lu\r\n", prefix,
             (unsigned long)a, (unsigned long)b);
    eff_uart_puts(UART_DEV, buf);
}

static int spi0_init(void)
{
    eff_spi_cfg_t spi_cfg = EFF_SPI_DEFAULTS;
    spi_cfg.xfer_mode = SPI_XFER_READ_ONLY;
    spi_cfg.bus_size = SPI_BUS_SINGLE;

    if (eff_spi_init(SPI_DEV, &spi_cfg)) return -1;
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) return -1;
    return 0;
}

static int spi0_read_frame(iq_spi_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    if (eff_spi_xfer(SPI_DEV, 0, 0, NULL, 0, (uint8_t *)frame, (uint32_t)sizeof(*frame))) {
        return 0;
    }
    return 1;
}

int main(void)
{
    iq_spi_frame_t frame;

    if (uart_init_debug()) {
        return -1;
    }

    uart_log("[BOOT] spi0 smoke start\r\n");

    if (spi0_init()) {
        uart_log("[ERR] spi0 init failed\r\n");
        return -1;
    }

    uart_log("[BOOT] spi0 init ok, waiting for frames\r\n");

    while (1) {
        if (!spi0_read_frame(&frame)) {
            uart_log("[ERR] spi0 read failed\r\n");
            continue;
        }

        uart_log_hex32("[HDR] magic=", frame.magic);
        uart_log_u32_2("[HDR] ver bytes=", frame.version, frame.payload_bytes);
        uart_log_u32_2("[HDR] seq fs=", frame.sequence, frame.sample_rate_hz);
        uart_log_u32_2("[HDR] cfreq i0=", frame.center_freq_hz, frame.iq_u8[0]);

        if (frame.magic != FRAME_MAGIC) {
            uart_log("[WARN] bad magic\r\n");
        }
        if (frame.version != FRAME_VERSION) {
            uart_log("[WARN] bad version\r\n");
        }
        if (frame.payload_bytes != FRAME_DATA_BYTES) {
            uart_log("[WARN] bad payload_bytes\r\n");
        }
    }

    return 0;
}
