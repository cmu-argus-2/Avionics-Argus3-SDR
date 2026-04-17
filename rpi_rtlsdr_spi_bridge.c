#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <rtl-sdr.h>

#define DEFAULT_SPI_DEV        "/dev/spidev0.0"
#define DEFAULT_CENTER_HZ      1575420000u  /* GPS L1 */
#define DEFAULT_SAMPLE_RATE_HZ 1024000u     /* 1.024 Msps => 1 ms = 1024 IQ samples */
#define DEFAULT_GAIN_TENTH_DB  0            /* 0 => auto gain */
#define DEFAULT_SPI_HZ         20000000u
#define RTL_ASYNC_BUF_NUM      12
#define RTL_ASYNC_BUF_LEN      (16 * 32 * 512)   /* librtlsdr default-friendly */

#define FRAME_MAGIC            0x49515130u  /* 'IQQ0' */
#define FRAME_VERSION          1u
#define FRAME_DATA_BYTES       2048u        /* 1024 complex U8 IQ samples */
#define FRAME_TOTAL_BYTES      (sizeof(iq_spi_frame_t))
#define FRAME_QUEUE_DEPTH      128u

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;
#pragma pack(pop)

typedef struct {
    iq_spi_frame_t frames[FRAME_QUEUE_DEPTH];
    uint32_t wr;
    uint32_t rd;
    uint32_t count;
    uint32_t drops;
    uint32_t partial_fill;
    iq_spi_frame_t partial;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    bool stop;
} frame_queue_t;

typedef struct {
    frame_queue_t *q;
    int spi_fd;
} spi_worker_arg_t;

static volatile sig_atomic_t g_stop = 0;
static rtlsdr_dev_t *g_dev = NULL;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
    if (g_dev) {
        rtlsdr_cancel_async(g_dev);
    }
}

static void queue_init(frame_queue_t *q, uint32_t fs_hz, uint32_t fc_hz) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->cv, NULL);

    q->partial.magic = FRAME_MAGIC;
    q->partial.version = FRAME_VERSION;
    q->partial.payload_bytes = FRAME_DATA_BYTES;
    q->partial.sequence = 0;
    q->partial.sample_rate_hz = fs_hz;
    q->partial.center_freq_hz = fc_hz;
}

static void queue_destroy(frame_queue_t *q) {
    pthread_mutex_destroy(&q->mu);
    pthread_cond_destroy(&q->cv);
}

static bool queue_push_locked(frame_queue_t *q, const iq_spi_frame_t *f) {
    if (q->count == FRAME_QUEUE_DEPTH) {
        q->drops++;
        return false;
    }
    q->frames[q->wr] = *f;
    q->wr = (q->wr + 1u) % FRAME_QUEUE_DEPTH;
    q->count++;
    pthread_cond_signal(&q->cv);
    return true;
}

static bool queue_pop(frame_queue_t *q, iq_spi_frame_t *out) {
    pthread_mutex_lock(&q->mu);
    while (q->count == 0 && !q->stop) {
        pthread_cond_wait(&q->cv, &q->mu);
    }
    if (q->count == 0 && q->stop) {
        pthread_mutex_unlock(&q->mu);
        return false;
    }
    *out = q->frames[q->rd];
    q->rd = (q->rd + 1u) % FRAME_QUEUE_DEPTH;
    q->count--;
    pthread_mutex_unlock(&q->mu);
    return true;
}

static void queue_finish(frame_queue_t *q) {
    pthread_mutex_lock(&q->mu);
    q->stop = true;
    pthread_cond_broadcast(&q->cv);
    pthread_mutex_unlock(&q->mu);
}

static void enqueue_bytes(frame_queue_t *q, const uint8_t *buf, uint32_t len) {
    pthread_mutex_lock(&q->mu);

    while (len > 0) {
        uint32_t room = FRAME_DATA_BYTES - q->partial_fill;
        uint32_t take = (len < room) ? len : room;

        memcpy(&q->partial.iq_u8[q->partial_fill], buf, take);
        q->partial_fill += take;
        buf += take;
        len -= take;

        if (q->partial_fill == FRAME_DATA_BYTES) {
            iq_spi_frame_t ready = q->partial;
            ready.sequence = q->partial.sequence;
            (void)queue_push_locked(q, &ready);

            q->partial.sequence++;
            q->partial_fill = 0;
        }
    }

    pthread_mutex_unlock(&q->mu);
}

static void rtlsdr_cb(unsigned char *buf, uint32_t len, void *ctx) {
    frame_queue_t *q = (frame_queue_t *)ctx;
    if (g_stop) {
        return;
    }
    enqueue_bytes(q, buf, len);
}

static int write_all(int fd, const uint8_t *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        off += (size_t)n;
    }
    return 0;
}

static void *spi_worker(void *arg) {
    spi_worker_arg_t *wa = (spi_worker_arg_t *)arg;
    iq_spi_frame_t frame;

    while (queue_pop(wa->q, &frame)) {
        if (write_all(wa->spi_fd, (const uint8_t *)&frame, sizeof(frame)) != 0) {
            perror("SPI write failed");
            g_stop = 1;
            if (g_dev) {
                rtlsdr_cancel_async(g_dev);
            }
            break;
        }
    }
    return NULL;
}

static int spi_open_configure(const char *path, uint32_t speed_hz) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("open spi");
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("SPI_IOC_WR_MODE");
        close(fd);
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        close(fd);
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        close(fd);
        return -1;
    }

    return fd;
}

static int rtlsdr_open_configure(rtlsdr_dev_t **out,
                                 uint32_t center_hz,
                                 uint32_t sample_rate_hz,
                                 int gain_tenth_db)
{
    rtlsdr_dev_t *dev = NULL;
    int r;

    r = rtlsdr_open(&dev, 0);
    if (r != 0) {
        fprintf(stderr, "rtlsdr_open failed: %d\n", r);
        return -1;
    }

    if (rtlsdr_set_tuner_gain_mode(dev, gain_tenth_db == 0 ? 0 : 1) != 0) {
        fprintf(stderr, "warning: set_tuner_gain_mode failed\n");
    }
    if (gain_tenth_db != 0 && rtlsdr_set_tuner_gain(dev, gain_tenth_db) != 0) {
        fprintf(stderr, "warning: set_tuner_gain failed\n");
    }
    if (rtlsdr_set_sample_rate(dev, sample_rate_hz) != 0) {
        fprintf(stderr, "rtlsdr_set_sample_rate failed\n");
        rtlsdr_close(dev);
        return -1;
    }
    if (rtlsdr_set_center_freq(dev, center_hz) != 0) {
        fprintf(stderr, "rtlsdr_set_center_freq failed\n");
        rtlsdr_close(dev);
        return -1;
    }
    if (rtlsdr_reset_buffer(dev) != 0) {
        fprintf(stderr, "rtlsdr_reset_buffer failed\n");
        rtlsdr_close(dev);
        return -1;
    }

    *out = dev;
    return 0;
}

int main(int argc, char **argv) {
    const char *spi_path = DEFAULT_SPI_DEV;
    uint32_t center_hz = DEFAULT_CENTER_HZ;
    uint32_t sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ;
    uint32_t spi_hz = DEFAULT_SPI_HZ;
    int gain_tenth_db = DEFAULT_GAIN_TENTH_DB;

    if (argc > 1) spi_path = argv[1];
    if (argc > 2) center_hz = (uint32_t)strtoul(argv[2], NULL, 0);
    if (argc > 3) sample_rate_hz = (uint32_t)strtoul(argv[3], NULL, 0);
    if (argc > 4) spi_hz = (uint32_t)strtoul(argv[4], NULL, 0);
    if (argc > 5) gain_tenth_db = (int)strtol(argv[5], NULL, 0);

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    frame_queue_t queue;
    queue_init(&queue, sample_rate_hz, center_hz);

    int spi_fd = spi_open_configure(spi_path, spi_hz);
    if (spi_fd < 0) {
        queue_destroy(&queue);
        return 1;
    }

    if (rtlsdr_open_configure(&g_dev, center_hz, sample_rate_hz, gain_tenth_db) != 0) {
        close(spi_fd);
        queue_destroy(&queue);
        return 1;
    }

    fprintf(stderr,
            "Streaming RTL-SDR -> SPI\n"
            "  SPI device     : %s\n"
            "  Center freq Hz : %u\n"
            "  Sample rate Hz : %u\n"
            "  SPI clock Hz   : %u\n"
            "  Frame bytes    : %zu\n",
            spi_path,
            center_hz,
            sample_rate_hz,
            spi_hz,
            sizeof(iq_spi_frame_t));

    spi_worker_arg_t wa = {
        .q = &queue,
        .spi_fd = spi_fd,
    };
    pthread_t worker;
    if (pthread_create(&worker, NULL, spi_worker, &wa) != 0) {
        perror("pthread_create");
        rtlsdr_close(g_dev);
        close(spi_fd);
        queue_destroy(&queue);
        return 1;
    }

    int r = rtlsdr_read_async(g_dev, rtlsdr_cb, &queue, RTL_ASYNC_BUF_NUM, RTL_ASYNC_BUF_LEN);
    if (r != 0 && !g_stop) {
        fprintf(stderr, "rtlsdr_read_async failed: %d\n", r);
    }

    queue_finish(&queue);
    pthread_join(worker, NULL);

    fprintf(stderr, "Stopped. Dropped frames: %u\n", queue.drops);

    if (g_dev) {
        rtlsdr_close(g_dev);
        g_dev = NULL;
    }
    close(spi_fd);
    queue_destroy(&queue);
    return 0;
}
