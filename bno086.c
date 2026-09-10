#define _POSIX_C_SOURCE 200809L

#include "bno086.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/*
 * ==================================================
 * SH-2 / SHTP Protocol Constants
 * ==================================================
 *
 * BNO086은 하나의 I2C 주소 위에 용도별 SHTP channel을 다중화한다.
 * 실행/재시작 알림은 channel 1, feature 설정과 command 응답은 channel 2,
 * 실제 센서 report는 channel 3/4로 들어온다. SHTP header의 length 상위 bit는
 * continuation 표시이고 하위 15 bit만 header를 포함한 packet 길이다.
 * 따라서 transport 검증과 sensor report 해석을 분리해 두어야 bus 오류와
 * protocol 오류를 혼동하지 않을 수 있다.
 *
 * SHTP header 네 byte는 다음처럼 읽는다.
 *
 *     [length LSB] [length MSB + continuation] [channel] [sequence]
 *
 * 즉 header가 정상인지 먼저 확인해야 뒤의 cargo를 어떤 report로 해석할지
 * 결정할 수 있다. header 오류를 센서값 0으로 취급하면 정상 측정처럼 섞이므로
 * packet 전체를 실패시키고 diagnostics에 원인을 남긴다.
 */

#define CHAN_COMMAND       0u
#define CHAN_EXECUTABLE    1u
#define CHAN_CONTROL       2u
#define CHAN_INPUT_NORMAL  3u
#define CHAN_INPUT_WAKE    4u

#define REPORT_COMMAND_REQ       0xF2u
#define REPORT_COMMAND_RESP      0xF1u
#define REPORT_SET_FEATURE       0xFDu
#define REPORT_GET_FEATURE_REQ   0xFEu
#define REPORT_GET_FEATURE_RESP  0xFCu
#define REPORT_PRODUCT_ID_REQ    0xF9u
#define REPORT_PRODUCT_ID_RESP   0xF8u
#define REPORT_TIMEBASE_REF      0xFBu

#define COMMAND_TARE          0x03u
#define COMMAND_ME_CALIBRATE  0x07u
#define COMMAND_INITIALIZE    0x84u

#define EXECUTABLE_RESET_COMPLETE  0x01u

#define DEFAULT_FEATURE_TIMEOUT_MS  750u
#define READ_PACKET_TIMEOUT_MS      20u
#define FEATURE_REQUERY_DELAY_NS    UINT64_C(10000000)
#define STARTUP_QUIET_NS            UINT64_C(250000000)

#define NS_PER_SECOND       UINT64_C(1000000000)
#define NS_PER_MILLISECOND  UINT64_C(1000000)

#define BNO086_PI 3.14159265358979323846

typedef enum
{
    ERROR_KIND_BUS,
    ERROR_KIND_VALIDATION,
    ERROR_KIND_OTHER

} error_kind_t;

typedef struct
{
    uint16_t total_length;
    uint8_t channel;
    uint8_t sequence;
    size_t cargo_length;
    uint8_t cargo[BNO086_MAX_SHTP_PACKET - 4u];

} shtp_packet_t;

/*
 * ==================================================
 * Driver Internal State
 * ==================================================
 */

typedef struct
{
    char i2c_device[128];
    char gpiochip_device[128];
    uint8_t i2c_address;
    int int_line;
    int reset_line;

    int i2c_fd;
    int gpiochip_fd;
    int int_fd;
    int reset_fd;

    pthread_mutex_t io_mutex;
    pthread_mutex_t state_mutex;
    pthread_cond_t event_cond;
    bool mutexes_initialized;
    bool closing;

    bno086_snapshot_t state;
    bno086_diagnostics_t diagnostics;

    bool feature_interval_valid[256];
    uint32_t feature_interval_us[256];

    uint8_t sequence_out[BNO086_CHANNEL_COUNT];
    uint8_t command_sequence;

    bno086_event_t event_ring[BNO086_EVENT_RING_CAPACITY];
    uint64_t next_event_sequence;

} bno086_impl_t;

/*
 * Lock 소유 규칙:
 *
 * - io_mutex: 한 SHTP read/write transaction을 직렬화한다. I2C byte stream을
 *   여러 Thread가 섞어 읽지 못하게 하는 장치 소유 lock이다.
 * - state_mutex: 공개 snapshot, diagnostics, feature 응답표와 event ring을
 *   보호한다. 장시간 I/O를 기다리는 동안에는 잡지 않는다.
 * - event_cond: event ring producer가 새 sample을 넣었음을 독립 consumer
 *   cursor들에게 알린다. 느린 consumer는 producer를 막지 않고 dropped로
 *   자신의 유실량을 확인한다.
 */

/* 아래 transport 함수들이 파일 후반에 정의되므로 먼저 형식을 고정한다. */
static ssize_t i2c_read_once(int fd, void *buffer, size_t size);
static ssize_t i2c_write_once(int fd, const void *buffer, size_t size);

#ifdef BNO086_TESTING

typedef struct
{
    size_t size;
    uint8_t data[BNO086_MAX_SHTP_PACKET];

} bno086_test_write_record_t;

static const bno086_test_read_step_t *test_read_steps;
static size_t test_read_step_count;
static size_t test_read_index;
static bno086_test_write_record_t test_writes[BNO086_TEST_MAX_WRITES];
static size_t test_write_count_value;
static bool test_transport_active;

#endif

/*
 * ==================================================
 * Time / Byte Helpers
 * ==================================================
 */

static uint64_t monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return 0;
    }

    return (uint64_t)now.tv_sec * NS_PER_SECOND + (uint64_t)now.tv_nsec;
}

static void sleep_ns(uint64_t duration_ns)
{
    struct timespec request;

    request.tv_sec = (time_t)(duration_ns / NS_PER_SECOND);
    request.tv_nsec = (long)(duration_ns % NS_PER_SECOND);

    while (nanosleep(&request, &request) != 0 && errno == EINTR)
    {
        /* 남은 시간만 다시 기다린다. */
    }
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void copy_text(char *destination, size_t destination_size, const char *source)
{
    if (destination == NULL || destination_size == 0u)
    {
        return;
    }

    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }

    (void)snprintf(destination, destination_size, "%s", source);
}

static bno086_impl_t *get_impl(const bno086_t *imu)
{
    if (imu == NULL)
    {
        return NULL;
    }

    return (bno086_impl_t *)imu->impl;
}

/*
 * ==================================================
 * Diagnostic Recording
 * ==================================================
 *
 * 기존 호환용 io_error_count는 모든 transport 계층 실패의 합계다. 원인을
 * 판단할 때는 syscall_bus_error_count와 shtp_validation_error_count를
 * 반드시 따로 본다. 전자는 read/write/ioctl의 실제 Linux I/O 실패 또는
 * short transfer이고, 후자는 읽기는 끝났지만 SHTP header 규칙이 깨진 경우다.
 */

static void record_error(
    bno086_impl_t *impl,
    error_kind_t kind,
    int error_number,
    const char *format,
    ...
)
{
    char message[BNO086_ERROR_TEXT_SIZE];
    va_list arguments;

    va_start(arguments, format);
    (void)vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);

    pthread_mutex_lock(&impl->state_mutex);

    /*
     * 마지막 오류 문자열은 현장에서 즉시 볼 요약이고 counter는 60초 시험처럼
     * 전체 구간의 발생량을 보는 값이다. 새 오류가 와도 누적 counter는 보존된다.
     */
    impl->diagnostics.io_error_count++;
    impl->diagnostics.last_errno = error_number;
    copy_text(
        impl->diagnostics.last_io_error,
        sizeof(impl->diagnostics.last_io_error),
        message
    );

    if (kind == ERROR_KIND_BUS)
    {
        impl->diagnostics.syscall_bus_error_count++;
        copy_text(
            impl->diagnostics.last_syscall_error,
            sizeof(impl->diagnostics.last_syscall_error),
            message
        );
    }
    else if (kind == ERROR_KIND_VALIDATION)
    {
        impl->diagnostics.shtp_validation_error_count++;
        copy_text(
            impl->diagnostics.last_validation_error,
            sizeof(impl->diagnostics.last_validation_error),
            message
        );
    }
    else
    {
        impl->diagnostics.other_transport_error_count++;
    }

    pthread_mutex_unlock(&impl->state_mutex);
}

static void set_failure_reason(bno086_impl_t *impl, const char *format, ...)
{
    va_list arguments;

    pthread_mutex_lock(&impl->state_mutex);
    va_start(arguments, format);
    (void)vsnprintf(
        impl->state.boot_failure_reason,
        sizeof(impl->state.boot_failure_reason),
        format,
        arguments
    );
    va_end(arguments);
    pthread_mutex_unlock(&impl->state_mutex);
}

/*
 * ==================================================
 * Linux GPIO Character Device v2
 * ==================================================
 *
 * libgpiod/lgpio 개발 헤더가 설치되어 있지 않은 Raspberry Pi 환경에서도
 * 빌드되도록 kernel UAPI인 <linux/gpio.h>만 사용한다.
 */

static int gpio_request_input(
    int chip_fd,
    unsigned int line,
    const char *consumer
)
{
    struct gpio_v2_line_request request;

    memset(&request, 0, sizeof(request));
    request.offsets[0] = line;
    request.num_lines = 1u;
    request.config.flags = GPIO_V2_LINE_FLAG_INPUT
                         | GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
    copy_text(request.consumer, sizeof(request.consumer), consumer);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) != 0)
    {
        return -1;
    }

    return request.fd;
}

static int gpio_request_output(
    int chip_fd,
    unsigned int line,
    int initial_value,
    const char *consumer
)
{
    struct gpio_v2_line_request request;

    memset(&request, 0, sizeof(request));
    request.offsets[0] = line;
    request.num_lines = 1u;
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    request.config.num_attrs = 1u;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = initial_value != 0 ? 1u : 0u;
    request.config.attrs[0].mask = 1u;
    copy_text(request.consumer, sizeof(request.consumer), consumer);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) != 0)
    {
        return -1;
    }

    return request.fd;
}

static int gpio_get_value(int line_fd, int *value)
{
    struct gpio_v2_line_values values;

    if (line_fd < 0 || value == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    memset(&values, 0, sizeof(values));
    values.mask = 1u;

    if (ioctl(line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) != 0)
    {
        return -1;
    }

    *value = (values.bits & 1u) != 0u ? 1 : 0;
    return 0;
}

static int gpio_set_value(int line_fd, int value)
{
    struct gpio_v2_line_values values;

    if (line_fd < 0)
    {
        errno = EINVAL;
        return -1;
    }

    memset(&values, 0, sizeof(values));
    values.mask = 1u;
    values.bits = value != 0 ? 1u : 0u;

    return ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
}

/*
 * ==================================================
 * Event Ring
 * ==================================================
 *
 * snapshot은 "현재 최신값"을 읽는 위치 융합용이고, ring은 가속도 sample을
 * 하나도 합치지 않고 순서대로 보는 충돌 검출용이다. 각 event에는 BNO
 * generation을 함께 실어 reset 전후 sample이 하나의 jerk/delta-V 구간으로
 * 연결되지 않게 한다. ring이 가득 차면 가장 오래된 slot을 덮어쓴다.
 */

static void event_ring_push_locked(
    bno086_impl_t *impl,
    bno086_event_type_t type,
    uint8_t report_id,
    uint8_t report_sequence,
    uint8_t accuracy,
    uint64_t timestamp_ns,
    double x,
    double y,
    double z,
    double w
)
{
    bno086_event_t *event;
    uint64_t sequence = impl->next_event_sequence++;
    size_t index = (size_t)((sequence - 1u) % BNO086_EVENT_RING_CAPACITY);

    /* sequence를 slot 번호와 분리해 한 바퀴 돈 뒤 오래된 event인지 구별한다. */
    if (sequence > BNO086_EVENT_RING_CAPACITY)
    {
        impl->diagnostics.event_overwrite_count++;
    }

    event = &impl->event_ring[index];
    event->ring_sequence = sequence;
    event->monotonic_ns = timestamp_ns;
    event->bno_generation = impl->state.bno_generation;
    event->type = type;
    event->report_id = report_id;
    event->report_sequence = report_sequence;
    event->accuracy = accuracy;
    event->x = x;
    event->y = y;
    event->z = z;
    event->w = w;

    pthread_cond_broadcast(&impl->event_cond);
}

/*
 * ==================================================
 * GPIO / Interrupt Helpers
 * ==================================================
 *
 * H_INTN은 active-low다. GPIO line을 사용하지 않는 test backend에서는 읽을
 * mock step이 준비됐는지를 같은 assertion 의미로 반환한다. wait_for_int()는
 * 짧게 polling하여 signal 처리와 timeout 판정을 계속 가능하게 한다.
 */

static int is_int_asserted_internal(bno086_impl_t *impl, bool record_failure)
{
    int value;

#ifdef BNO086_TESTING
    if (test_transport_active)
    {
        if (test_read_index >= test_read_step_count)
        {
            return 0;
        }

        return test_write_count_value
                    >= test_read_steps[test_read_index].required_write_count
             ? 1
             : 0;
    }
#endif

    if (impl->int_line < 0)
    {
        return 1;
    }

    if (gpio_get_value(impl->int_fd, &value) != 0)
    {
        if (record_failure)
        {
            int saved_errno = errno;
            record_error(
                impl,
                ERROR_KIND_BUS,
                saved_errno,
                "GPIO INT read failed: %s",
                strerror(saved_errno)
            );
        }
        return -1;
    }

    /* BNO086 H_INTN은 active-low다. */
    return value == 0 ? 1 : 0;
}

static int wait_for_int(bno086_impl_t *impl, unsigned int timeout_ms)
{
    uint64_t deadline = monotonic_ns()
                      + (uint64_t)timeout_ms * NS_PER_MILLISECOND;

    for (;;)
    {
        int asserted = is_int_asserted_internal(impl, true);

        if (asserted > 0)
        {
            return 1;
        }
        if (asserted < 0)
        {
            return -1;
        }
        if (monotonic_ns() >= deadline)
        {
            return 0;
        }

        /* 0.2 ms마다 확인해 CPU 점유를 낮추면서 짧은 센서 주기를 놓치지 않는다. */
        sleep_ns(200000u);
    }
}

/*
 * ==================================================
 * SHTP Packet Transport
 * ==================================================
 */

static int read_packet(
    bno086_impl_t *impl,
    unsigned int timeout_ms,
    shtp_packet_t *packet
)
{
    uint8_t header[4];
    uint8_t chunk[BNO086_MAX_SHTP_PACKET];
    uint16_t raw_length;
    uint16_t total_length;
    uint8_t phase1_channel;
    uint8_t phase1_sequence;
    bool null_retried = false;
    ssize_t read_size;
    size_t cargo_size;
    size_t remaining;
    int asserted;
    int result = 0;

    if (packet == NULL)
    {
        return -1;
    }

    asserted = is_int_asserted_internal(impl, true);
    if (asserted <= 0)
    {
        return asserted;
    }

    memset(packet, 0, sizeof(*packet));
    pthread_mutex_lock(&impl->io_mutex);

    if (impl->i2c_fd < 0)
    {
        pthread_mutex_unlock(&impl->io_mutex);
        return -1;
    }

    /*
     * Phase 1
     *
     * 첫 transaction에서 정확히 4-byte header를 읽어 전체 packet 길이와
     * channel/sequence를 확정한다. cargo가 있으면 Phase 2의 새 read에서
     * continuation header와 cargo를 함께 받는다. 이 두 단계가 센서의 공식
     * partial-read 형태이며, Phase 1의 length가 이후 모든 검증의 기준이다.
     */
    read_size = i2c_read_once(impl->i2c_fd, header, sizeof(header));
    if (read_size < 0)
    {
        int saved_errno = errno;
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "Phase 1 I2C read failed: %s",
            strerror(saved_errno)
        );
        goto done;
    }
    if (read_size < (ssize_t)sizeof(header))
    {
        record_error(
            impl,
            ERROR_KIND_BUS,
            0,
            "short Phase 1 SHTP header: %zd/4",
            read_size
        );
        goto done;
    }

    raw_length = read_u16_le(header);
    total_length = raw_length & 0x7FFFu;

    /* length 0은 BNO086이 반환할 수 있는 정상 null packet이다. */
    if (total_length == 0u)
    {
        goto done;
    }

    if (total_length < 4u || total_length > BNO086_MAX_SHTP_PACKET)
    {
        record_error(
            impl,
            ERROR_KIND_VALIDATION,
            0,
            "invalid SHTP length %u",
            (unsigned int)total_length
        );
        goto done;
    }

    phase1_channel = header[2];
    phase1_sequence = header[3];

    packet->total_length = total_length;
    packet->channel = phase1_channel;
    packet->sequence = phase1_sequence;

    if (total_length == 4u)
    {
        result = 1;
        goto done;
    }

    /*
     * 현재 실기기에서 검증된 Python 기준 H_INTN handshake를 유지한다.
     * Phase 1 직후 이미 LOW이면 바로 진행하고 HIGH일 때만 continuation 준비를
     * 기다린다. 추가 continuation chunk 사이의 대기도 같은 규칙을 따른다.
     * 여기서 기다리는 대상은 새 sensor report가 아니라 방금 header로 시작한
     * 동일 packet의 나머지 byte다.
     */
    asserted = is_int_asserted_internal(impl, true);
    if (asserted < 0)
    {
        goto done;
    }
    if (asserted == 0)
    {
        int wait_result = wait_for_int(impl, timeout_ms);

        if (wait_result < 0)
        {
            goto done;
        }
        if (wait_result == 0)
        {
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "H_INTN timeout waiting for Phase 2 continuation"
            );
            goto done;
        }
    }

    /*
     * Phase 2
     *
     * Phase 1에서 얻은 total_length만큼 새 I2C read를 요청한다. Linux가
     * 한 번에 일부만 돌려주면 아래 continuation loop가 나머지를 조립한다.
     */
    read_size = i2c_read_once(impl->i2c_fd, chunk, total_length);
    if (read_size < 0)
    {
        int saved_errno = errno;
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "Phase 2 I2C read failed: %s",
            strerror(saved_errno)
        );
        goto done;
    }
    if (read_size < 4)
    {
        record_error(
            impl,
            ERROR_KIND_BUS,
            0,
            "short Phase 2 SHTP header: %zd/4",
            read_size
        );
        goto done;
    }

    /*
     * 실측된 유일한 recovery 예외:
     *
     *     Phase 2 header == 00 00 00 00
     *
     * 인 경우에만 remaining/channel/expected sequence를 바꾸지 않고 같은
     * read를 딱 한 번 즉시 재시도한다. 다른 validation failure에는 retry를
     * 절대 확대하지 않는다. 재시도 packet 역시 continuation bit, Phase 1
     * channel, Phase 1 sequence+1, Phase 1 total length를 모두 만족해야 한다.
     * null 발생 횟수와 recovery 성공/실패는 별도 counter로 남긴다.
     * 예: Phase 1이 15 00 02 1A이고 첫 Phase 2가 00 00 00 00이어도 상태를
     * 전진시키지 않는다. 즉시 재읽은 15 80 02 1B만 정상 continuation으로 받는다.
     */
    if (memcmp(chunk, "\0\0\0\0", 4u) == 0)
    {
        pthread_mutex_lock(&impl->state_mutex);
        impl->diagnostics.phase2_null_count++;
        pthread_mutex_unlock(&impl->state_mutex);

        null_retried = true;
        read_size = i2c_read_once(impl->i2c_fd, chunk, total_length);
        if (read_size < 0)
        {
            int saved_errno = errno;

            pthread_mutex_lock(&impl->state_mutex);
            impl->diagnostics.phase2_null_retry_fail++;
            pthread_mutex_unlock(&impl->state_mutex);
            record_error(
                impl,
                ERROR_KIND_BUS,
                saved_errno,
                "Phase 2 null retry I2C read failed: %s",
                strerror(saved_errno)
            );
            goto done;
        }
        if (read_size < 4)
        {
            pthread_mutex_lock(&impl->state_mutex);
            impl->diagnostics.phase2_null_retry_fail++;
            pthread_mutex_unlock(&impl->state_mutex);
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "short Phase 2 null retry header: %zd/4",
                read_size
            );
            goto done;
        }
    }

    raw_length = read_u16_le(chunk);
    {
        uint16_t continuation_length = raw_length & 0x7FFFu;
        bool continuation = (raw_length & 0x8000u) != 0u;
        uint8_t expected_sequence = (uint8_t)(phase1_sequence + 1u);

        if (!continuation)
        {
            if (null_retried)
            {
                pthread_mutex_lock(&impl->state_mutex);
                impl->diagnostics.phase2_null_retry_fail++;
                pthread_mutex_unlock(&impl->state_mutex);
            }
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "Phase 2 continuation bit not set"
            );
            goto done;
        }
        if (chunk[2] != phase1_channel)
        {
            if (null_retried)
            {
                pthread_mutex_lock(&impl->state_mutex);
                impl->diagnostics.phase2_null_retry_fail++;
                pthread_mutex_unlock(&impl->state_mutex);
            }
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "Phase 2 channel mismatch: %u != %u",
                (unsigned int)chunk[2],
                (unsigned int)phase1_channel
            );
            goto done;
        }
        if (chunk[3] != expected_sequence)
        {
            if (null_retried)
            {
                pthread_mutex_lock(&impl->state_mutex);
                impl->diagnostics.phase2_null_retry_fail++;
                pthread_mutex_unlock(&impl->state_mutex);
            }
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "Phase 2 sequence mismatch: %u != %u",
                (unsigned int)chunk[3],
                (unsigned int)expected_sequence
            );
            goto done;
        }
        if (continuation_length != total_length)
        {
            if (null_retried)
            {
                pthread_mutex_lock(&impl->state_mutex);
                impl->diagnostics.phase2_null_retry_fail++;
                pthread_mutex_unlock(&impl->state_mutex);
            }
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "Phase 2 length mismatch: %u != %u",
                (unsigned int)continuation_length,
                (unsigned int)total_length
            );
            goto done;
        }
    }

    if (null_retried)
    {
        pthread_mutex_lock(&impl->state_mutex);
        impl->diagnostics.phase2_null_retry_success++;
        pthread_mutex_unlock(&impl->state_mutex);
    }

    packet->sequence = chunk[3];
    cargo_size = (size_t)read_size - 4u;
    if (cargo_size > (size_t)total_length - 4u)
    {
        cargo_size = (size_t)total_length - 4u;
    }
    memcpy(packet->cargo, chunk + 4u, cargo_size);
    remaining = (size_t)total_length - 4u - cargo_size;

    while (remaining > 0u)
    {
        size_t request_length = remaining + 4u;
        size_t take;
        int wait_result = wait_for_int(impl, timeout_ms);

        if (wait_result < 0)
        {
            goto done;
        }
        if (wait_result == 0)
        {
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "H_INTN timeout during continuation chunk"
            );
            goto done;
        }

        /*
         * Linux read가 요청 cargo를 한 번에 모두 주지 않은 드문 경우다.
         * 이미 받은 byte는 보존하고 남은 cargo+새 header 길이만 다시 요청한다.
         */
        read_size = i2c_read_once(impl->i2c_fd, chunk, request_length);
        if (read_size < 0)
        {
            int saved_errno = errno;
            record_error(
                impl,
                ERROR_KIND_BUS,
                saved_errno,
                "continuation I2C read failed: %s",
                strerror(saved_errno)
            );
            goto done;
        }
        if (read_size < 4)
        {
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "short continuation chunk: %zd/4",
                read_size
            );
            goto done;
        }

        raw_length = read_u16_le(chunk);
        if ((raw_length & 0x8000u) == 0u)
        {
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "invalid continuation chunk: continuation bit not set"
            );
            goto done;
        }
        if (chunk[2] != phase1_channel)
        {
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "invalid continuation chunk: channel %u != %u",
                (unsigned int)chunk[2],
                (unsigned int)phase1_channel
            );
            goto done;
        }
        if ((raw_length & 0x7FFFu) != request_length)
        {
            record_error(
                impl,
                ERROR_KIND_VALIDATION,
                0,
                "invalid continuation chunk: length %u != %zu",
                (unsigned int)(raw_length & 0x7FFFu),
                request_length
            );
            goto done;
        }

        take = (size_t)read_size - 4u;
        if (take > remaining)
        {
            take = remaining;
        }
        if (take == 0u)
        {
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "continuation chunk contained no cargo"
            );
            goto done;
        }

        memcpy(packet->cargo + cargo_size, chunk + 4u, take);
        cargo_size += take;
        remaining -= take;
    }

    packet->cargo_length = cargo_size;
    result = 1;

done:
    pthread_mutex_unlock(&impl->io_mutex);
    return result;
}

static int send_packet(
    bno086_impl_t *impl,
    uint8_t channel,
    const uint8_t *payload,
    size_t payload_size
)
{
    uint8_t packet[BNO086_MAX_SHTP_PACKET];
    size_t packet_size = payload_size + 4u;
    uint8_t sequence;
    unsigned int attempt;

    if (channel >= BNO086_CHANNEL_COUNT
        || payload == NULL
        || packet_size > sizeof(packet))
    {
        return 0;
    }

    /* SHTP 송신 sequence는 channel마다 독립적으로 증가한다. */
    sequence = impl->sequence_out[channel]++;
    packet[0] = (uint8_t)(packet_size & 0xFFu);
    packet[1] = (uint8_t)((packet_size >> 8) & 0xFFu);
    packet[2] = channel;
    packet[3] = sequence;
    memcpy(packet + 4u, payload, payload_size);

    /* 같은 완성 packet만 재전송하며 retry 도중 sequence를 다시 올리지 않는다. */
    for (attempt = 0u; attempt < 5u; attempt++)
    {
        ssize_t written;

        pthread_mutex_lock(&impl->io_mutex);
        written = i2c_write_once(impl->i2c_fd, packet, packet_size);
        pthread_mutex_unlock(&impl->io_mutex);

        if (written == (ssize_t)packet_size)
        {
            return 1;
        }
        if (written < 0)
        {
            int saved_errno = errno;
            record_error(
                impl,
                ERROR_KIND_BUS,
                saved_errno,
                "SHTP I2C write failed: %s",
                strerror(saved_errno)
            );
        }
        else
        {
            record_error(
                impl,
                ERROR_KIND_BUS,
                0,
                "short SHTP write %zd/%zu",
                written,
                packet_size
            );
        }

        sleep_ns(20000000u);
    }

    return 0;
}

/*
 * ==================================================
 * SHTP Packet Decode
 * ==================================================
 *
 * transport가 완전한 cargo를 조립한 뒤에만 이 계층이 snapshot과 event를
 * 갱신한다. quaternion은 SH-2 Q14, gyro는 Q9 rad/s, magnetic field는
 * Q4 uT, acceleration은 Q8 m/s^2를 실제 단위로 변환한다. report sequence와
 * CLOCK_MONOTONIC 시각은 값과 함께 원자적으로 공개된다.
 * Q 표기는 고정소수점이다. 예를 들어 Q8 acceleration raw 256은 1.0 m/s^2로
 * 나누어 읽는다. 단위 변환을 이곳에서 한 번만 해 상위 코드가 raw scale을
 * 반복해서 알 필요가 없게 한다.
 */

static void update_public_feature_interval_locked(
    bno086_impl_t *impl,
    uint8_t sensor_id,
    uint32_t interval_us,
    bool valid
)
{
    switch (sensor_id)
    {
        case BNO086_SENSOR_ROTATION_VECTOR:
            impl->state.rotation_interval_valid = valid;
            impl->state.rotation_interval_us = interval_us;
            break;

        case BNO086_SENSOR_GYROSCOPE_CALIBRATED:
            impl->state.gyro_interval_valid = valid;
            impl->state.gyro_interval_us = interval_us;
            break;

        case BNO086_SENSOR_MAGNETOMETER_CALIBRATED:
            impl->state.mag_interval_valid = valid;
            impl->state.mag_interval_us = interval_us;
            break;

        case BNO086_SENSOR_ACCELEROMETER:
            impl->state.accel_interval_valid = valid;
            impl->state.accel_interval_us = interval_us;
            break;

        case BNO086_SENSOR_LINEAR_ACCELERATION:
            impl->state.linear_accel_interval_valid = valid;
            impl->state.linear_accel_interval_us = interval_us;
            break;

        default:
            break;
    }
}

static void invalidate_feature_intervals_locked(bno086_impl_t *impl)
{
    memset(impl->feature_interval_valid, 0, sizeof(impl->feature_interval_valid));

    impl->state.rotation_interval_valid = false;
    impl->state.gyro_interval_valid = false;
    impl->state.mag_interval_valid = false;
    impl->state.accel_interval_valid = false;
    impl->state.linear_accel_interval_valid = false;
}

static int process_packet(bno086_impl_t *impl, const shtp_packet_t *packet)
{
    const uint8_t *cargo = packet->cargo;
    size_t cargo_length = packet->cargo_length;
    size_t cursor = 0u;
    int updated_count = 0;

    if (packet->channel < BNO086_CHANNEL_COUNT)
    {
        pthread_mutex_lock(&impl->state_mutex);
        impl->diagnostics.packet_counts[packet->channel]++;
        pthread_mutex_unlock(&impl->state_mutex);
    }

    /*
     * startup 이후 새 advertisement/reset-complete가 오면 hub가 기존 feature
     * 설정을 잃었다. 소비자가 이전 heading과 새 heading을 이어 붙이지 않게
     * startup_ready를 즉시 내리고 generation을 바꾼다. 저장된 feature
     * interval도 무효화하여 reset 전 설정을 현재 설정으로 오인하지 않는다.
     */
    if (packet->channel == CHAN_COMMAND && cargo_length > 0u)
    {
        pthread_mutex_lock(&impl->state_mutex);

        if (cargo[0] == 0x00u)
        {
            if (impl->state.startup_ready)
            {
                impl->state.startup_ready = false;
                impl->state.bno_generation++;
                invalidate_feature_intervals_locked(impl);
                copy_text(
                    impl->state.boot_failure_reason,
                    sizeof(impl->state.boot_failure_reason),
                    "BNO086 restarted (new advertisement)"
                );
            }
        }
        else if (cargo[0] == 0x01u)
        {
            size_t count = cargo_length - 1u;

            if (count > sizeof(impl->diagnostics.last_shtp_errors))
            {
                count = sizeof(impl->diagnostics.last_shtp_errors);
            }
            memcpy(impl->diagnostics.last_shtp_errors, cargo + 1u, count);
            impl->diagnostics.last_shtp_error_count = count;
        }

        pthread_mutex_unlock(&impl->state_mutex);
        return 0;
    }

    if (packet->channel == CHAN_EXECUTABLE
        && cargo_length == 1u
        && cargo[0] == EXECUTABLE_RESET_COMPLETE)
    {
        pthread_mutex_lock(&impl->state_mutex);
        impl->diagnostics.reset_count++;
        impl->state.startup_ready = false;
        impl->state.bno_generation++;
        invalidate_feature_intervals_locked(impl);
        copy_text(
            impl->state.boot_failure_reason,
            sizeof(impl->state.boot_failure_reason),
            "BNO086 reset-complete received during session"
        );
        pthread_mutex_unlock(&impl->state_mutex);
        return 0;
    }

    /* channel 2는 측정값이 아니라 제품/feature/command의 제어 응답이다. */
    if (packet->channel == CHAN_CONTROL && cargo_length > 0u)
    {
        pthread_mutex_lock(&impl->state_mutex);

        if (cargo[0] == REPORT_PRODUCT_ID_RESP && cargo_length >= 2u)
        {
            impl->state.last_reset_cause = cargo[1];
        }
        else if (cargo[0] == REPORT_GET_FEATURE_RESP && cargo_length >= 2u)
        {
            uint8_t sensor_id = cargo[1];

            if (cargo_length >= 9u)
            {
                uint32_t interval_us = read_u32_le(cargo + 5u);

                impl->feature_interval_valid[sensor_id] = true;
                impl->feature_interval_us[sensor_id] = interval_us;
                update_public_feature_interval_locked(
                    impl,
                    sensor_id,
                    interval_us,
                    true
                );
            }
        }
        else if (cargo[0] == REPORT_COMMAND_RESP && cargo_length >= 6u)
        {
            impl->state.last_command_id = cargo[2];
            impl->state.last_command_status = cargo[5];
        }

        pthread_mutex_unlock(&impl->state_mutex);
        return 0;
    }

    if ((packet->channel != CHAN_INPUT_NORMAL
         && packet->channel != CHAN_INPUT_WAKE)
        || cargo_length < 5u)
    {
        return 0;
    }

    /* 하나의 SHTP cargo에 sensor report 여러 개가 연달아 들어올 수 있다. */
    while (cursor < cargo_length)
    {
        uint8_t report_id = cargo[cursor];

        if (report_id == REPORT_TIMEBASE_REF && cursor + 5u <= cargo_length)
        {
            cursor += 5u;
        }
        else if ((report_id == BNO086_SENSOR_GAME_ROTATION_VECTOR
                  && cursor + 12u <= cargo_length)
                 || (report_id == BNO086_SENSOR_ROTATION_VECTOR
                     && cursor + 14u <= cargo_length))
        {
            double qi = (double)read_i16_le(cargo + cursor + 4u) / 16384.0;
            double qj = (double)read_i16_le(cargo + cursor + 6u) / 16384.0;
            double qk = (double)read_i16_le(cargo + cursor + 8u) / 16384.0;
            double qr = (double)read_i16_le(cargo + cursor + 10u) / 16384.0;
            double norm = sqrt(qr * qr + qi * qi + qj * qj + qk * qk);
            double sinr_cosp;
            double cosr_cosp;
            double sinp;
            double siny_cosp;
            double cosy_cosp;
            double roll;
            double pitch;
            double yaw;
            uint8_t accuracy = cargo[cursor + 2u] & 0x03u;
            uint8_t report_sequence = cargo[cursor + 1u];
            uint64_t now = monotonic_ns();

            if (norm > 0.0)
            {
                qr /= norm;
                qi /= norm;
                qj /= norm;
                qk /= norm;
            }

            sinr_cosp = 2.0 * (qr * qi + qj * qk);
            cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj);
            roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / BNO086_PI;

            sinp = 2.0 * (qr * qj - qk * qi);
            if (fabs(sinp) >= 1.0)
            {
                pitch = copysign(90.0, sinp);
            }
            else
            {
                pitch = asin(sinp) * 180.0 / BNO086_PI;
            }

            siny_cosp = 2.0 * (qr * qk + qi * qj);
            cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
            yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / BNO086_PI;

            pthread_mutex_lock(&impl->state_mutex);
            impl->state.roll_deg = roll;
            impl->state.pitch_deg = pitch;
            impl->state.yaw_deg = yaw;
            impl->state.quat_i = qi;
            impl->state.quat_j = qj;
            impl->state.quat_k = qk;
            impl->state.quat_real = qr;
            impl->state.rotation_accuracy = accuracy;
            impl->state.last_update_ns = now;
            impl->state.last_orientation_update_ns = now;
            impl->state.orientation_sequence++;
            event_ring_push_locked(
                impl,
                BNO086_EVENT_ORIENTATION,
                report_id,
                report_sequence,
                accuracy,
                now,
                qi,
                qj,
                qk,
                qr
            );
            pthread_mutex_unlock(&impl->state_mutex);

            updated_count++;
            cursor += report_id == BNO086_SENSOR_ROTATION_VECTOR ? 14u : 12u;
        }
        else if ((report_id == BNO086_SENSOR_ACCELEROMETER
                  || report_id == BNO086_SENSOR_GYROSCOPE_CALIBRATED
                  || report_id == BNO086_SENSOR_MAGNETOMETER_CALIBRATED
                  || report_id == BNO086_SENSOR_LINEAR_ACCELERATION)
                 && cursor + 10u <= cargo_length)
        {
            int16_t raw_x = read_i16_le(cargo + cursor + 4u);
            int16_t raw_y = read_i16_le(cargo + cursor + 6u);
            int16_t raw_z = read_i16_le(cargo + cursor + 8u);
            uint8_t accuracy = cargo[cursor + 2u] & 0x03u;
            uint8_t report_sequence = cargo[cursor + 1u];
            uint64_t now = monotonic_ns();
            double x;
            double y;
            double z;
            bno086_event_type_t event_type;

            if (report_id == BNO086_SENSOR_GYROSCOPE_CALIBRATED)
            {
                x = (double)raw_x / 512.0;
                y = (double)raw_y / 512.0;
                z = (double)raw_z / 512.0;
                event_type = BNO086_EVENT_GYROSCOPE;
            }
            else if (report_id == BNO086_SENSOR_MAGNETOMETER_CALIBRATED)
            {
                x = (double)raw_x / 16.0;
                y = (double)raw_y / 16.0;
                z = (double)raw_z / 16.0;
                event_type = BNO086_EVENT_MAGNETOMETER;
            }
            else
            {
                /* Accelerometer와 Linear Acceleration은 모두 Q8 m/s^2다. */
                x = (double)raw_x / 256.0;
                y = (double)raw_y / 256.0;
                z = (double)raw_z / 256.0;
                event_type = report_id == BNO086_SENSOR_ACCELEROMETER
                           ? BNO086_EVENT_ACCELEROMETER
                           : BNO086_EVENT_LINEAR_ACCELERATION;
            }

            pthread_mutex_lock(&impl->state_mutex);
            impl->state.last_update_ns = now;

            if (report_id == BNO086_SENSOR_GYROSCOPE_CALIBRATED)
            {
                impl->state.gyro_x_rad_s = x;
                impl->state.gyro_y_rad_s = y;
                impl->state.gyro_z_rad_s = z;
                impl->state.gyro_accuracy = accuracy;
                impl->state.last_gyro_update_ns = now;
                impl->state.gyro_sequence++;
            }
            else if (report_id == BNO086_SENSOR_MAGNETOMETER_CALIBRATED)
            {
                impl->state.mag_x_ut = x;
                impl->state.mag_y_ut = y;
                impl->state.mag_z_ut = z;
                impl->state.mag_accuracy = accuracy;
                impl->state.last_mag_update_ns = now;
                impl->state.mag_sequence++;
            }
            else if (report_id == BNO086_SENSOR_ACCELEROMETER)
            {
                impl->state.accel_x_mps2 = x;
                impl->state.accel_y_mps2 = y;
                impl->state.accel_z_mps2 = z;
                impl->state.accel_accuracy = accuracy;
                impl->state.last_accel_update_ns = now;
                impl->state.accel_sequence++;
            }
            else
            {
                impl->state.linear_accel_x_mps2 = x;
                impl->state.linear_accel_y_mps2 = y;
                impl->state.linear_accel_z_mps2 = z;
                impl->state.linear_accel_accuracy = accuracy;
                impl->state.last_linear_accel_update_ns = now;
                impl->state.linear_accel_sequence++;
            }

            event_ring_push_locked(
                impl,
                event_type,
                report_id,
                report_sequence,
                accuracy,
                now,
                x,
                y,
                z,
                0.0
            );
            pthread_mutex_unlock(&impl->state_mutex);

            updated_count++;
            cursor += 10u;
        }
        else
        {
            /* 알려지지 않은 record 뒤에 있는 known record까지 찾는다. */
            cursor++;
        }
    }

    return updated_count;
}

typedef int (*packet_predicate_t)(const shtp_packet_t *packet, const void *context);

static int wait_for_packet(
    bno086_impl_t *impl,
    packet_predicate_t predicate,
    const void *context,
    unsigned int timeout_ms,
    shtp_packet_t *matched_packet
)
{
    uint64_t deadline = monotonic_ns()
                      + (uint64_t)timeout_ms * NS_PER_MILLISECOND;
    uint64_t reset_count_at_start;

    pthread_mutex_lock(&impl->state_mutex);
    reset_count_at_start = impl->diagnostics.reset_count;
    pthread_mutex_unlock(&impl->state_mutex);

    while (monotonic_ns() < deadline)
    {
        int asserted = is_int_asserted_internal(impl, true);

        if (asserted < 0)
        {
            return 0;
        }
        if (asserted > 0)
        {
            shtp_packet_t packet;
            int read_result = read_packet(impl, READ_PACKET_TIMEOUT_MS, &packet);

            if (read_result > 0)
            {
                process_packet(impl, &packet);

                if (predicate(&packet, context))
                {
                    if (matched_packet != NULL)
                    {
                        *matched_packet = packet;
                    }
                    return 1;
                }

                pthread_mutex_lock(&impl->state_mutex);
                if (impl->diagnostics.reset_count != reset_count_at_start)
                {
                    copy_text(
                        impl->state.boot_failure_reason,
                        sizeof(impl->state.boot_failure_reason),
                        "BNO086 reset while waiting for response"
                    );
                    pthread_mutex_unlock(&impl->state_mutex);
                    return 0;
                }
                pthread_mutex_unlock(&impl->state_mutex);
            }
            else
            {
                sleep_ns(1000000u);
            }
        }
        else
        {
            sleep_ns(1000000u);
        }
    }

    return 0;
}

static int product_id_predicate(const shtp_packet_t *packet, const void *context)
{
    (void)context;

    return packet->channel == CHAN_CONTROL
        && packet->cargo_length >= 2u
        && packet->cargo[0] == REPORT_PRODUCT_ID_RESP;
}

typedef struct
{
    uint8_t command;
    uint8_t command_sequence;

} command_match_t;

static int command_response_predicate(
    const shtp_packet_t *packet,
    const void *context
)
{
    const command_match_t *match = context;

    return packet->channel == CHAN_CONTROL
        && packet->cargo_length >= 6u
        && packet->cargo[0] == REPORT_COMMAND_RESP
        && packet->cargo[2] == match->command
        && packet->cargo[3] == match->command_sequence;
}

/*
 * ==================================================
 * Startup Handshake
 * ==================================================
 *
 * reset 직후 무작정 고정 시간만 자고 시작하지 않는다. advertisement로 SHTP가
 * 다시 올라왔는지, reset-complete로 hub reset이 끝났는지, 가능하면 initialize
 * command 응답까지 packet으로 확인한다. 일부 firmware에서 initialize 응답이
 * 노출되지 않는 경우에는 advertisement+reset 이후 충분한 quiet 구간을 확인하고
 * 경고를 남긴 뒤, 이어지는 Product ID/feature/report 확인을 더 엄격히 사용한다.
 */

static int drain_boot_packets(bno086_impl_t *impl, unsigned int timeout_ms)
{
    uint64_t deadline = monotonic_ns()
                      + (uint64_t)timeout_ms * NS_PER_MILLISECOND;
    uint64_t quiet_since = 0u;

    pthread_mutex_lock(&impl->state_mutex);
    impl->state.startup_ready = false;
    impl->state.boot_advertisement_seen = false;
    impl->state.boot_reset_seen = false;
    impl->state.boot_initialize_seen = false;
    impl->state.boot_failure_reason[0] = '\0';
    impl->state.startup_warning[0] = '\0';
    pthread_mutex_unlock(&impl->state_mutex);

    while (monotonic_ns() < deadline)
    {
        int asserted = is_int_asserted_internal(impl, true);

        if (asserted < 0)
        {
            break;
        }
        if (asserted > 0)
        {
            shtp_packet_t packet;
            int read_result;

            quiet_since = 0u;
            read_result = read_packet(impl, 100u, &packet);

            if (read_result > 0)
            {
                pthread_mutex_lock(&impl->state_mutex);

                if (packet.channel == CHAN_COMMAND
                    && packet.cargo_length > 0u
                    && packet.cargo[0] == 0x00u)
                {
                    if (impl->state.boot_reset_seen
                        || impl->state.boot_initialize_seen)
                    {
                        impl->state.boot_reset_seen = false;
                        impl->state.boot_initialize_seen = false;
                    }
                    impl->state.boot_advertisement_seen = true;
                }
                else if (packet.channel == CHAN_COMMAND
                         && packet.cargo_length > 0u
                         && packet.cargo[0] == 0x01u)
                {
                    size_t count = packet.cargo_length - 1u;

                    if (count > sizeof(impl->diagnostics.last_shtp_errors))
                    {
                        count = sizeof(impl->diagnostics.last_shtp_errors);
                    }
                    memcpy(
                        impl->diagnostics.last_shtp_errors,
                        packet.cargo + 1u,
                        count
                    );
                    impl->diagnostics.last_shtp_error_count = count;
                }
                else if (impl->state.boot_advertisement_seen
                         && packet.channel == CHAN_EXECUTABLE
                         && packet.cargo_length == 1u
                         && packet.cargo[0] == EXECUTABLE_RESET_COMPLETE)
                {
                    impl->state.boot_reset_seen = true;
                }
                else if (impl->state.boot_advertisement_seen
                         && impl->state.boot_reset_seen
                         && packet.channel == CHAN_CONTROL
                         && packet.cargo_length >= 7u
                         && packet.cargo[0] == REPORT_COMMAND_RESP
                         && packet.cargo[2] == COMMAND_INITIALIZE
                         && packet.cargo[3] == 0u
                         && packet.cargo[5] == 0u
                         && packet.cargo[6] == 1u)
                {
                    impl->state.boot_initialize_seen = true;
                }

                if (impl->state.boot_advertisement_seen
                    && impl->state.boot_reset_seen
                    && impl->state.boot_initialize_seen)
                {
                    impl->state.startup_ready = true;
                    pthread_mutex_unlock(&impl->state_mutex);
                    return 1;
                }

                pthread_mutex_unlock(&impl->state_mutex);
            }
            else
            {
                sleep_ns(1000000u);
            }
        }
        else
        {
            uint64_t now = monotonic_ns();
            bool boot_base_ready;

            pthread_mutex_lock(&impl->state_mutex);
            boot_base_ready = impl->state.boot_advertisement_seen
                           && impl->state.boot_reset_seen;
            pthread_mutex_unlock(&impl->state_mutex);

            if (boot_base_ready)
            {
                if (quiet_since == 0u)
                {
                    quiet_since = now;
                }
                else if (now - quiet_since >= STARTUP_QUIET_NS)
                {
                    pthread_mutex_lock(&impl->state_mutex);
                    impl->state.startup_ready = true;
                    copy_text(
                        impl->state.startup_warning,
                        sizeof(impl->state.startup_warning),
                        "SH-2 initialize(ch2/F1-84) not observed; using strict control/report confirmation"
                    );
                    pthread_mutex_unlock(&impl->state_mutex);
                    return 1;
                }
            }

            sleep_ns(1000000u);
        }
    }

    pthread_mutex_lock(&impl->state_mutex);
    if (impl->state.boot_advertisement_seen
        && impl->state.boot_reset_seen
        && impl->diagnostics.io_error_count > 0u)
    {
        (void)snprintf(
            impl->state.boot_failure_reason,
            sizeof(impl->state.boot_failure_reason),
            "startup never became idle after reset-complete (I2C: %.96s)",
            impl->diagnostics.last_io_error
        );
    }
    else
    {
        (void)snprintf(
            impl->state.boot_failure_reason,
            sizeof(impl->state.boot_failure_reason),
            "startup incomplete: advertisement=%d reset=%d initialize=%d",
            impl->state.boot_advertisement_seen ? 1 : 0,
            impl->state.boot_reset_seen ? 1 : 0,
            impl->state.boot_initialize_seen ? 1 : 0
        );
    }
    pthread_mutex_unlock(&impl->state_mutex);

    return 0;
}

/*
 * ==================================================
 * Public Lifecycle API
 * ==================================================
 */

void bno086_default_config(bno086_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->i2c_device = "/dev/i2c-1";
    config->i2c_address = 0x4Bu;
    config->gpiochip_device = "/dev/gpiochip0";
    config->int_line = 18;
    config->reset_line = 23;
}

int bno086_init(bno086_t *imu, const bno086_config_t *config)
{
    bno086_config_t defaults;
    bno086_impl_t *impl;
    pthread_condattr_t condition_attributes;
    bool io_mutex_ready = false;
    bool state_mutex_ready = false;
    bool condition_attributes_ready = false;

    if (imu == NULL || imu->impl != NULL)
    {
        return -1;
    }

    bno086_default_config(&defaults);
    if (config == NULL)
    {
        config = &defaults;
    }

    /* init은 하드웨어를 건드리지 않고 Thread 동기화와 빈 상태만 준비한다. */
    impl = calloc(1u, sizeof(*impl));
    if (impl == NULL)
    {
        return 0;
    }

    copy_text(
        impl->i2c_device,
        sizeof(impl->i2c_device),
        config->i2c_device != NULL ? config->i2c_device : defaults.i2c_device
    );
    copy_text(
        impl->gpiochip_device,
        sizeof(impl->gpiochip_device),
        config->gpiochip_device != NULL
            ? config->gpiochip_device
            : defaults.gpiochip_device
    );
    impl->i2c_address = config->i2c_address;
    impl->int_line = config->int_line;
    impl->reset_line = config->reset_line;
    impl->i2c_fd = -1;
    impl->gpiochip_fd = -1;
    impl->int_fd = -1;
    impl->reset_fd = -1;
    impl->state.quat_real = 1.0;
    impl->state.last_command_id = -1;
    impl->state.last_command_status = -1;
    impl->state.last_reset_cause = -1;
    impl->next_event_sequence = 1u;

    if (pthread_mutex_init(&impl->io_mutex, NULL) != 0)
    {
        goto fail;
    }
    io_mutex_ready = true;

    if (pthread_mutex_init(&impl->state_mutex, NULL) != 0)
    {
        goto fail;
    }
    state_mutex_ready = true;

    if (pthread_condattr_init(&condition_attributes) != 0)
    {
        goto fail;
    }
    condition_attributes_ready = true;

    if (pthread_condattr_setclock(&condition_attributes, CLOCK_MONOTONIC) != 0)
    {
        goto fail;
    }

    if (pthread_cond_init(&impl->event_cond, &condition_attributes) != 0)
    {
        goto fail;
    }

    pthread_condattr_destroy(&condition_attributes);
    impl->mutexes_initialized = true;
    imu->impl = impl;
    return 1;

fail:
    if (condition_attributes_ready)
    {
        pthread_condattr_destroy(&condition_attributes);
    }
    if (state_mutex_ready)
    {
        pthread_mutex_destroy(&impl->state_mutex);
    }
    if (io_mutex_ready)
    {
        pthread_mutex_destroy(&impl->io_mutex);
    }
    free(impl);
    return 0;
}

int bno086_open(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return -1;
    }
    if (impl->i2c_fd >= 0)
    {
        return 1;
    }

    /* GPIO line을 먼저 확보해야 reset과 H_INTN을 제어하며 I2C startup을 읽을 수 있다. */
    if (impl->int_line >= 0 || impl->reset_line >= 0)
    {
        impl->gpiochip_fd = open(impl->gpiochip_device, O_RDONLY | O_CLOEXEC);
        if (impl->gpiochip_fd < 0)
        {
            int saved_errno = errno;
            record_error(
                impl,
                ERROR_KIND_BUS,
                saved_errno,
                "open %s failed: %s",
                impl->gpiochip_device,
                strerror(saved_errno)
            );
            return 0;
        }

        if (impl->reset_line >= 0)
        {
            impl->reset_fd = gpio_request_output(
                impl->gpiochip_fd,
                (unsigned int)impl->reset_line,
                1,
                "bno086-reset"
            );
            if (impl->reset_fd < 0)
            {
                int saved_errno = errno;
                record_error(
                    impl,
                    ERROR_KIND_BUS,
                    saved_errno,
                    "claim BNO086 reset GPIO %d failed: %s",
                    impl->reset_line,
                    strerror(saved_errno)
                );
                bno086_close(imu);
                return 0;
            }
        }

        if (impl->int_line >= 0)
        {
            impl->int_fd = gpio_request_input(
                impl->gpiochip_fd,
                (unsigned int)impl->int_line,
                "bno086-int"
            );
            if (impl->int_fd < 0)
            {
                int saved_errno = errno;
                record_error(
                    impl,
                    ERROR_KIND_BUS,
                    saved_errno,
                    "claim BNO086 INT GPIO %d failed: %s",
                    impl->int_line,
                    strerror(saved_errno)
                );
                bno086_close(imu);
                return 0;
            }
        }
    }

    impl->i2c_fd = open(impl->i2c_device, O_RDWR | O_CLOEXEC);
    if (impl->i2c_fd < 0)
    {
        int saved_errno = errno;
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "open %s failed: %s",
            impl->i2c_device,
            strerror(saved_errno)
        );
        bno086_close(imu);
        return 0;
    }

    if (ioctl(impl->i2c_fd, I2C_SLAVE, impl->i2c_address) != 0)
    {
        int saved_errno = errno;
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "select I2C address 0x%02X failed: %s",
            (unsigned int)impl->i2c_address,
            strerror(saved_errno)
        );
        bno086_close(imu);
        return 0;
    }

    pthread_mutex_lock(&impl->state_mutex);
    impl->closing = false;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

void bno086_close(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return;
    }

    /* 대기 중인 event consumer를 먼저 깨워 fd가 닫힌 뒤 영원히 기다리지 않게 한다. */
    pthread_mutex_lock(&impl->state_mutex);
    impl->closing = true;
    pthread_cond_broadcast(&impl->event_cond);
    pthread_mutex_unlock(&impl->state_mutex);

    pthread_mutex_lock(&impl->io_mutex);

    if (impl->reset_fd >= 0)
    {
        /* line handle을 놓기 직전에도 active-low reset을 해제해 둔다. */
        (void)gpio_set_value(impl->reset_fd, 1);
    }
    if (impl->i2c_fd >= 0)
    {
        close(impl->i2c_fd);
        impl->i2c_fd = -1;
    }
    if (impl->int_fd >= 0)
    {
        close(impl->int_fd);
        impl->int_fd = -1;
    }
    if (impl->reset_fd >= 0)
    {
        close(impl->reset_fd);
        impl->reset_fd = -1;
    }
    if (impl->gpiochip_fd >= 0)
    {
        close(impl->gpiochip_fd);
        impl->gpiochip_fd = -1;
    }

    pthread_mutex_unlock(&impl->io_mutex);
}

void bno086_deinit(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return;
    }

    bno086_close(imu);

    if (impl->mutexes_initialized)
    {
        pthread_cond_destroy(&impl->event_cond);
        pthread_mutex_destroy(&impl->state_mutex);
        pthread_mutex_destroy(&impl->io_mutex);
    }

    free(impl);
    imu->impl = NULL;
}

int bno086_is_open(const bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return 0;
    }

    return impl->i2c_fd >= 0 ? 1 : 0;
}

int bno086_reset_hardware(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL || impl->i2c_fd < 0)
    {
        return -1;
    }

    /* reset 전 값은 더 이상 현재 센서 상태가 아니므로 시각/feature를 먼저 무효화한다. */
    pthread_mutex_lock(&impl->state_mutex);
    impl->state.startup_ready = false;
    impl->state.boot_advertisement_seen = false;
    impl->state.boot_reset_seen = false;
    impl->state.boot_initialize_seen = false;
    impl->state.last_update_ns = 0u;
    impl->state.last_orientation_update_ns = 0u;
    impl->state.last_gyro_update_ns = 0u;
    impl->state.last_mag_update_ns = 0u;
    impl->state.last_accel_update_ns = 0u;
    impl->state.last_linear_accel_update_ns = 0u;
    impl->state.bno_generation++;
    invalidate_feature_intervals_locked(impl);
    pthread_mutex_unlock(&impl->state_mutex);

    /* 외부 reset GPIO를 쓰지 않는 구성에서는 상태 경계만 만들고 성공한다. */
    if (impl->reset_line < 0)
    {
        return 1;
    }

    pthread_mutex_lock(&impl->io_mutex);
    if (gpio_set_value(impl->reset_fd, 0) != 0)
    {
        int saved_errno = errno;
        pthread_mutex_unlock(&impl->io_mutex);
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "assert BNO086 reset failed: %s",
            strerror(saved_errno)
        );
        return 0;
    }

    sleep_ns(10000000u);

    if (gpio_set_value(impl->reset_fd, 1) != 0)
    {
        int saved_errno = errno;
        pthread_mutex_unlock(&impl->io_mutex);
        record_error(
            impl,
            ERROR_KIND_BUS,
            saved_errno,
            "release BNO086 reset failed: %s",
            strerror(saved_errno)
        );
        return 0;
    }
    pthread_mutex_unlock(&impl->io_mutex);

    sleep_ns(5000000u);
    return 1;
}

int bno086_is_int_asserted(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return -1;
    }

    return is_int_asserted_internal(impl, true);
}

int bno086_begin(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);
    const uint8_t product_id_request[2] = {REPORT_PRODUCT_ID_REQ, 0x00u};
    shtp_packet_t product_packet;

    if (impl == NULL)
    {
        return -1;
    }

    /* open -> 물리 reset -> boot packet 확인 -> Product ID 응답의 순서를 지킨다. */
    if (!bno086_open(imu))
    {
        return 0;
    }
    if (!bno086_reset_hardware(imu))
    {
        return 0;
    }
    if (!drain_boot_packets(impl, 2000u))
    {
        return 0;
    }

    /* reset 뒤 host-to-hub channel sequence도 0부터 시작한다. */
    memset(impl->sequence_out, 0, sizeof(impl->sequence_out));
    impl->command_sequence = 0u;

    if (!send_packet(
            impl,
            CHAN_CONTROL,
            product_id_request,
            sizeof(product_id_request)))
    {
        set_failure_reason(impl, "Product ID request write failed");
        return 0;
    }

    if (!wait_for_packet(
            impl,
            product_id_predicate,
            NULL,
            800u,
            &product_packet))
    {
        pthread_mutex_lock(&impl->state_mutex);
        if (impl->state.boot_failure_reason[0] == '\0')
        {
            copy_text(
                impl->state.boot_failure_reason,
                sizeof(impl->state.boot_failure_reason),
                "no Product ID response"
            );
        }
        pthread_mutex_unlock(&impl->state_mutex);
        return 0;
    }

    return 1;
}

/*
 * ==================================================
 * Feature Configuration
 * ==================================================
 *
 * Set Feature(0xFD) 전송만으로 성공을 선언하지 않는다. 같은 deadline 안에서
 * Get Feature(0xFE)를 보내고 해당 sensor ID의 Feature Response(0xFC)를 직접
 * 확인한다. enable 요청에 0 us FC가 먼저 오는 SH-2 race가 있으므로 그 FC는
 * 과도 상태로 기록하고 잠시 뒤 FE를 다시 보낸다. 반대로 disable 요청은
 * 0 us가 확인되어야 성공이다. SH-2가 지원 rate로 양자화할 수 있으므로
 * enable의 최종 성공 조건은 요청값과의 일치가 아니라 non-zero 적용값이다.
 * 대기 중 reset/startup loss가 생기면 이전 generation의 FC를 받을 수 있으므로
 * timeout까지 기다리지 않고 즉시 실패한다.
 *
 * 관측 가능한 race를 예로 들면 FC interval이 [0, 10000] 순서로 올 수 있다.
 * 첫 0은 "설정 실패"로 확정하지 않고 다시 FE를 보내 현재값을 묻는다. 두 번째
 * non-zero가 오면 실제 적용값을 저장하고 성공한다. 반면 끝까지 0이거나 FC가
 * 없을 때만 전체 timeout에서 실패한다.
 */

static int configuration_session_valid(
    bno086_impl_t *impl,
    uint64_t reset_count_at_start
)
{
    int valid;

    pthread_mutex_lock(&impl->state_mutex);
    valid = impl->state.startup_ready
         && impl->diagnostics.reset_count == reset_count_at_start;
    pthread_mutex_unlock(&impl->state_mutex);

    return valid;
}

int bno086_enable_feature(
    bno086_t *imu,
    uint8_t sensor_id,
    uint32_t interval_us,
    unsigned int timeout_ms
)
{
    bno086_impl_t *impl = get_impl(imu);
    uint8_t set_feature[17];
    uint8_t get_feature[2] = {REPORT_GET_FEATURE_REQ, sensor_id};
    uint64_t deadline;
    uint64_t next_query_at;
    uint64_t reset_count_at_start;
    bool response_is_eligible = false;

    if (impl == NULL)
    {
        return -1;
    }
    if (timeout_ms == 0u)
    {
        timeout_ms = DEFAULT_FEATURE_TIMEOUT_MS;
    }

    pthread_mutex_lock(&impl->state_mutex);
    if (!impl->state.startup_ready)
    {
        copy_text(
            impl->state.boot_failure_reason,
            sizeof(impl->state.boot_failure_reason),
            "feature requested before a complete SH-2 startup"
        );
        pthread_mutex_unlock(&impl->state_mutex);
        return 0;
    }
    reset_count_at_start = impl->diagnostics.reset_count;
    pthread_mutex_unlock(&impl->state_mutex);

    deadline = monotonic_ns() + (uint64_t)timeout_ms * NS_PER_MILLISECOND;

    memset(set_feature, 0, sizeof(set_feature));
    set_feature[0] = REPORT_SET_FEATURE;
    set_feature[1] = sensor_id;
    write_u32_le(set_feature + 5u, interval_us);

    if (!send_packet(impl, CHAN_CONTROL, set_feature, sizeof(set_feature)))
    {
        set_failure_reason(
            impl,
            "failed to send 0xFD set feature for sensor 0x%02X",
            (unsigned int)sensor_id
        );
        return 0;
    }

    next_query_at = monotonic_ns();

    while (monotonic_ns() < deadline)
    {
        uint64_t now;
        int asserted;

        if (!configuration_session_valid(impl, reset_count_at_start))
        {
            set_failure_reason(
                impl,
                "BNO086 startup lost while configuring sensor 0x%02X",
                (unsigned int)sensor_id
            );
            return 0;
        }

        now = monotonic_ns();
        if (now >= next_query_at)
        {
            if (send_packet(
                    impl,
                    CHAN_CONTROL,
                    get_feature,
                    sizeof(get_feature)))
            {
                response_is_eligible = true;
                next_query_at = deadline;
            }
            else
            {
                response_is_eligible = false;
                next_query_at = monotonic_ns() + FEATURE_REQUERY_DELAY_NS;
                if (next_query_at > deadline)
                {
                    next_query_at = deadline;
                }
            }

            if (!configuration_session_valid(impl, reset_count_at_start))
            {
                set_failure_reason(
                    impl,
                    "BNO086 startup lost while configuring sensor 0x%02X",
                    (unsigned int)sensor_id
                );
                return 0;
            }
        }

        asserted = is_int_asserted_internal(impl, true);
        if (asserted < 0)
        {
            return 0;
        }
        if (asserted > 0)
        {
            shtp_packet_t packet;
            int read_result = read_packet(
                impl,
                READ_PACKET_TIMEOUT_MS,
                &packet
            );

            if (read_result <= 0)
            {
                sleep_ns(1000000u);
                continue;
            }

            process_packet(impl, &packet);

            if (!configuration_session_valid(impl, reset_count_at_start))
            {
                set_failure_reason(
                    impl,
                    "BNO086 startup lost while configuring sensor 0x%02X",
                    (unsigned int)sensor_id
                );
                return 0;
            }

            if (packet.channel == CHAN_CONTROL
                && packet.cargo_length >= 9u
                && packet.cargo[0] == REPORT_GET_FEATURE_RESP
                && packet.cargo[1] == sensor_id
                && response_is_eligible)
            {
                uint32_t applied_interval_us = read_u32_le(packet.cargo + 5u);
                bool state_matches_request = interval_us > 0u
                                           ? applied_interval_us > 0u
                                           : applied_interval_us == 0u;

                /*
                 * 요청값과 정확히 같을 필요는 없다. SH-2가 조정한 non-zero
                 * interval도 실제 적용값으로 인정하며 process_packet()이 이미
                 * snapshot/feature table에 기록했다.
                 */
                if (state_matches_request)
                {
                    return 1;
                }

                /* stale/transitional FC는 실패가 아니라 "한 번 더 물어보라"는 상태다. */
                response_is_eligible = false;
                next_query_at = monotonic_ns() + FEATURE_REQUERY_DELAY_NS;
                if (next_query_at > deadline)
                {
                    next_query_at = deadline;
                }
            }
        }
        else
        {
            sleep_ns(1000000u);
        }
    }

    set_failure_reason(
        impl,
        "no valid %s FC feature response for sensor 0x%02X before timeout",
        interval_us > 0u ? "non-zero" : "zero",
        (unsigned int)sensor_id
    );
    return 0;
}

int bno086_enable_rotation_vector(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_ROTATION_VECTOR,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

int bno086_enable_game_rotation_vector(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_GAME_ROTATION_VECTOR,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

int bno086_enable_gyroscope(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_GYROSCOPE_CALIBRATED,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

int bno086_enable_magnetometer(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

int bno086_enable_accelerometer(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_ACCELEROMETER,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

int bno086_enable_linear_acceleration(bno086_t *imu, uint32_t interval_us)
{
    return bno086_enable_feature(
        imu,
        BNO086_SENSOR_LINEAR_ACCELERATION,
        interval_us,
        DEFAULT_FEATURE_TIMEOUT_MS
    );
}

static uint64_t sensor_sequence_from_snapshot(
    const bno086_snapshot_t *snapshot,
    uint8_t sensor_id
)
{
    switch (sensor_id)
    {
        case BNO086_SENSOR_ROTATION_VECTOR:
        case BNO086_SENSOR_GAME_ROTATION_VECTOR:
            return snapshot->orientation_sequence;

        case BNO086_SENSOR_GYROSCOPE_CALIBRATED:
            return snapshot->gyro_sequence;

        case BNO086_SENSOR_MAGNETOMETER_CALIBRATED:
            return snapshot->mag_sequence;

        case BNO086_SENSOR_ACCELEROMETER:
            return snapshot->accel_sequence;

        case BNO086_SENSOR_LINEAR_ACCELERATION:
            return snapshot->linear_accel_sequence;

        default:
            return UINT64_MAX;
    }
}

int bno086_wait_for_sensor_report(
    bno086_t *imu,
    uint8_t sensor_id,
    unsigned int timeout_ms
)
{
    bno086_impl_t *impl = get_impl(imu);
    bno086_snapshot_t snapshot;
    uint64_t initial_sequence;
    uint64_t deadline;

    if (impl == NULL)
    {
        return -1;
    }
    if (!bno086_get_snapshot(imu, &snapshot))
    {
        return 0;
    }

    /*
     * FC는 설정 상태만 증명한다. 이 함수는 호출 당시 sequence를 기준으로
     * channel 3/4의 실제 input report가 새로 도착해야 성공하므로, startup은
     * "FD -> FE/FC -> 실제 report"의 두 단계로 검증된다.
     */
    initial_sequence = sensor_sequence_from_snapshot(&snapshot, sensor_id);
    if (initial_sequence == UINT64_MAX)
    {
        return 0;
    }

    deadline = monotonic_ns() + (uint64_t)timeout_ms * NS_PER_MILLISECOND;
    while (monotonic_ns() < deadline)
    {
        (void)bno086_update(imu);
        (void)bno086_get_snapshot(imu, &snapshot);

        if (sensor_sequence_from_snapshot(&snapshot, sensor_id) > initial_sequence)
        {
            return 1;
        }
        if (!snapshot.startup_ready)
        {
            return 0;
        }

        sleep_ns(1000000u);
    }

    set_failure_reason(
        impl,
        "no input report for sensor 0x%02X",
        (unsigned int)sensor_id
    );
    return 0;
}

/*
 * ==================================================
 * Motion Engine Commands
 * ==================================================
 *
 * command sequence를 request/response에 함께 사용하여 다른 command 응답을
 * 성공으로 오인하지 않는다. 동적 보정은 calibration engine을 켤 뿐 accuracy
 * 완료를 뜻하지 않으며, 호출자는 RV/Mag accuracy를 별도로 관찰해야 한다.
 * 즉 command 성공은 "보정 기능을 시작했다"는 뜻이지 "지금 accuracy=3"이라는
 * 뜻이 아니다. 실제 준비 여부는 계속 들어오는 report의 accuracy로 판단한다.
 */

int bno086_start_dynamic_calibration(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);
    uint8_t payload[12] = {
        REPORT_COMMAND_REQ, 0u, COMMAND_ME_CALIBRATE,
        0u, 1u, 1u, 0u, 0u, 0u, 0u, 0u, 0u
    };
    uint8_t command_sequence;
    command_match_t match;
    shtp_packet_t response;

    if (impl == NULL)
    {
        return -1;
    }

    command_sequence = impl->command_sequence++;
    payload[1] = command_sequence;
    match.command = COMMAND_ME_CALIBRATE;
    match.command_sequence = command_sequence;

    if (!send_packet(impl, CHAN_CONTROL, payload, sizeof(payload)))
    {
        return 0;
    }

    if (!wait_for_packet(
            impl,
            command_response_predicate,
            &match,
            750u,
            &response))
    {
        set_failure_reason(impl, "no response to dynamic calibration command");
        return 0;
    }

    if (response.cargo[5] != 0u)
    {
        set_failure_reason(
            impl,
            "dynamic calibration command failed (R0=%u)",
            (unsigned int)response.cargo[5]
        );
        return 0;
    }

    return 1;
}

int bno086_tare_all_axes(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);
    uint8_t payload[12] = {
        REPORT_COMMAND_REQ, 0u, COMMAND_TARE,
        0u, 0x07u, 0x00u, 0u, 0u, 0u, 0u, 0u, 0u
    };
    bool ready;

    if (impl == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    ready = impl->state.startup_ready;
    pthread_mutex_unlock(&impl->state_mutex);
    if (!ready)
    {
        return 0;
    }

    payload[1] = impl->command_sequence++;
    return send_packet(impl, CHAN_CONTROL, payload, sizeof(payload));
}

/*
 * ==================================================
 * Packet Service
 * ==================================================
 *
 * 한 호출에서 최대 8 packet만 drain해 다른 Thread 작업을 굶기지 않는다.
 * 반환값은 읽은 packet 수가 아니라 새 sensor report로 갱신된 항목 수다.
 */

int bno086_update(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);
    int updated_count = 0;
    unsigned int index;

    if (impl == NULL)
    {
        return -1;
    }

    for (index = 0u; index < 8u; index++)
    {
        shtp_packet_t packet;
        int asserted = is_int_asserted_internal(impl, true);
        int read_result;

        if (asserted <= 0)
        {
            break;
        }

        read_result = read_packet(impl, READ_PACKET_TIMEOUT_MS, &packet);
        if (read_result <= 0)
        {
            break;
        }

        updated_count += process_packet(impl, &packet);
    }

    return updated_count;
}

/*
 * ==================================================
 * Thread-safe Snapshot / Diagnostics
 * ==================================================
 *
 * 호출자에게 lock 내부 포인터를 노출하지 않고 구조체 전체를 복사한다.
 * 따라서 위치/IPC Thread는 필드들이 서로 다른 report 순간에 섞이지 않은
 * 일관된 snapshot을 짧은 lock 구간으로 얻는다.
 */

int bno086_get_snapshot(const bno086_t *imu, bno086_snapshot_t *snapshot)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL || snapshot == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    *snapshot = impl->state;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

int bno086_get_diagnostics(
    const bno086_t *imu,
    bno086_diagnostics_t *diagnostics
)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL || diagnostics == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    *diagnostics = impl->diagnostics;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

int bno086_get_feature_interval(
    const bno086_t *imu,
    uint8_t sensor_id,
    uint32_t *interval_us
)
{
    bno086_impl_t *impl = get_impl(imu);
    int valid;

    if (impl == NULL || interval_us == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    valid = impl->feature_interval_valid[sensor_id] ? 1 : 0;
    if (valid)
    {
        *interval_us = impl->feature_interval_us[sensor_id];
    }
    pthread_mutex_unlock(&impl->state_mutex);

    return valid;
}

/*
 * ==================================================
 * SPMC Event Cursor
 * ==================================================
 *
 * 한 producer(BNO packet decoder)와 여러 consumer를 위한 cursor 방식이다.
 * consumer마다 next_sequence가 따로 있어 위치 융합과 충돌 검출이 동일 sample을
 * 독립적으로 볼 수 있다. producer는 consumer 진행을 기다리지 않으며, ring
 * 용량을 넘긴 consumer만 자신의 cursor.dropped가 증가한다.
 */

static uint64_t oldest_event_sequence_locked(const bno086_impl_t *impl)
{
    if (impl->next_event_sequence > BNO086_EVENT_RING_CAPACITY)
    {
        return impl->next_event_sequence - BNO086_EVENT_RING_CAPACITY;
    }

    return 1u;
}

int bno086_event_cursor_init(
    const bno086_t *imu,
    bno086_event_cursor_t *cursor,
    bool include_buffered
)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL || cursor == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    cursor->next_sequence = include_buffered
                          ? oldest_event_sequence_locked(impl)
                          : impl->next_event_sequence;
    cursor->dropped = 0u;
    cursor->initialized = true;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

int bno086_event_next(
    const bno086_t *imu,
    bno086_event_cursor_t *cursor,
    bno086_event_t *event,
    int timeout_ms
)
{
    bno086_impl_t *impl = get_impl(imu);
    struct timespec deadline;
    bool deadline_valid = false;

    if (impl == NULL || cursor == NULL || event == NULL || !cursor->initialized)
    {
        return -1;
    }

    if (timeout_ms > 0)
    {
        uint64_t deadline_ns = monotonic_ns()
                             + (uint64_t)timeout_ms * NS_PER_MILLISECOND;

        deadline.tv_sec = (time_t)(deadline_ns / NS_PER_SECOND);
        deadline.tv_nsec = (long)(deadline_ns % NS_PER_SECOND);
        deadline_valid = true;
    }

    pthread_mutex_lock(&impl->state_mutex);

    for (;;)
    {
        uint64_t oldest_sequence = oldest_event_sequence_locked(impl);

        if (cursor->next_sequence < oldest_sequence)
        {
            cursor->dropped += oldest_sequence - cursor->next_sequence;
            cursor->next_sequence = oldest_sequence;
        }

        if (cursor->next_sequence < impl->next_event_sequence)
        {
            size_t index = (size_t)(
                (cursor->next_sequence - 1u) % BNO086_EVENT_RING_CAPACITY
            );
            const bno086_event_t *stored = &impl->event_ring[index];

            /*
             * producer가 lock을 잡고 slot을 쓴 뒤 sequence를 공개하므로 보통
             * 항상 일치한다. 방어적으로 불일치하면 다음 유효 범위로 재동기화한다.
             */
            if (stored->ring_sequence != cursor->next_sequence)
            {
                cursor->dropped++;
                cursor->next_sequence++;
                continue;
            }

            *event = *stored;
            cursor->next_sequence++;
            pthread_mutex_unlock(&impl->state_mutex);
            return 1;
        }

        if (impl->closing || timeout_ms == 0)
        {
            pthread_mutex_unlock(&impl->state_mutex);
            return 0;
        }

        if (timeout_ms < 0)
        {
            (void)pthread_cond_wait(&impl->event_cond, &impl->state_mutex);
        }
        else if (deadline_valid)
        {
            int wait_result = pthread_cond_timedwait(
                &impl->event_cond,
                &impl->state_mutex,
                &deadline
            );

            if (wait_result == ETIMEDOUT)
            {
                pthread_mutex_unlock(&impl->state_mutex);
                return 0;
            }
            if (wait_result != 0)
            {
                pthread_mutex_unlock(&impl->state_mutex);
                return -1;
            }
        }
    }
}

int bno086_status_summary(
    const bno086_t *imu,
    char *buffer,
    size_t buffer_size
)
{
    bno086_snapshot_t state;
    bno086_diagnostics_t diagnostics;
    int written;

    if (buffer == NULL || buffer_size == 0u)
    {
        return -1;
    }
    if (bno086_get_snapshot(imu, &state) <= 0
        || bno086_get_diagnostics(imu, &diagnostics) <= 0)
    {
        buffer[0] = '\0';
        return -1;
    }

    written = snprintf(
        buffer,
        buffer_size,
        "ready=%d, boot A/R/I=%d/%d/%d, resets=%llu, "
        "bus_errors=%llu, validation_errors=%llu, "
        "phase2_null=%llu/%llu/%llu%s%s",
        state.startup_ready ? 1 : 0,
        state.boot_advertisement_seen ? 1 : 0,
        state.boot_reset_seen ? 1 : 0,
        state.boot_initialize_seen ? 1 : 0,
        (unsigned long long)diagnostics.reset_count,
        (unsigned long long)diagnostics.syscall_bus_error_count,
        (unsigned long long)diagnostics.shtp_validation_error_count,
        (unsigned long long)diagnostics.phase2_null_count,
        (unsigned long long)diagnostics.phase2_null_retry_success,
        (unsigned long long)diagnostics.phase2_null_retry_fail,
        diagnostics.last_io_error[0] != '\0' ? ", last=" : "",
        diagnostics.last_io_error[0] != '\0' ? diagnostics.last_io_error : ""
    );

    if (written < 0)
    {
        buffer[0] = '\0';
        return 0;
    }

    return 1;
}

/*
 * ==================================================
 * I2C Read / Write
 * ==================================================
 *
 * EINTR만 같은 syscall에서 재시도한다. NACK, Remote I/O, timeout 등 다른
 * errno와 short transfer는 상위 transport가 실제 bus 오류로 분류한다.
 * BNO086_TESTING backend도 같은 read/write 경계를 재현해 protocol 검증을
 * 하드웨어 없이 시험할 수 있게 한다.
 */

static ssize_t i2c_read_once(int fd, void *buffer, size_t size)
{
    ssize_t result;

#ifdef BNO086_TESTING
    if (test_transport_active)
    {
        const bno086_test_read_step_t *step;
        size_t copy_size;

        (void)fd;
        if (test_read_index >= test_read_step_count)
        {
            errno = EAGAIN;
            return -1;
        }

        step = &test_read_steps[test_read_index++];
        if (step->error_number != 0)
        {
            errno = step->error_number;
            return -1;
        }

        copy_size = step->size < size ? step->size : size;
        if (copy_size > 0u && step->data != NULL)
        {
            memcpy(buffer, step->data, copy_size);
        }
        return (ssize_t)copy_size;
    }
#endif

    do
    {
        result = read(fd, buffer, size);
    }
    while (result < 0 && errno == EINTR);

    return result;
}

static ssize_t i2c_write_once(int fd, const void *buffer, size_t size)
{
    ssize_t result;

#ifdef BNO086_TESTING
    if (test_transport_active)
    {
        (void)fd;
        if (test_write_count_value < BNO086_TEST_MAX_WRITES)
        {
            bno086_test_write_record_t *record =
                &test_writes[test_write_count_value];
            size_t copy_size = size < sizeof(record->data)
                             ? size
                             : sizeof(record->data);

            memcpy(record->data, buffer, copy_size);
            record->size = copy_size;
        }
        test_write_count_value++;
        return (ssize_t)size;
    }
#endif

    do
    {
        result = write(fd, buffer, size);
    }
    while (result < 0 && errno == EINTR);

    return result;
}

#ifdef BNO086_TESTING

/*
 * ==================================================
 * Deterministic Test Hooks
 * ==================================================
 *
 * 실제 장치가 없는 개발 환경에서 raw SHTP read sequence를 주입한다.
 * production build에는 이 코드와 symbol이 전혀 포함되지 않는다.
 */

void bno086_test_transport_reset(void)
{
    test_read_steps = NULL;
    test_read_step_count = 0u;
    test_read_index = 0u;
    memset(test_writes, 0, sizeof(test_writes));
    test_write_count_value = 0u;
    test_transport_active = true;
}

void bno086_test_set_read_script(
    const bno086_test_read_step_t *steps,
    size_t step_count
)
{
    test_read_steps = steps;
    test_read_step_count = step_count;
    test_read_index = 0u;
}

size_t bno086_test_read_call_count(void)
{
    return test_read_index;
}

size_t bno086_test_write_count(void)
{
    return test_write_count_value;
}

size_t bno086_test_copy_write(
    size_t index,
    uint8_t *buffer,
    size_t buffer_size
)
{
    size_t copy_size;

    if (index >= test_write_count_value
        || index >= BNO086_TEST_MAX_WRITES
        || buffer == NULL)
    {
        return 0u;
    }

    copy_size = test_writes[index].size < buffer_size
              ? test_writes[index].size
              : buffer_size;
    memcpy(buffer, test_writes[index].data, copy_size);
    return copy_size;
}

int bno086_test_make_virtual_open(bno086_t *imu)
{
    bno086_impl_t *impl = get_impl(imu);
    int virtual_fd;

    if (impl == NULL)
    {
        return -1;
    }

    virtual_fd = open("/dev/null", O_RDWR | O_CLOEXEC);
    if (virtual_fd < 0)
    {
        return 0;
    }

    if (impl->i2c_fd >= 0)
    {
        close(impl->i2c_fd);
    }
    impl->i2c_fd = virtual_fd;
    impl->int_line = -1;
    impl->reset_line = -1;

    pthread_mutex_lock(&impl->state_mutex);
    impl->closing = false;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

int bno086_test_set_startup_ready(bno086_t *imu, bool ready)
{
    bno086_impl_t *impl = get_impl(imu);

    if (impl == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&impl->state_mutex);
    impl->state.startup_ready = ready;
    pthread_mutex_unlock(&impl->state_mutex);
    return 1;
}

int bno086_test_read_packet(
    bno086_t *imu,
    bno086_test_packet_t *packet
)
{
    bno086_impl_t *impl = get_impl(imu);
    shtp_packet_t internal_packet;
    int result;

    if (impl == NULL || packet == NULL)
    {
        return -1;
    }

    result = read_packet(impl, READ_PACKET_TIMEOUT_MS, &internal_packet);
    if (result > 0)
    {
        packet->total_length = internal_packet.total_length;
        packet->channel = internal_packet.channel;
        packet->sequence = internal_packet.sequence;
        packet->cargo_length = internal_packet.cargo_length;
        memcpy(
            packet->cargo,
            internal_packet.cargo,
            internal_packet.cargo_length
        );
    }

    return result;
}

int bno086_test_feed_packet(
    bno086_t *imu,
    uint8_t channel,
    uint8_t sequence,
    const uint8_t *cargo,
    size_t cargo_length
)
{
    bno086_impl_t *impl = get_impl(imu);
    shtp_packet_t packet;

    if (impl == NULL
        || (cargo == NULL && cargo_length > 0u)
        || cargo_length > sizeof(packet.cargo))
    {
        return -1;
    }

    memset(&packet, 0, sizeof(packet));
    packet.total_length = (uint16_t)(cargo_length + 4u);
    packet.channel = channel;
    packet.sequence = sequence;
    packet.cargo_length = cargo_length;
    if (cargo_length > 0u)
    {
        memcpy(packet.cargo, cargo, cargo_length);
    }

    return process_packet(impl, &packet);
}

#endif
