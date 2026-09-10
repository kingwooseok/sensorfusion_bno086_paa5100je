#define _POSIX_C_SOURCE 200809L

#include "../paa5100je.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/*
 * ==================================================
 * PAA5100JE C Driver Mock Backend
 * ==================================================
 *
 * 실제 /dev/spidev 장치를 열지 않고 driver가 kernel에 전달할
 * SPI message를 메모리에서 검사한다.
 *
 * 이 시험은 protocol 구성과 C 변환을 검증하지만
 * 실제 전기적 timing, SQUAL 및 motion 정확도를 보장하지 않는다.
 */
#define MOCK_FILE_DESCRIPTOR 73
#define MAX_WRITES           256u

typedef struct
{
    unsigned int bank;

    uint8_t chip_id;
    uint8_t inverse_chip_id;
    uint8_t revision;

    uint8_t burst[12];

    unsigned int open_count;
    unsigned int close_count;
    unsigned int config_count;
    unsigned int message_count;
    unsigned int sleep_count;

    uint64_t slept_microseconds;

    int short_transfer_once;
    int validation_failed;

    struct
    {
        uint8_t bank;
        uint8_t reg;
        uint8_t value;

    } writes[MAX_WRITES];

    size_t write_count;

} mock_context_t;


static int mock_open_device(
    void *opaque,
    const char *path,
    int flags
)
{
    mock_context_t *context = opaque;

    if (strcmp(path, "/dev/mock-paa5100je") != 0 ||
        (flags & O_RDWR) == 0)
    {
        context->validation_failed = 1;
        errno = EINVAL;
        return -1;
    }

    context->open_count++;

    return MOCK_FILE_DESCRIPTOR;
}


static int mock_close_device(
    void *opaque,
    int file_descriptor
)
{
    mock_context_t *context = opaque;

    if (file_descriptor != MOCK_FILE_DESCRIPTOR)
    {
        context->validation_failed = 1;
        errno = EBADF;
        return -1;
    }

    context->close_count++;

    return 0;
}


static int mock_sleep_microseconds(
    void *opaque,
    uint32_t microseconds
)
{
    mock_context_t *context = opaque;

    context->sleep_count++;
    context->slept_microseconds += microseconds;

    return 0;
}


static void set_little_endian_int16(
    uint8_t *destination,
    int16_t value
)
{
    uint16_t raw = (uint16_t) value;

    destination[0] = (uint8_t) (raw & 0xFFu);
    destination[1] = (uint8_t) (raw >> 8u);
}


static uint8_t mock_register_value(
    const mock_context_t *context,
    uint8_t reg
)
{
    if (context->bank == 0u)
    {
        if (reg == 0x00u)
        {
            return context->chip_id;
        }

        if (reg == 0x01u)
        {
            return context->revision;
        }

        if (reg == 0x5Fu)
        {
            return context->inverse_chip_id;
        }
    }

    if (context->bank == 0x0Eu)
    {
        if (reg == 0x67u)
        {
            return 0x00u;
        }

        if (reg == 0x73u)
        {
            /*
             * 0을 반환하여 Python 구현의 dynamic trimming 경로를
             * 실제로 통과시킨다.
             */
            return 0x00u;
        }

        if (reg == 0x70u)
        {
            return 20u;
        }

        if (reg == 0x71u)
        {
            return 200u;
        }
    }

    return 0u;
}


static int validate_common_transfer(
    mock_context_t *context,
    const struct spi_ioc_transfer *transfers
)
{
    size_t i;

    if (transfers[0].len != 1u ||
        transfers[0].tx_buf == 0u ||
        transfers[0].rx_buf != 0u ||
        transfers[1].len == 0u ||
        transfers[1].tx_buf == 0u)
    {
        context->validation_failed = 1;
        return -1;
    }

    for (i = 0u; i < 2u; i++)
    {
        if (transfers[i].speed_hz != 2000000u ||
            transfers[i].bits_per_word != 8u ||
            transfers[i].cs_change != 0u)
        {
            context->validation_failed = 1;
            return -1;
        }
    }

    if (transfers[1].delay_usecs != 0u)
    {
        context->validation_failed = 1;
        return -1;
    }

    return 0;
}


static int mock_spi_message(
    mock_context_t *context,
    struct spi_ioc_transfer *transfers
)
{
    uint8_t command;
    uint8_t reg;
    uint8_t *receive_data;
    const uint8_t *transmit_data;
    size_t total_length;

    if (validate_common_transfer(context, transfers) != 0)
    {
        errno = EPROTO;
        return -1;
    }

    command = *(const uint8_t *) (uintptr_t) transfers[0].tx_buf;
    transmit_data =
        (const uint8_t *) (uintptr_t) transfers[1].tx_buf;
    receive_data =
        (uint8_t *) (uintptr_t) transfers[1].rx_buf;

    if (command == 0x16u && transfers[1].len == sizeof(context->burst))
    {
        if (transfers[0].delay_usecs !=
                PAA5100JE_MOTION_BURST_DELAY_US ||
            receive_data == NULL)
        {
            context->validation_failed = 1;
            errno = EPROTO;
            return -1;
        }

        memcpy(receive_data, context->burst, sizeof(context->burst));
    }
    else if ((command & 0x80u) != 0u)
    {
        reg = (uint8_t) (command & 0x7Fu);

        if (transfers[0].delay_usecs !=
                PAA5100JE_REGISTER_WRITE_DELAY_US ||
            transfers[1].len != 1u ||
            receive_data != NULL)
        {
            context->validation_failed = 1;
            errno = EPROTO;
            return -1;
        }

        if (context->write_count >= MAX_WRITES)
        {
            context->validation_failed = 1;
            errno = ENOSPC;
            return -1;
        }

        context->writes[context->write_count].bank =
            (uint8_t) context->bank;
        context->writes[context->write_count].reg = reg;
        context->writes[context->write_count].value = transmit_data[0];
        context->write_count++;

        if (reg == 0x7Fu)
        {
            context->bank = transmit_data[0];
        }
    }
    else
    {
        reg = command;

        if (transfers[0].delay_usecs !=
                PAA5100JE_REGISTER_READ_DELAY_US ||
            transfers[1].len != 1u ||
            receive_data == NULL ||
            transmit_data[0] != 0u)
        {
            context->validation_failed = 1;
            errno = EPROTO;
            return -1;
        }

        receive_data[0] = mock_register_value(context, reg);
    }

    context->message_count++;

    total_length =
        (size_t) transfers[0].len +
        (size_t) transfers[1].len;

    if (context->short_transfer_once != 0)
    {
        context->short_transfer_once = 0;

        return (int) total_length - 1;
    }

    return (int) total_length;
}


static int mock_ioctl_device(
    void *opaque,
    int file_descriptor,
    unsigned long request,
    void *argument
)
{
    mock_context_t *context = opaque;

    if (file_descriptor != MOCK_FILE_DESCRIPTOR)
    {
        context->validation_failed = 1;
        errno = EBADF;
        return -1;
    }

    if (request == SPI_IOC_WR_MODE)
    {
        if (*(const uint8_t *) argument != SPI_MODE_3)
        {
            context->validation_failed = 1;
        }

        context->config_count++;
        return 0;
    }

    if (request == SPI_IOC_WR_BITS_PER_WORD)
    {
        if (*(const uint8_t *) argument != 8u)
        {
            context->validation_failed = 1;
        }

        context->config_count++;
        return 0;
    }

    if (request == SPI_IOC_WR_MAX_SPEED_HZ)
    {
        if (*(const uint32_t *) argument != 2000000u)
        {
            context->validation_failed = 1;
        }

        context->config_count++;
        return 0;
    }

    if (request != SPI_IOC_MESSAGE(2))
    {
        context->validation_failed = 1;
        errno = EINVAL;
        return -1;
    }

    return mock_spi_message(
        context,
        argument
    );
}


static const paa5100je_io_ops_t mock_io_ops =
{
    .open_device = mock_open_device,
    .close_device = mock_close_device,
    .ioctl_device = mock_ioctl_device,
    .sleep_microseconds = mock_sleep_microseconds
};


static int find_write(
    const mock_context_t *context,
    uint8_t bank,
    uint8_t reg,
    uint8_t value
)
{
    size_t i;

    for (i = 0u; i < context->write_count; i++)
    {
        if (context->writes[i].bank == bank &&
            context->writes[i].reg == reg &&
            context->writes[i].value == value)
        {
            return 1;
        }
    }

    return 0;
}


/*
 * Python 기준 구현의 전체 write 순서를 작은 golden value로 고정한다.
 *
 * 각 write 직전 bank, register, value 세 byte를 FNV-1a로 누적한다.
 * 단순 write 개수만 확인할 때 놓칠 수 있는 register 순서나 값의
 * 회귀를 함께 검출하기 위한 값이다.
 */
static uint64_t hash_write_sequence(
    const mock_context_t *context
)
{
    uint64_t hash = UINT64_C(1469598103934665603);
    size_t i;

    for (i = 0u; i < context->write_count; i++)
    {
        hash ^= context->writes[i].bank;
        hash *= UINT64_C(1099511628211);

        hash ^= context->writes[i].reg;
        hash *= UINT64_C(1099511628211);

        hash ^= context->writes[i].value;
        hash *= UINT64_C(1099511628211);
    }

    return hash;
}


static void initialize_context(
    mock_context_t *context
)
{
    memset(context, 0, sizeof(*context));

    context->chip_id = PAA5100JE_CHIP_ID;
    context->inverse_chip_id = PAA5100JE_CHIP_ID_INVERSE;
    context->revision = 0x01u;
}


static int open_mock_sensor(
    paa5100je_t *sensor,
    mock_context_t *context
)
{
    return paa5100je_open_with_ops(
        sensor,
        "/dev/mock-paa5100je",
        PAA5100JE_DEFAULT_SPEED_HZ,
        PAA5100JE_DEFAULT_HEIGHT_CM,
        PAA5100JE_DEFAULT_SCALER,
        &mock_io_ops,
        context
    );
}


/*
 * Product ID, inverse Product ID, dynamic trimming 및 전체 init sequence를
 * 실제 system call 없이 확인한다.
 */
static int test_begin_and_identification(void)
{
    paa5100je_t sensor;
    mock_context_t context;

    initialize_context(&context);

    if (open_mock_sensor(&sensor, &context) != 0 ||
        paa5100je_begin(&sensor) != 0)
    {
        perror("begin mock sensor");
        return -1;
    }

    if (context.open_count != 1u ||
        context.config_count != 3u ||
        context.validation_failed != 0 ||
        sensor.chip_id != PAA5100JE_CHIP_ID ||
        sensor.inverse_chip_id != PAA5100JE_CHIP_ID_INVERSE ||
        sensor.revision != 0x01u ||
        sensor.initialized == 0)
    {
        fprintf(stderr, "begin/ID state mismatch\n");
        return -1;
    }

    /*
     * Mock c1=20, c2=200이므로 Python과 같은 trimming 결과는
     * 각각 34와 90이어야 한다.
     */
    if (!find_write(&context, 0x0Eu, 0x70u, 34u) ||
        !find_write(&context, 0x0Eu, 0x71u, 90u) ||
        context.write_count != 102u ||
        hash_write_sequence(&context) != UINT64_C(0x6770C7BADC9C5384))
    {
        fprintf(stderr, "dynamic trimming/init sequence mismatch\n");
        return -1;
    }

    if (paa5100je_close(&sensor) != 0 ||
        context.close_count != 1u)
    {
        fprintf(stderr, "mock close mismatch\n");
        return -1;
    }

    return 0;
}


/*
 * Motion Burst byte order, signed int16, SQUAL, shutter 및
 * height/scaler 거리 변환을 확인한다.
 */
static int test_motion_burst_decode(void)
{
    paa5100je_t sensor;
    paa5100je_motion_t motion;
    mock_context_t context;
    double expected_x;
    double expected_y;

    initialize_context(&context);

    context.burst[0] = 0x80u;
    context.burst[1] = 0xA5u;
    set_little_endian_int16(&context.burst[2], -1234);
    set_little_endian_int16(&context.burst[4], 2345);
    context.burst[6] = 77u;
    context.burst[7] = 1u;
    context.burst[8] = 2u;
    context.burst[9] = 3u;
    context.burst[10] = 0x12u;
    context.burst[11] = 0x34u;

    if (open_mock_sensor(&sensor, &context) != 0)
    {
        return -1;
    }

    /*
     * 이 test의 목적은 motion transport이므로 긴 begin sequence는
     * 앞 시험에서 검증하고 여기서는 initialized 상태만 설정한다.
     */
    sensor.initialized = 1;

    if (paa5100je_read_motion(&sensor, &motion) != 0)
    {
        perror("motion burst");
        return -1;
    }

    expected_x =
        (-1234.0 * PAA5100JE_DEFAULT_HEIGHT_CM) /
        PAA5100JE_DEFAULT_SCALER;

    expected_y =
        (2345.0 * PAA5100JE_DEFAULT_HEIGHT_CM) /
        PAA5100JE_DEFAULT_SCALER;

    if (motion.motion_detected == 0 ||
        motion.delta_x_ticks != -1234 ||
        motion.delta_y_ticks != 2345 ||
        fabs(motion.delta_x_cm - expected_x) > 0.000000001 ||
        fabs(motion.delta_y_cm - expected_y) > 0.000000001 ||
        motion.observation != 0xA5u ||
        motion.squal != 77u ||
        motion.raw_sum != 1u ||
        motion.raw_max != 2u ||
        motion.raw_min != 3u ||
        motion.shutter != 0x1234u ||
        sensor.last_squal != 77u ||
        sensor.last_shutter != 0x1234u ||
        context.validation_failed != 0)
    {
        fprintf(stderr, "motion burst decode mismatch\n");
        return -1;
    }

    context.burst[0] = 0x00u;

    if (paa5100je_read_motion(&sensor, &motion) != 0 ||
        motion.delta_x_ticks != 0 ||
        motion.delta_y_ticks != 0)
    {
        fprintf(stderr, "motion flag clear handling mismatch\n");
        return -1;
    }

    return paa5100je_close(&sensor);
}


/*
 * inverse ID가 다르면 정상 장치로 인정하지 않아야 한다.
 */
static int test_inverse_id_rejection(void)
{
    paa5100je_t sensor;
    mock_context_t context;

    initialize_context(&context);
    context.inverse_chip_id = 0x00u;

    if (open_mock_sensor(&sensor, &context) != 0)
    {
        return -1;
    }

    errno = 0;

    if (paa5100je_begin(&sensor) == 0 || errno != ENODEV)
    {
        fprintf(stderr, "inverse Product ID was accepted\n");
        return -1;
    }

    return paa5100je_close(&sensor);
}


/*
 * kernel이 요청한 byte보다 적게 전송했다고 보고하면
 * 정상 motion으로 받아들이지 않고 EIO로 분류해야 한다.
 */
static int test_short_spi_message(void)
{
    paa5100je_t sensor;
    mock_context_t context;
    uint8_t value;

    initialize_context(&context);

    if (open_mock_sensor(&sensor, &context) != 0)
    {
        return -1;
    }

    context.short_transfer_once = 1;
    errno = 0;

    if (paa5100je_register_read(&sensor, 0x00u, &value) == 0 ||
        errno != EIO)
    {
        fprintf(stderr, "short SPI message was accepted\n");
        return -1;
    }

    return paa5100je_close(&sensor);
}


typedef int (*test_function_t)(void);

typedef struct
{
    const char *name;
    test_function_t function;

} test_case_t;


int main(void)
{
    static const test_case_t tests[] =
    {
        { "begin / Product ID / init sequence", test_begin_and_identification },
        { "segmented motion burst / signed decode", test_motion_burst_decode },
        { "inverse Product ID rejection", test_inverse_id_rejection },
        { "short SPI message rejection", test_short_spi_message }
    };

    size_t i;
    unsigned int passed = 0u;

    printf("=== PAA5100JE C DRIVER MOCK TEST ===\n");

    for (i = 0u; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        if (tests[i].function() == 0)
        {
            printf("%s: PASS\n", tests[i].name);
            passed++;
        }
        else
        {
            printf("%s: FAIL\n", tests[i].name);
        }
    }

    printf("\npassed=%u/%zu\n", passed, sizeof(tests) / sizeof(tests[0]));
    printf(
        "CODE/MOCK RESULT=%s\n",
        passed == sizeof(tests) / sizeof(tests[0]) ? "PASS" : "FAIL"
    );
    printf("PAA5100JE HARDWARE RESULT=UNVERIFIED (sensor not connected)\n");

    return passed == sizeof(tests) / sizeof(tests[0]) ? 0 : 1;
}
