#define _POSIX_C_SOURCE 200809L

#include "paa5100je.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/*
 * 이 파일은 SPI 전송과 PAA5100JE register protocol만 담당한다. SQUAL에 따른
 * sample 폐기, body/world 좌표 변환과 위치 적분은 position_fusion.c에서 한다.
 * 이렇게 경계를 나누면 센서가 없는 환경에서도 transfer layout은 mock으로,
 * 좌표 계산은 순수 입력값으로 각각 독립 회귀시험할 수 있다.
 *
 * 이 센서에서 가장 중요한 구현 조건은 "주소를 보낸 뒤 센서가 값을 준비할
 * 시간을 주되 CS는 계속 LOW"라는 점이다. 단순히 호출을 두 번 하면 호출
 * 사이에 CS가 HIGH가 될 수 있으므로 Linux에 두 segment를 한 message로
 * 제출하고 kernel이 하나의 연속 transaction으로 실행하게 한다.
 */

/*
 * ==================================================
 * PAA5100JE Registers
 * ==================================================
 */
#define REG_CHIP_ID          0x00u
#define REG_REVISION         0x01u
#define REG_MOTION           0x02u
#define REG_DELTA_X_L        0x03u
#define REG_DELTA_X_H        0x04u
#define REG_DELTA_Y_L        0x05u
#define REG_DELTA_Y_H        0x06u
#define REG_MOTION_BURST     0x16u
#define REG_POWER_UP_RESET   0x3Au
#define REG_CHIP_ID_INVERSE  0x5Fu

#define POWER_UP_RESET_VALUE 0x5Au

#define MOTION_BURST_LENGTH  12u
#define MOTION_OCCURRED_MASK 0x80u

#define REGISTER_WRITE_SETTLE_US 100u
#define REGISTER_READ_SETTLE_US  50u


/*
 * 초기화 sequence에는 실제 register write와 millisecond delay가
 * 함께 들어간다.
 *
 * reg가 INIT_DELAY_MARKER이면 value를 millisecond 단위 delay로
 * 해석하고, 그 외에는 일반 register/value pair로 해석한다.
 */
#define INIT_DELAY_MARKER (-1)

typedef struct
{
    int16_t reg;
    uint8_t value;

} init_operation_t;


/*
 * ==================================================
 * Default Linux I/O Backend
 * ==================================================
 */
static int system_open_device(
    void *context,
    const char *path,
    int flags
)
{
    (void) context;

    return open(path, flags);
}


static int system_close_device(
    void *context,
    int file_descriptor
)
{
    (void) context;

    return close(file_descriptor);
}


static int system_ioctl_device(
    void *context,
    int file_descriptor,
    unsigned long request,
    void *argument
)
{
    (void) context;

    return ioctl(file_descriptor, request, argument);
}


static int system_sleep_microseconds(
    void *context,
    uint32_t microseconds
)
{
    struct timespec request;
    struct timespec remaining;

    (void) context;

    request.tv_sec = (time_t) (microseconds / 1000000u);
    request.tv_nsec = (long) (microseconds % 1000000u) * 1000L;

    while (nanosleep(&request, &remaining) != 0)
    {
        if (errno != EINTR)
        {
            return -1;
        }

        request = remaining;
    }

    return 0;
}


static const paa5100je_io_ops_t default_io_ops =
{
    .open_device = system_open_device,
    .close_device = system_close_device,
    .ioctl_device = system_ioctl_device,
    .sleep_microseconds = system_sleep_microseconds
};


/*
 * ==================================================
 * Internal Validation Helpers
 * ==================================================
 */
static int validate_open_sensor(
    const paa5100je_t *sensor
)
{
    if (sensor == NULL ||
        sensor->is_open == 0 ||
        sensor->spi_fd < 0 ||
        sensor->io_ops.ioctl_device == NULL)
    {
        errno = EBADF;
        return -1;
    }

    return 0;
}


static int delay_microseconds(
    paa5100je_t *sensor,
    uint32_t microseconds
)
{
    if (sensor == NULL ||
        sensor->io_ops.sleep_microseconds == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    return sensor->io_ops.sleep_microseconds(
        sensor->io_context,
        microseconds
    );
}


/*
 * ==================================================
 * Segmented SPI Transfer
 * ==================================================
 *
 * address와 data를 독립된 SPI transaction으로 보내면
 * hardware chip-select가 둘 사이에서 HIGH로 돌아갈 수 있다.
 *
 * 따라서 두 phase를 struct spi_ioc_transfer 배열 하나에 넣어
 * SPI_IOC_MESSAGE(2) 한 번으로 kernel에 전달한다.
 *
 * 첫 번째 segment의 delay_usecs는 address 전송이 끝난 뒤에도
 * chip-select를 유지한 상태로 turnaround delay를 만든다.
 * 두 segment 모두 cs_change=0이므로 data phase까지 같은
 * chip-select assertion 안에서 수행된다.
 * 독립된 xfer/ioctl 두 번으로 쪼개면 중간에 CS가 올라갈 수 있으므로 이 함수의
 * 한-message 구조는 단순 성능 최적화가 아니라 센서 timing 요구사항이다.
 *
 * 그림으로 보면 다음과 같다.
 *
 *     CS LOW -> address 전송 -> delay_us 동안 LOW 유지 -> data 송수신 -> CS HIGH
 *
 * 함수가 성공하려면 kernel 반환 byte 수도 address+data 전체 길이와 같아야 한다.
 */
static int transfer_with_turnaround(
    paa5100je_t *sensor,
    const uint8_t *command,
    size_t command_length,
    const uint8_t *transmit_data,
    uint8_t *receive_data,
    size_t data_length,
    uint16_t delay_us
)
{
    struct spi_ioc_transfer transfers[2];
    size_t expected_length;
    int transferred;

    if (validate_open_sensor(sensor) != 0)
    {
        return -1;
    }

    if (command == NULL ||
        command_length == 0u ||
        transmit_data == NULL ||
        data_length == 0u ||
        command_length > UINT32_MAX ||
        data_length > UINT32_MAX)
    {
        errno = EINVAL;
        return -1;
    }

    expected_length = command_length + data_length;

    if (expected_length > (size_t) INT32_MAX)
    {
        errno = EOVERFLOW;
        return -1;
    }

    memset(transfers, 0, sizeof(transfers));

    transfers[0].tx_buf = (uintptr_t) command;
    transfers[0].len = (uint32_t) command_length;
    transfers[0].speed_hz = sensor->speed_hz;
    transfers[0].delay_usecs = delay_us;
    transfers[0].bits_per_word = 8u;
    transfers[0].cs_change = 0u;

    transfers[1].tx_buf = (uintptr_t) transmit_data;
    transfers[1].rx_buf = (uintptr_t) receive_data;
    transfers[1].len = (uint32_t) data_length;
    transfers[1].speed_hz = sensor->speed_hz;
    transfers[1].bits_per_word = 8u;
    transfers[1].cs_change = 0u;

    transferred = sensor->io_ops.ioctl_device(
        sensor->io_context,
        sensor->spi_fd,
        SPI_IOC_MESSAGE(2),
        transfers
    );

    if (transferred < 0)
    {
        return -1;
    }

    if ((size_t) transferred != expected_length)
    {
        errno = EIO;
        return -1;
    }

    return 0;
}


/*
 * ==================================================
 * Public Register Access
 * ==================================================
 */
int paa5100je_register_write(
    paa5100je_t *sensor,
    uint8_t reg,
    uint8_t value
)
{
    uint8_t command;

    /* Register address의 MSB=1은 PAA5100JE write cycle을 뜻한다. */
    command = (uint8_t) (reg | 0x80u);

    if (transfer_with_turnaround(
            sensor,
            &command,
            1u,
            &value,
            NULL,
            1u,
            PAA5100JE_REGISTER_WRITE_DELAY_US
        ) != 0)
    {
        return -1;
    }

    return delay_microseconds(
        sensor,
        REGISTER_WRITE_SETTLE_US
    );
}


int paa5100je_register_read(
    paa5100je_t *sensor,
    uint8_t reg,
    uint8_t *value
)
{
    uint8_t command;
    uint8_t dummy = 0u;
    uint8_t result = 0u;

    if (value == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    /* MSB=0으로 address를 보내고 500 us 뒤 dummy byte로 값을 clock-out한다. */
    command = (uint8_t) (reg & 0x7Fu);

    if (transfer_with_turnaround(
            sensor,
            &command,
            1u,
            &dummy,
            &result,
            1u,
            PAA5100JE_REGISTER_READ_DELAY_US
        ) != 0)
    {
        return -1;
    }

    if (delay_microseconds(
            sensor,
            REGISTER_READ_SETTLE_US
        ) != 0)
    {
        return -1;
    }

    *value = result;

    return 0;
}


/*
 * 여러 register/value pair와 delay marker를 순서대로 적용한다.
 * 중간 실패가 발생하면 errno를 유지한 채 즉시 호출자에게 반환한다.
 */
static int apply_init_operations(
    paa5100je_t *sensor,
    const init_operation_t *operations,
    size_t operation_count
)
{
    size_t i;

    /* 배열 순서가 곧 센서 설정 절차이므로 성공한 단계만 건너뛰는 동작은 없다. */
    for (i = 0u; i < operation_count; i++)
    {
        if (operations[i].reg == INIT_DELAY_MARKER)
        {
            if (delay_microseconds(
                    sensor,
                    (uint32_t) operations[i].value * 1000u
                ) != 0)
            {
                return -1;
            }

            continue;
        }

        if (operations[i].reg < 0 || operations[i].reg > UINT8_MAX)
        {
            errno = EINVAL;
            return -1;
        }

        if (paa5100je_register_write(
                sensor,
                (uint8_t) operations[i].reg,
                operations[i].value
            ) != 0)
        {
            return -1;
        }
    }

    return 0;
}


/*
 * ==================================================
 * PAA5100JE Initialization Sequence
 * ==================================================
 *
 * 기존 Python PAA5100JE 구현에서 사용하고 검증한
 * PixArt register sequence와 동적 trimming 절차를 그대로 옮긴다.
 * 0x7F write는 register bank 전환이므로 순서를 재배열하면 같은 주소라도 전혀
 * 다른 register를 건드린다. table과 delay marker의 순서를 그대로 보존한다.
 * 각 숫자의 의미를 임의로 추정해 이름을 붙이기보다 검증된 제조사 sequence를
 * 그대로 유지하고, 동적으로 읽어야 하는 trimming 값만 코드로 계산한다.
 */
static int initialize_paa5100je_registers(
    paa5100je_t *sensor
)
{
    static const init_operation_t preamble[] =
    {
        { 0x7F, 0x00 },
        { 0x55, 0x01 },
        { 0x50, 0x07 },
        { 0x7F, 0x0E },
        { 0x43, 0x10 }
    };

    static const init_operation_t trimming_setup[] =
    {
        { 0x7F, 0x00 },
        { 0x51, 0x7B },
        { 0x50, 0x00 },
        { 0x55, 0x00 },
        { 0x7F, 0x0E }
    };

    static const init_operation_t main_sequence[] =
    {
        { 0x7F, 0x00 },
        { 0x61, 0xAD },
        { 0x7F, 0x03 },
        { 0x40, 0x00 },
        { 0x7F, 0x05 },
        { 0x41, 0xB3 },
        { 0x43, 0xF1 },
        { 0x45, 0x14 },
        { 0x5F, 0x34 },
        { 0x7B, 0x08 },
        { 0x5E, 0x34 },
        { 0x5B, 0x11 },
        { 0x6D, 0x11 },
        { 0x45, 0x17 },
        { 0x70, 0xE5 },
        { 0x71, 0xE5 },
        { 0x7F, 0x06 },
        { 0x44, 0x1B },
        { 0x40, 0xBF },
        { 0x4E, 0x3F },
        { 0x7F, 0x08 },
        { 0x66, 0x44 },
        { 0x65, 0x20 },
        { 0x6A, 0x3A },
        { 0x61, 0x05 },
        { 0x62, 0x05 },
        { 0x7F, 0x09 },
        { 0x4F, 0xAF },
        { 0x5F, 0x40 },
        { 0x48, 0x80 },
        { 0x49, 0x80 },
        { 0x57, 0x77 },
        { 0x60, 0x78 },
        { 0x61, 0x78 },
        { 0x62, 0x08 },
        { 0x63, 0x50 },
        { 0x7F, 0x0A },
        { 0x45, 0x60 },
        { 0x7F, 0x00 },
        { 0x4D, 0x11 },
        { 0x55, 0x80 },
        { 0x74, 0x21 },
        { 0x75, 0x1F },
        { 0x4A, 0x78 },
        { 0x4B, 0x78 },
        { 0x44, 0x08 },
        { 0x45, 0x50 },
        { 0x64, 0xFF },
        { 0x65, 0x1F },
        { 0x7F, 0x14 },
        { 0x65, 0x67 },
        { 0x66, 0x08 },
        { 0x63, 0x70 },
        { 0x6F, 0x1C },
        { 0x7F, 0x15 },
        { 0x48, 0x48 },
        { 0x7F, 0x07 },
        { 0x41, 0x0D },
        { 0x43, 0x14 },
        { 0x4B, 0x0E },
        { 0x45, 0x0F },
        { 0x44, 0x42 },
        { 0x4C, 0x80 },
        { 0x7F, 0x10 },
        { 0x5B, 0x02 },
        { 0x7F, 0x07 },
        { 0x40, 0x41 },
        { INIT_DELAY_MARKER, 10 },
        { 0x7F, 0x00 },
        { 0x32, 0x00 },
        { 0x7F, 0x07 },
        { 0x40, 0x40 },
        { 0x7F, 0x06 },
        { 0x68, 0xF0 },
        { 0x69, 0x00 },
        { 0x7F, 0x0D },
        { 0x48, 0xC0 },
        { 0x6F, 0xD5 },
        { 0x7F, 0x00 },
        { 0x5B, 0xA0 },
        { 0x4E, 0xA8 },
        { 0x5A, 0x90 },
        { 0x40, 0x80 },
        { 0x73, 0x1F },
        { INIT_DELAY_MARKER, 10 },
        { 0x73, 0x00 }
    };

    uint8_t calibration_1;
    uint8_t calibration_2;
    uint8_t dynamic_status;
    uint16_t adjusted_calibration_1;

    if (apply_init_operations(
            sensor,
            preamble,
            sizeof(preamble) / sizeof(preamble[0])
        ) != 0)
    {
        return -1;
    }

    if (paa5100je_register_read(sensor, 0x67u, &dynamic_status) != 0)
    {
        return -1;
    }

    if (paa5100je_register_write(
            sensor,
            0x48u,
            (dynamic_status & 0x80u) != 0u ? 0x04u : 0x02u
        ) != 0)
    {
        return -1;
    }

    if (apply_init_operations(
            sensor,
            trimming_setup,
            sizeof(trimming_setup) / sizeof(trimming_setup[0])
        ) != 0)
    {
        return -1;
    }

    if (paa5100je_register_read(sensor, 0x73u, &dynamic_status) != 0)
    {
        return -1;
    }

    if (dynamic_status == 0x00u)
    {
        /* Sensor 내부 OTP 측정값에 datasheet 계수를 적용해 trimming을 확정한다. */
        if (paa5100je_register_read(sensor, 0x70u, &calibration_1) != 0 ||
            paa5100je_register_read(sensor, 0x71u, &calibration_2) != 0)
        {
            return -1;
        }

        if (calibration_1 <= 28u)
        {
            adjusted_calibration_1 =
                (uint16_t) calibration_1 + 14u;
        }
        else
        {
            adjusted_calibration_1 =
                (uint16_t) calibration_1 + 11u;
        }

        if (adjusted_calibration_1 > 0x3Fu)
        {
            adjusted_calibration_1 = 0x3Fu;
        }

        calibration_1 = (uint8_t) adjusted_calibration_1;

        calibration_2 = (uint8_t) (
            ((uint16_t) calibration_2 * 45u) / 100u
        );

        if (paa5100je_register_write(sensor, 0x7Fu, 0x00u) != 0 ||
            paa5100je_register_write(sensor, 0x61u, 0xADu) != 0 ||
            paa5100je_register_write(sensor, 0x51u, 0x70u) != 0 ||
            paa5100je_register_write(sensor, 0x7Fu, 0x0Eu) != 0 ||
            paa5100je_register_write(sensor, 0x70u, calibration_1) != 0 ||
            paa5100je_register_write(sensor, 0x71u, calibration_2) != 0)
        {
            return -1;
        }
    }

    return apply_init_operations(
        sensor,
        main_sequence,
        sizeof(main_sequence) / sizeof(main_sequence[0])
    );
}


/*
 * ==================================================
 * Device Open / Setup
 * ==================================================
 */
int paa5100je_open_with_ops(
    paa5100je_t *sensor,
    const char *device_path,
    uint32_t speed_hz,
    double height_cm,
    double scaler,
    const paa5100je_io_ops_t *io_ops,
    void *io_context
)
{
    uint8_t mode = SPI_MODE_3;
    uint8_t bits_per_word = 8u;
    int saved_errno;

    if (sensor == NULL ||
        device_path == NULL ||
        device_path[0] == '\0' ||
        speed_hz == 0u ||
        !isfinite(height_cm) ||
        height_cm <= 0.0f ||
        !isfinite(scaler) ||
        scaler <= 0.0f)
    {
        errno = EINVAL;
        return -1;
    }

    /* 실패 cleanup도 안전하도록 fd=-1인 명확한 미개방 상태에서 시작한다. */
    memset(sensor, 0, sizeof(*sensor));
    sensor->spi_fd = -1;

    sensor->io_ops = io_ops != NULL ? *io_ops : default_io_ops;
    sensor->io_context = io_context;

    if (sensor->io_ops.open_device == NULL ||
        sensor->io_ops.close_device == NULL ||
        sensor->io_ops.ioctl_device == NULL ||
        sensor->io_ops.sleep_microseconds == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    sensor->spi_fd = sensor->io_ops.open_device(
        sensor->io_context,
        device_path,
        O_RDWR | O_CLOEXEC
    );

    if (sensor->spi_fd < 0)
    {
        return -1;
    }

    sensor->is_open = 1;
    sensor->speed_hz = speed_hz;
    sensor->height_cm = height_cm;
    sensor->scaler = scaler;

    /* PAA5100JE가 요구하는 mode 3, 8-bit word와 지정 clock을 fd에 고정한다. */
    if (sensor->io_ops.ioctl_device(
            sensor->io_context,
            sensor->spi_fd,
            SPI_IOC_WR_MODE,
            &mode
        ) != 0 ||
        sensor->io_ops.ioctl_device(
            sensor->io_context,
            sensor->spi_fd,
            SPI_IOC_WR_BITS_PER_WORD,
            &bits_per_word
        ) != 0 ||
        sensor->io_ops.ioctl_device(
            sensor->io_context,
            sensor->spi_fd,
            SPI_IOC_WR_MAX_SPEED_HZ,
            &sensor->speed_hz
        ) != 0)
    {
        saved_errno = errno;
        sensor->io_ops.close_device(
            sensor->io_context,
            sensor->spi_fd
        );
        sensor->spi_fd = -1;
        sensor->is_open = 0;
        errno = saved_errno;

        return -1;
    }

    return 0;
}


int paa5100je_open(
    paa5100je_t *sensor,
    unsigned int spi_bus,
    unsigned int chip_select,
    uint32_t speed_hz,
    double height_cm,
    double scaler
)
{
    char device_path[64];
    int path_length;

    path_length = snprintf(
        device_path,
        sizeof(device_path),
        "/dev/spidev%u.%u",
        spi_bus,
        chip_select
    );

    if (path_length < 0 ||
        (size_t) path_length >= sizeof(device_path))
    {
        errno = ENAMETOOLONG;
        return -1;
    }

    return paa5100je_open_with_ops(
        sensor,
        device_path,
        speed_hz,
        height_cm,
        scaler,
        NULL,
        NULL
    );
}


int paa5100je_begin(
    paa5100je_t *sensor
)
{
    uint8_t discarded;
    uint8_t reg;

    if (validate_open_sensor(sensor) != 0)
    {
        return -1;
    }

    sensor->initialized = 0;

    /*
     * Python 구현의 power-up CS settle 구간과 같은 3 ms를 둔다.
     * hardware CS는 실제 SPI message 동안 kernel이 제어한다.
     */
    if (delay_microseconds(sensor, 1000u) != 0 ||
        delay_microseconds(sensor, 1000u) != 0 ||
        delay_microseconds(sensor, 1000u) != 0)
    {
        return -1;
    }

    if (paa5100je_register_write(
            sensor,
            REG_POWER_UP_RESET,
            POWER_UP_RESET_VALUE
        ) != 0 ||
        delay_microseconds(sensor, 50000u) != 0)
    {
        return -1;
    }

    if (paa5100je_register_read(
            sensor,
            REG_CHIP_ID,
            &sensor->chip_id
        ) != 0 ||
        paa5100je_register_read(
            sensor,
            REG_REVISION,
            &sensor->revision
        ) != 0 ||
        paa5100je_register_read(
            sensor,
            REG_CHIP_ID_INVERSE,
            &sensor->inverse_chip_id
        ) != 0)
    {
        return -1;
    }

    /* 두 ID가 상보 관계여야 다른 SPI 장치나 floating MISO를 통과시키지 않는다. */
    if (sensor->chip_id != PAA5100JE_CHIP_ID ||
        sensor->inverse_chip_id != PAA5100JE_CHIP_ID_INVERSE)
    {
        errno = ENODEV;
        return -1;
    }

    /*
     * Power-up 이후 남아 있을 수 있는 motion/delta register를
     * Python 기준 구현과 동일하게 한 번씩 읽어서 비운다.
     */
    for (reg = REG_MOTION; reg <= REG_DELTA_Y_H; reg++)
    {
        if (paa5100je_register_read(sensor, reg, &discarded) != 0)
        {
            return -1;
        }
    }

    if (delay_microseconds(sensor, 1000u) != 0 ||
        initialize_paa5100je_registers(sensor) != 0)
    {
        return -1;
    }

    /* 이 줄까지 모든 단계가 성공해야 read_motion이 허용된다. */
    sensor->initialized = 1;

    return 0;
}


/*
 * ==================================================
 * Motion Burst Decode
 * ==================================================
 *
 * burst byte 2..5의 dx/dy는 16-bit little-endian two's-complement다.
 * 직접 부호 확장하여 host endian과 int16_t 표현에 의존하지 않는다.
 * 양수/음수 부호는 장착 방향에 대한 센서축 부호이며 testbed 좌표축 보정은
 * driver가 아니라 상위 위치 융합에서 결정한다.
 */
static int16_t decode_signed_little_endian(
    const uint8_t *bytes
)
{
    uint16_t raw;
    int32_t signed_value;

    raw = (uint16_t) bytes[0] |
          ((uint16_t) bytes[1] << 8u);

    signed_value = raw <= INT16_MAX
        ? (int32_t) raw
        : (int32_t) raw - 65536;

    return (int16_t) signed_value;
}


int paa5100je_read_motion(
    paa5100je_t *sensor,
    paa5100je_motion_t *motion
)
{
    uint8_t command = REG_MOTION_BURST;
    uint8_t dummy[MOTION_BURST_LENGTH];
    uint8_t data[MOTION_BURST_LENGTH];
    int16_t delta_x;
    int16_t delta_y;

    if (sensor == NULL ||
        motion == NULL ||
        sensor->initialized == 0)
    {
        errno = EINVAL;
        return -1;
    }

    memset(dummy, 0, sizeof(dummy));
    memset(data, 0, sizeof(data));

    if (transfer_with_turnaround(
            sensor,
            &command,
            1u,
            dummy,
            data,
            sizeof(data),
            PAA5100JE_MOTION_BURST_DELAY_US
        ) != 0)
    {
        return -1;
    }

    /* 모든 필드는 같은 CS assertion에서 latch된 하나의 motion sample이다. */
    delta_x = decode_signed_little_endian(&data[2]);
    delta_y = decode_signed_little_endian(&data[4]);

    memset(motion, 0, sizeof(*motion));

    motion->motion_detected = (data[0] & MOTION_OCCURRED_MASK) != 0u;
    motion->observation = data[1];
    motion->squal = data[6];
    motion->raw_sum = data[7];
    motion->raw_max = data[8];
    motion->raw_min = data[9];
    motion->shutter = (uint16_t) (
        ((uint16_t) data[10] << 8u) |
        (uint16_t) data[11]
    );

    /* motion bit가 없으면 오래된 delta register가 위치에 재사용되지 않게 0을 유지한다. */
    if (motion->motion_detected != 0)
    {
        motion->delta_x_ticks = delta_x;
        motion->delta_y_ticks = delta_y;
    }

    /* 같은 raw tick이라도 센서 높이가 높을수록 바닥에서 더 긴 이동에 해당한다. */
    motion->delta_x_cm =
        ((double) motion->delta_x_ticks * sensor->height_cm) /
        sensor->scaler;

    motion->delta_y_cm =
        ((double) motion->delta_y_ticks * sensor->height_cm) /
        sensor->scaler;

    sensor->last_squal = motion->squal;
    sensor->last_shutter = motion->shutter;

    return 0;
}


int paa5100je_read_motion_count(
    paa5100je_t *sensor,
    int16_t *delta_x_ticks,
    int16_t *delta_y_ticks
)
{
    paa5100je_motion_t motion;

    if (delta_x_ticks == NULL || delta_y_ticks == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    if (paa5100je_read_motion(sensor, &motion) != 0)
    {
        return -1;
    }

    /* 호출자가 거리/진단 필드가 필요 없을 때도 내부적으로 동일 burst 경로를 쓴다. */
    *delta_x_ticks = motion.delta_x_ticks;
    *delta_y_ticks = motion.delta_y_ticks;

    return 0;
}


int paa5100je_read_motion_distance_cm(
    paa5100je_t *sensor,
    double *delta_x_cm,
    double *delta_y_cm
)
{
    paa5100je_motion_t motion;

    if (delta_x_cm == NULL || delta_y_cm == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    if (paa5100je_read_motion(sensor, &motion) != 0)
    {
        return -1;
    }

    *delta_x_cm = motion.delta_x_cm;
    *delta_y_cm = motion.delta_y_cm;

    return 0;
}


/*
 * ==================================================
 * Runtime Configuration
 * ==================================================
 *
 * height와 scaler는 tick->cm 환산 계수만 바꾼다. PAA 연결 후 실제 높이,
 * 정지 noise와 왕복 이동으로 확정하기 전에는 물리 거리 정확도가 검증된 값이 아니다.
 */
int paa5100je_set_height_cm(
    paa5100je_t *sensor,
    double height_cm
)
{
    if (sensor == NULL ||
        !isfinite(height_cm) ||
        height_cm <= 0.0f)
    {
        errno = EINVAL;
        return -1;
    }

    sensor->height_cm = height_cm;

    return 0;
}


int paa5100je_set_scaler(
    paa5100je_t *sensor,
    double scaler
)
{
    if (sensor == NULL ||
        !isfinite(scaler) ||
        scaler <= 0.0f)
    {
        errno = EINVAL;
        return -1;
    }

    sensor->scaler = scaler;

    return 0;
}


int paa5100je_height_in_focus_range(
    double height_cm
)
{
    return isfinite(height_cm) &&
           height_cm >= PAA5100JE_FOCUS_MIN_CM &&
           height_cm <= PAA5100JE_FOCUS_MAX_CM;
}


int paa5100je_set_led(
    paa5100je_t *sensor,
    int led_on
)
{
    /* bank 0x14에서 LED 값을 바꾼 뒤 항상 기본 bank 0으로 되돌린다. */
    if (delay_microseconds(sensor, 50000u) != 0 ||
        paa5100je_register_write(sensor, 0x7Fu, 0x14u) != 0 ||
        paa5100je_register_write(
            sensor,
            0x6Fu,
            led_on != 0 ? 0x1Cu : 0x00u
        ) != 0 ||
        paa5100je_register_write(sensor, 0x7Fu, 0x00u) != 0)
    {
        return -1;
    }

    return 0;
}


int paa5100je_close(
    paa5100je_t *sensor
)
{
    int result;

    if (sensor == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    /* 여러 cleanup 경로에서 호출할 수 있도록 이미 닫힌 객체도 성공 처리한다. */
    if (sensor->is_open == 0 || sensor->spi_fd < 0)
    {
        sensor->spi_fd = -1;
        sensor->is_open = 0;
        sensor->initialized = 0;

        return 0;
    }

    result = sensor->io_ops.close_device(
        sensor->io_context,
        sensor->spi_fd
    );

    sensor->spi_fd = -1;
    sensor->is_open = 0;
    sensor->initialized = 0;

    return result;
}
