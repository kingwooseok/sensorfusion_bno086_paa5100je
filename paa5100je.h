#ifndef PAA5100JE_H
#define PAA5100JE_H

#include <stddef.h>
#include <stdint.h>

/*
 * ==================================================
 * PAA5100JE Optical Flow Sensor Driver
 * ==================================================
 *
 * Raspberry Pi의 Linux spidev를 통해 PAA5100JE를 제어한다.
 *
 * 이 드라이버의 SPI transaction은 다음 형태를 반드시 유지한다.
 *
 *     address phase
 *          -> turnaround delay
 *          -> data phase
 *
 * 두 phase는 하나의 SPI_IOC_MESSAGE(2)에 포함되므로
 * 중간에 hardware chip-select가 해제되지 않는다.
 *
 * PAA5100JE 장치는 하나의 Sensor I/O Thread만 소유해야 한다.
 * 이 구조체와 함수는 내부 lock을 제공하지 않으므로
 * 여러 Thread에서 동시에 호출해서는 안 된다.
 *
 * 일반적인 호출 순서는 open -> begin -> read_motion 반복 -> close다.
 * 모든 public 함수는 성공 시 0, 실패 시 -1을 반환하고 errno를 보존한다.
 * 장치가 없는 개발 단계에서는 open_with_ops로 동일 transaction을 mock한다.
 *
 * 센서는 바닥 무늬가 영상에서 얼마나 이동했는지를 tick으로 알려줄 뿐 RC카의
 * 절대 좌표를 직접 알지 못한다. 따라서 이 driver가 제공하는 값은 "직전 read
 * 이후 센서 아래에서 관측된 상대 이동"이고, position_fusion이 높이와 heading을
 * 사용해 cm 및 world 좌표로 바꾼다. 센서 높이와 scaler가 틀리면 방향은 맞아도
 * 이동 거리는 일정 비율로 틀릴 수 있다.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define PAA5100JE_CHIP_ID                 0x49u
#define PAA5100JE_CHIP_ID_INVERSE         0xB6u

#define PAA5100JE_DEFAULT_SPEED_HZ        2000000u
#define PAA5100JE_DEFAULT_HEIGHT_CM       3.5
#define PAA5100JE_DEFAULT_SCALER          400.8

#define PAA5100JE_FOCUS_MIN_CM            1.5
#define PAA5100JE_FOCUS_MAX_CM            3.5

#define PAA5100JE_REGISTER_WRITE_DELAY_US 50u
#define PAA5100JE_REGISTER_READ_DELAY_US  500u
#define PAA5100JE_MOTION_BURST_DELAY_US   50u


/*
 * ==================================================
 * Injectable Linux I/O Operations
 * ==================================================
 *
 * 실제 실행에서는 기본 system call 구현을 사용한다.
 *
 * PAA5100JE가 연결되지 않은 개발 환경에서는
 * 이 함수 포인터를 mock 구현으로 교체하여 다음 항목을 검증한다.
 *
 * - SPI_IOC_MESSAGE(2)의 두 segment 구성
 * - address/data 사이 delay
 * - chip-select 유지 조건
 * - Product ID / inverse Product ID 검증
 * - motion burst의 signed delta와 진단값 변환
 *
 * 이 추상화는 production protocol을 우회하기 위한 것이 아니다. mock도 실제와
 * 같은 한 번의 SPI_IOC_MESSAGE(2)를 받아 CS 유지와 segment 순서를 검사한다.
 */
typedef struct
{
    int (*open_device)(
        void *context,
        const char *path,
        int flags
    );

    int (*close_device)(
        void *context,
        int file_descriptor
    );

    int (*ioctl_device)(
        void *context,
        int file_descriptor,
        unsigned long request,
        void *argument
    );

    int (*sleep_microseconds)(
        void *context,
        uint32_t microseconds
    );

} paa5100je_io_ops_t;


/*
 * ==================================================
 * Motion Burst Result
 * ==================================================
 *
 * 0x16 Motion Burst에서 한 번에 latch된 값을 보관한다.
 *
 * motion_detected가 0이면 Python 기준 구현과 동일하게
 * delta_x_ticks와 delta_y_ticks를 0으로 반환한다.
 * 나머지 SQUAL, shutter 및 raw 진단값은 그대로 보존한다.
 * tick은 센서 body frame의 원시 누적량이고 cm 값은
 * ticks * height_cm / scaler로 변환한 동일 sample이다. SQUAL은 위치 융합이
 * 저품질 sample을 버리는 판단에 사용하지만 driver 자체는 이를 필터하지 않는다.
 * 예를 들어 delta_x_ticks=100, height=3.5 cm, scaler=400.8이면 약 0.873 cm다.
 * 이 수치는 환산 예일 뿐이며 실제 scaler는 PAA를 연결한 거리 시험으로 정한다.
 */
typedef struct
{
    /* motion bit: 새 이동이 검출됐는지 나타내며 0이면 dx/dy는 사용하지 않는다. */
    int motion_detected;

    int16_t delta_x_ticks;
    int16_t delta_y_ticks;

    double delta_x_cm;
    double delta_y_cm;

    /* 아래 값들은 표면/노출 상태를 이해하기 위한 진단값이며 이동 좌표가 아니다. */
    uint8_t observation;
    uint8_t squal;
    uint8_t raw_sum;
    uint8_t raw_max;
    uint8_t raw_min;
    uint16_t shutter;

} paa5100je_motion_t;


/*
 * ==================================================
 * PAA5100JE Device State
 * ==================================================
 *
 * 동적 메모리를 사용하지 않고 호출자가 이 구조체를 소유한다.
 * paa5100je_open() 또는 paa5100je_open_with_ops()가 성공한 뒤
 * paa5100je_begin()을 호출해야 motion을 읽을 수 있다.
 * chip/revision과 last_squal/shutter는 현장 진단용 관측값이며, height/scaler는
 * 이후 burst의 거리 환산에 즉시 적용되는 runtime geometry다.
 */
typedef struct
{
    int spi_fd;
    int is_open;
    int initialized;

    uint32_t speed_hz;
    double height_cm;
    double scaler;

    uint8_t chip_id;
    uint8_t inverse_chip_id;
    uint8_t revision;

    uint8_t last_squal;
    uint16_t last_shutter;

    paa5100je_io_ops_t io_ops;
    void *io_context;

} paa5100je_t;


/*
 * 실제 /dev/spidev<bus>.<chip_select> 장치를 연다.
 *
 * 성공: 0
 * 실패: -1, errno 유지
 */
int paa5100je_open(
    paa5100je_t *sensor,
    unsigned int spi_bus,
    unsigned int chip_select,
    uint32_t speed_hz,
    double height_cm,
    double scaler
);


/*
 * mock 또는 대체 I/O backend를 사용하여 장치를 연다.
 *
 * 실제 Sensor Process에서는 paa5100je_open()을 사용하고,
 * 정적/모의 회귀시험에서만 이 함수를 사용한다.
 * 함수 포인터를 전달하지 않으면 실제 Linux backend를 사용한다.
 */
int paa5100je_open_with_ops(
    paa5100je_t *sensor,
    const char *device_path,
    uint32_t speed_hz,
    double height_cm,
    double scaler,
    const paa5100je_io_ops_t *io_ops,
    void *io_context
);


/*
 * Power-on reset, Product ID 0x49와 inverse ID 0xB6 검증, power-on delta
 * flush 및 PAA5100JE 전용 초기화 register sequence를 수행한다. 성공한 뒤에만
 * initialized가 설정되므로 실패한 부분 초기화 상태에서는 motion을 읽지 않는다.
 */
int paa5100je_begin(
    paa5100je_t *sensor
);


/*
 * 한 번의 atomic motion burst를 읽는다. 출력 구조체는 SPI read와 decode가
 * 모두 성공한 뒤 채워지므로 실패한 호출을 새 sample로 사용하면 안 된다.
 */
int paa5100je_read_motion(
    paa5100je_t *sensor,
    paa5100je_motion_t *motion
);


/*
 * Python read_motion_count()와 같은 간단한 인터페이스다.
 */
int paa5100je_read_motion_count(
    paa5100je_t *sensor,
    int16_t *delta_x_ticks,
    int16_t *delta_y_ticks
);


/*
 * 저장된 height/scaler를 이용하여 cm 단위 이동량을 반환한다.
 */
int paa5100je_read_motion_distance_cm(
    paa5100je_t *sensor,
    double *delta_x_cm,
    double *delta_y_cm
);


/*
 * calibration 과정에서 높이와 scaler를 갱신한다.
 * 양수이고 유한한 값만 허용하며 sensor register를 변경하지는 않는다.
 */
int paa5100je_set_height_cm(
    paa5100je_t *sensor,
    double height_cm
);

int paa5100je_set_scaler(
    paa5100je_t *sensor,
    double scaler
);


/*
 * 현재 높이가 PAA5100JE 권장 초점 범위 안인지 확인한다.
 * 범위를 벗어나도 driver 동작 자체를 금지하지는 않는다.
 * 범위 밖이라는 결과는 "읽기 불가"가 아니라 SQUAL과 scale을 실기기로 다시
 * 확인하라는 경고 성격이다.
 */
int paa5100je_height_in_focus_range(
    double height_cm
);


/*
 * 센서 LED 설정을 변경한다.
 */
int paa5100je_set_led(
    paa5100je_t *sensor,
    int led_on
);


/*
 * 진단 도구와 begin sequence가 사용하는 raw register 접근 함수다.
 * 호출자가 bank-select register를 포함한 순서와 값의 의미를 책임진다.
 */
int paa5100je_register_write(
    paa5100je_t *sensor,
    uint8_t reg,
    uint8_t value
);

int paa5100je_register_read(
    paa5100je_t *sensor,
    uint8_t reg,
    uint8_t *value
);


/*
 * SPI file descriptor를 닫고 상태를 초기화한다.
 * 닫을 장치가 없으면 성공으로 처리한다.
 */
int paa5100je_close(
    paa5100je_t *sensor
);


#ifdef __cplusplus
}
#endif

#endif
