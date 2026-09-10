#define _POSIX_C_SOURCE 200809L

#include "../paa5100je.h"

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define TEST_POLL_MS 5
#define TEST_LOW_SQUAL 25U
#define TEST_LATE_GAP_NS UINT64_C(20000000)
#define TEST_RADIANS_TO_DEGREES 57.295779513082320876

typedef struct
{
    int64_t x_ticks;
    int64_t y_ticks;
    uint64_t reads;
    uint64_t motion_reads;
    uint64_t no_motion_reads;
    uint64_t low_squal_reads;
    uint64_t observation_nonzero;
    uint64_t near_limit_reads;
    uint64_t late_gap_reads;
    uint64_t max_gap_ns;
    uint64_t squal_sum;
    uint8_t squal_min;
    uint8_t squal_max;
    int32_t peak_abs_x;
    int32_t peak_abs_y;
    double duration_s;
} leg_stats_t;

static volatile sig_atomic_t stop_requested = 0;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static uint64_t monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return 0U;
    }
    return (uint64_t)now.tv_sec * UINT64_C(1000000000)
        + (uint64_t)now.tv_nsec;
}

static bool wait_for_enter(const char *prompt)
{
    char line[32];

    fputs(prompt, stdout);
    fflush(stdout);
    return fgets(line, sizeof(line), stdin) != NULL;
}

static int stdin_ready(int timeout_ms)
{
    struct pollfd descriptor = {
        .fd = STDIN_FILENO,
        .events = POLLIN,
        .revents = 0
    };
    int result;

    do
    {
        result = poll(&descriptor, 1U, timeout_ms);
    }
    while (result < 0 && errno == EINTR && !stop_requested);

    return result > 0 && (descriptor.revents & POLLIN) != 0;
}

static int absolute_tick(int16_t value)
{
    int converted = (int)value;
    return converted < 0 ? -converted : converted;
}

static int collect_leg(
    paa5100je_t *sensor,
    const char *name,
    const char *instruction,
    leg_stats_t *stats
)
{
    paa5100je_motion_t discarded;
    uint64_t started_ns;
    uint64_t previous_read_ns = 0U;
    char line[32];

    memset(stats, 0, sizeof(*stats));
    stats->squal_min = UINT8_MAX;

    printf("\n[%s 준비]\n", name);
    if (!wait_for_enter("현재 위치에서 정지한 뒤 Enter를 누르면 수집을 시작합니다: "))
    {
        return -1;
    }
    if (paa5100je_read_motion(sensor, &discarded) != 0)
    {
        return -1;
    }

    printf("%s\n", instruction);
    puts("이동을 끝내고 완전히 정지한 다음 Enter를 누르세요.");
    started_ns = monotonic_ns();

    while (!stop_requested)
    {
        paa5100je_motion_t motion;
        uint64_t read_ns;
        int abs_x;
        int abs_y;

        if (paa5100je_read_motion(sensor, &motion) != 0)
        {
            return -1;
        }
        read_ns = monotonic_ns();

        if (previous_read_ns != 0U)
        {
            uint64_t gap_ns = read_ns - previous_read_ns;
            if (gap_ns > stats->max_gap_ns)
            {
                stats->max_gap_ns = gap_ns;
            }
            if (gap_ns > TEST_LATE_GAP_NS)
            {
                stats->late_gap_reads++;
            }
        }
        previous_read_ns = read_ns;

        stats->reads++;
        stats->squal_sum += motion.squal;
        if (motion.squal < stats->squal_min)
        {
            stats->squal_min = motion.squal;
        }
        if (motion.squal > stats->squal_max)
        {
            stats->squal_max = motion.squal;
        }
        if (motion.squal < TEST_LOW_SQUAL)
        {
            stats->low_squal_reads++;
        }
        if (motion.observation != 0U)
        {
            stats->observation_nonzero++;
        }

        if (motion.motion_detected != 0)
        {
            stats->motion_reads++;
        }
        else
        {
            stats->no_motion_reads++;
        }

        stats->x_ticks += motion.delta_x_ticks;
        stats->y_ticks += motion.delta_y_ticks;
        abs_x = absolute_tick(motion.delta_x_ticks);
        abs_y = absolute_tick(motion.delta_y_ticks);
        if (abs_x > stats->peak_abs_x)
        {
            stats->peak_abs_x = abs_x;
        }
        if (abs_y > stats->peak_abs_y)
        {
            stats->peak_abs_y = abs_y;
        }
        if (abs_x >= 32000 || abs_y >= 32000)
        {
            stats->near_limit_reads++;
        }

        if (stdin_ready(TEST_POLL_MS))
        {
            if (fgets(line, sizeof(line), stdin) == NULL)
            {
                return -1;
            }
            break;
        }
    }

    stats->duration_s = (double)(monotonic_ns() - started_ns) / 1000000000.0;
    return stop_requested ? -1 : 0;
}

static void print_leg(const char *name, const leg_stats_t *stats)
{
    double norm = hypot((double)stats->x_ticks, (double)stats->y_ticks);
    double average_squal = stats->reads == 0U
        ? 0.0
        : (double)stats->squal_sum / (double)stats->reads;

    printf("\n%s\n", name);
    printf("raw_ticks=(%lld, %lld)  magnitude=%.1f\n",
           (long long)stats->x_ticks,
           (long long)stats->y_ticks,
           norm);
    printf("duration=%.3fs  reads=%llu  motion/no-motion=%llu/%llu\n",
           stats->duration_s,
           (unsigned long long)stats->reads,
           (unsigned long long)stats->motion_reads,
           (unsigned long long)stats->no_motion_reads);
    printf("SQUAL min/avg/max=%u/%.1f/%u  below_%u=%llu\n",
           stats->squal_min == UINT8_MAX ? 0U : stats->squal_min,
           average_squal,
           stats->squal_max,
           TEST_LOW_SQUAL,
           (unsigned long long)stats->low_squal_reads);
    printf("peak_abs_tick=(%d, %d)  max_read_gap=%.3fms  gap_over_20ms=%llu\n",
           stats->peak_abs_x,
           stats->peak_abs_y,
           (double)stats->max_gap_ns / 1000000.0,
           (unsigned long long)stats->late_gap_reads);
    printf("observation_nonzero=%llu  near_int16_limit=%llu\n",
           (unsigned long long)stats->observation_nonzero,
           (unsigned long long)stats->near_limit_reads);
}

static void print_comparison(
    const leg_stats_t *outbound,
    const leg_stats_t *returning
)
{
    double out_x = (double)outbound->x_ticks;
    double out_y = (double)outbound->y_ticks;
    double back_x = (double)returning->x_ticks;
    double back_y = (double)returning->y_ticks;
    double out_norm = hypot(out_x, out_y);
    double back_norm = hypot(back_x, back_y);
    double total_x = out_x + back_x;
    double total_y = out_y + back_y;
    double closure = hypot(total_x, total_y);
    double balance_percent = out_norm > 0.0
        ? back_norm * 100.0 / out_norm
        : 0.0;
    double closure_percent = out_norm > 0.0
        ? closure * 100.0 / out_norm
        : 0.0;
    double reverse_cosine = out_norm > 0.0 && back_norm > 0.0
        ? -(out_x * back_x + out_y * back_y) / (out_norm * back_norm)
        : 0.0;
    double direction_error_deg;

    if (reverse_cosine > 1.0)
    {
        reverse_cosine = 1.0;
    }
    else if (reverse_cosine < -1.0)
    {
        reverse_cosine = -1.0;
    }
    direction_error_deg = acos(reverse_cosine) * TEST_RADIANS_TO_DEGREES;

    puts("\n=== RAW TICK 왕복 비교 ===");
    printf("expected_return=(%+.0f, %+.0f)\n", -out_x, -out_y);
    printf("actual_return  =(%+.0f, %+.0f)\n", back_x, back_y);
    printf("uncancelled    =(%+.0f, %+.0f) ticks  magnitude=%.1f\n",
           total_x, total_y, closure);
    printf("return/out magnitude=%.2f%%\n", balance_percent);
    printf("reverse_direction_cosine=%.6f  (1.0이면 정확한 역방향)\n",
           reverse_cosine);
    printf("return_direction_error=%.2f deg  (0도면 정확한 역방향)\n",
           direction_error_deg);
    printf("raw_tick_closure_error=%.2f%% of outbound\n", closure_percent);
    if (direction_error_deg > 15.0)
    {
        puts("[!] 복귀 벡터가 역방향에서 15도 넘게 벗어났습니다.");
        puts("    차체 yaw/조향이 변했다면 이 trial로 PAA tick 대칭성을 판정할 수 없습니다.");
    }
    puts("판정 임계값은 두지 않습니다. 실제 이동거리/복귀 정밀도와 함께 위 원시값을 전달하세요.");
}

int main(void)
{
    paa5100je_t sensor;
    leg_stats_t outbound;
    leg_stats_t returning;
    int result = EXIT_FAILURE;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    memset(&sensor, 0, sizeof(sensor));

    puts("=== PAA5100JE FORWARD/RETURN RAW TICK TEST ===");
    puts("BNO086, heading, scaler, 좌표 융합을 사용하지 않고 PAA raw tick만 비교합니다.");

    if (paa5100je_open(
            &sensor,
            0U,
            1U,
            PAA5100JE_DEFAULT_SPEED_HZ,
            PAA5100JE_DEFAULT_HEIGHT_CM,
            PAA5100JE_DEFAULT_SCALER
        ) != 0
        || paa5100je_begin(&sensor) != 0)
    {
        fprintf(stderr, "PAA5100JE 초기화 실패: %s\n", strerror(errno));
        goto cleanup;
    }

    printf("PAA ID=0x%02X inverse=0x%02X revision=0x%02X\n",
           sensor.chip_id, sensor.inverse_chip_id, sensor.revision);
    puts("중요: 이 시험의 raw tick은 차량 body 좌표이므로 시험 내내 차체 yaw를 고정해야 합니다.");
    puts("차를 돌려서 돌아오거나 조향하지 말고, 앞을 향한 자세 그대로 후진하십시오.");
    puts("가능하면 30~50 cm 직선 가이드를 따라 천천히 왕복하십시오.");

    if (collect_leg(
            &sensor,
            "OUTBOUND",
            "이제 차량을 직선으로 전진시키세요.",
            &outbound
        ) != 0)
    {
        if (!stop_requested)
        {
            fprintf(stderr, "OUTBOUND 수집 실패: %s\n", strerror(errno));
        }
        goto cleanup;
    }
    print_leg("OUTBOUND", &outbound);

    if (collect_leg(
            &sensor,
            "RETURN",
            "차체 방향과 조향을 그대로 둔 채 후진하여 출발점으로 복귀시키세요.",
            &returning
        ) != 0)
    {
        if (!stop_requested)
        {
            fprintf(stderr, "RETURN 수집 실패: %s\n", strerror(errno));
        }
        goto cleanup;
    }
    print_leg("RETURN", &returning);
    print_comparison(&outbound, &returning);
    result = EXIT_SUCCESS;

cleanup:
    (void)paa5100je_close(&sensor);
    return result;
}
