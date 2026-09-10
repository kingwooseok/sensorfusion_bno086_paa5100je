#define _POSIX_C_SOURCE 200809L

/*
 * ==================================================
 * BNO086 RC카 충돌 데이터 수집기
 * ==================================================
 *
 * 이 프로그램은 사고 판정기가 아니라 실기기 계측 도구다. 실제 RC카의 정상
 * 주행, 약한 접촉, 충돌 데이터를 먼저 모아 팀이 범위를 검토한 뒤 production
 * 임계값을 정할 수 있게 한다. 소스 코드와 임계값은 절대로 자동 변경하지 않는다.
 *
 * Thread 역할:
 *
 *     BNO086 I/O Thread
 *          - begin(), feature 설정, update()를 단독 소유한다.
 *
 *     Main / Capture Thread
 *          - 시험 중 독립 SPMC cursor를 계속 비운다.
 *          - acceleration, jerk, sliding delta-V를 계산한다.
 *          - 사용자가 시험의 유효성을 확인한 뒤에만 파일을 저장한다.
 *
 * Build:
 *
 *     gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror \
 *         -I. test_collision_capture.c bno086.c -pthread -lm \
 *         -o test_collision_capture
 *
 * 하드웨어 없이 수학/상태 처리만 검사하는 self-test도 제공한다. 여기서 쓰는
 * 인공 숫자는 구현 회귀시험 전용이며 calibration 자료로 저장하거나 임계값을
 * 계산하는 데 절대로 사용하지 않는다.
 *
 *     ./test_collision_capture --self-test
 */

#include "bno086.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define CAPTURE_ROOT_DIRECTORY          "collision_captures"
#define CAPTURE_SUMMARY_FILE            "summary.csv"
#define CAPTURE_PATH_SIZE               512U
#define CAPTURE_LINE_SIZE              1024U
#define CAPTURE_NOTE_SIZE                96U
#define CAPTURE_SESSION_NAME_SIZE        64U
#define CAPTURE_FEATURE_TIMEOUT_MS      1500U
#define CAPTURE_REPORT_TIMEOUT_MS       1500U
#define CAPTURE_UPDATE_SLEEP_US         1000U
#define CAPTURE_DEFAULT_INTERVAL_US    10000U
#define CAPTURE_DEFAULT_MAX_SECONDS       300.0
#define CAPTURE_MAX_SECONDS              3600.0
#define CAPTURE_MAX_SAMPLES           1000000U
#define CAPTURE_WINDOW_COUNT                4U
#define CAPTURE_DV_QUEUE_CAPACITY         256U
#define CAPTURE_STALE_MIN_NS        UINT64_C(250000000)
#define CAPTURE_STANDARD_GRAVITY_MPS2      9.80665
#define CAPTURE_ACCEL_FULL_SCALE_G         8.0
#define CAPTURE_ACCEL_NEAR_RAIL_RATIO      0.95
#define CAPTURE_ACCEL_NEAR_RAIL_MPS2 \
    (CAPTURE_STANDARD_GRAVITY_MPS2 * CAPTURE_ACCEL_FULL_SCALE_G \
     * CAPTURE_ACCEL_NEAR_RAIL_RATIO)
#define NS_PER_SECOND              UINT64_C(1000000000)
#define NS_PER_MILLISECOND         UINT64_C(1000000)

typedef enum
{
    TRIAL_LABEL_NORMAL = 0,
    TRIAL_LABEL_COLLISION
} trial_label_t;

typedef enum
{
    TRIAL_OUTCOME_NORMAL = 0,
    TRIAL_OUTCOME_NON_ACCIDENT_CONTACT,
    TRIAL_OUTCOME_IMPACT,
    TRIAL_OUTCOME_INVALID
} trial_outcome_t;

typedef enum
{
    IMPACT_DIRECTION_FRONT = 0,
    IMPACT_DIRECTION_REAR,
    IMPACT_DIRECTION_LEFT,
    IMPACT_DIRECTION_RIGHT,
    IMPACT_DIRECTION_DIAGONAL,
    IMPACT_DIRECTION_UNKNOWN,
    IMPACT_DIRECTION_COUNT
} impact_direction_t;

typedef enum
{
    NORMAL_CONDITION_STATIONARY = 0,
    NORMAL_CONDITION_STRAIGHT,
    NORMAL_CONDITION_ACCELERATION,
    NORMAL_CONDITION_BRAKING,
    NORMAL_CONDITION_TURNING,
    NORMAL_CONDITION_ROAD_BUMP,
    NORMAL_CONDITION_UNKNOWN,
    NORMAL_CONDITION_COUNT
} normal_condition_t;

typedef enum
{
    REVIEW_KEEP = 0,
    REVIEW_RETRY,
    REVIEW_DISCARD,
    REVIEW_EXIT
} review_action_t;

typedef struct
{
    trial_label_t label;
    impact_direction_t direction;
    normal_condition_t normal_condition;
} trial_plan_t;

typedef struct
{
    double x;
    double y;
    double z;
} vector3_t;

typedef struct
{
    vector3_t values[CAPTURE_DV_QUEUE_CAPACITY];
    size_t head;
    size_t count;
    size_t limit;
    vector3_t sum;
    double max_magnitude;
    double max_time_s;
} delta_v_window_t;

typedef struct
{
    bool previous_valid;
    uint8_t previous_sequence;
    uint64_t previous_generation;
    vector3_t previous_value;
} derivative_state_t;

typedef struct
{
    uint64_t monotonic_ns;
    double elapsed_s;
    bno086_event_type_t type;
    uint8_t report_sequence;
    uint8_t accuracy;
    uint64_t generation;
    vector3_t value;
    double magnitude;
    bool sequence_contiguous;
    double applied_dt_s;
    double jerk_mps3;
    double delta_v_mps[CAPTURE_WINDOW_COUNT];
} capture_sample_t;

typedef struct
{
    capture_sample_t *items;
    size_t count;
    size_t capacity;
    bool limit_reached;
} sample_buffer_t;

typedef struct
{
    uint64_t raw_reports;
    uint64_t linear_reports;
    uint64_t gyro_reports;
    uint64_t raw_accel_near_rail_reports;
    uint64_t sequence_gaps;
    uint64_t generation_changes;

    double peak_raw_accel_mps2;
    double peak_raw_accel_time_s;
    vector3_t peak_raw_accel_vector;

    double peak_linear_accel_mps2;
    double peak_linear_accel_time_s;
    vector3_t peak_linear_accel_vector;

    double peak_raw_jerk_mps3;
    double peak_raw_jerk_time_s;
    double peak_linear_jerk_mps3;
    double peak_linear_jerk_time_s;

    double peak_gyro_rad_s;
    double peak_gyro_time_s;

    double max_delta_v_mps[CAPTURE_WINDOW_COUNT];
    double max_delta_v_time_s[CAPTURE_WINDOW_COUNT];

    derivative_state_t raw_derivative;
    derivative_state_t linear_derivative;
    derivative_state_t gyro_derivative;
    delta_v_window_t delta_v_windows[CAPTURE_WINDOW_COUNT];
} capture_metrics_t;

typedef struct
{
    trial_plan_t plan;
    trial_outcome_t outcome;
    impact_direction_t actual_direction;
    bool approved;
    bool eligible;
    bool technical_valid;
    char quality[32];
    char note[CAPTURE_NOTE_SIZE];

    uint64_t trial_id;
    double duration_s;
    uint64_t trial_start_ns;
    uint64_t marker_ns;
    size_t sample_count;
    capture_metrics_t metrics;

    uint32_t raw_interval_us;
    uint32_t linear_interval_us;
    uint32_t gyro_interval_us;

    uint64_t cursor_dropped;
    uint64_t startup_losses;
    uint64_t stale_events;
    uint64_t reset_count;
    uint64_t syscall_errors;
    uint64_t validation_errors;
    uint64_t other_transport_errors;
    uint64_t phase2_null_count;
    uint64_t phase2_null_retry_success;
    uint64_t phase2_null_retry_fail;
} trial_result_t;

typedef struct
{
    uint64_t trial_id;
    trial_label_t planned_label;
    trial_outcome_t outcome;
    impact_direction_t planned_direction;
    impact_direction_t actual_direction;
    normal_condition_t planned_normal_condition;
    bool eligible;
    uint64_t stale_events;
    double peak_raw_accel_mps2;
    double peak_linear_accel_mps2;
    double peak_raw_jerk_mps3;
    double peak_linear_jerk_mps3;
    double max_delta_v_mps[CAPTURE_WINDOW_COUNT];
} saved_trial_t;

typedef struct
{
    char name[CAPTURE_SESSION_NAME_SIZE];
    char directory[CAPTURE_PATH_SIZE];
    char summary_path[CAPTURE_PATH_SIZE];
    FILE *summary_file;
    saved_trial_t *trials;
    size_t trial_count;
    size_t trial_capacity;
    uint64_t next_trial_id;
} capture_session_t;

typedef struct
{
    bno086_t imu;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    bool synchronization_initialized;
    bool imu_initialized;
    bool thread_created;
    bool start_finished;
    bool ready;
    bool stop_requested;
    uint32_t raw_interval_us;
    uint32_t linear_interval_us;
    uint32_t gyro_interval_us;
    char start_error[BNO086_ERROR_TEXT_SIZE];
} imu_worker_t;

typedef struct
{
    const char *session_name;
    unsigned int minimum_normal_trials;
    unsigned int minimum_normal_conditions;
    unsigned int minimum_directions;
    unsigned int minimum_per_direction;
    double maximum_trial_seconds;
    bool self_test;
    bool help;
} program_options_t;

static const double delta_v_window_seconds[CAPTURE_WINDOW_COUNT] = {
    0.020,
    0.050,
    0.100,
    0.200
};

static volatile sig_atomic_t stop_requested;

/* ========================================================================== */
/* Time, signal, and text helpers                                              */
/* ========================================================================== */

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
    return (uint64_t)now.tv_sec * NS_PER_SECOND + (uint64_t)now.tv_nsec;
}

static void sleep_microseconds(unsigned int microseconds)
{
    struct timespec duration;

    duration.tv_sec = (time_t)(microseconds / 1000000U);
    duration.tv_nsec = (long)(microseconds % 1000000U) * 1000L;
    while (nanosleep(&duration, &duration) != 0 && errno == EINTR)
    {
        if (stop_requested != 0)
        {
            break;
        }
    }
}

static void trim_line(char *text)
{
    size_t length;
    size_t start = 0U;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);
    while (length > 0U && isspace((unsigned char)text[length - 1U]) != 0)
    {
        text[--length] = '\0';
    }
    while (text[start] != '\0' && isspace((unsigned char)text[start]) != 0)
    {
        start++;
    }
    if (start > 0U)
    {
        memmove(text, text + start, strlen(text + start) + 1U);
    }
}

static int read_line_prompt(const char *prompt, char *buffer, size_t size)
{
    if (prompt != NULL)
    {
        fputs(prompt, stdout);
        fflush(stdout);
    }
    if (buffer == NULL || size < 2U || fgets(buffer, (int)size, stdin) == NULL)
    {
        return 0;
    }
    if (strchr(buffer, '\n') == NULL)
    {
        int character;

        do
        {
            character = getchar();
        }
        while (character != '\n' && character != EOF);
    }
    trim_line(buffer);
    return 1;
}

static bool parse_unsigned(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0'
        || parsed > (unsigned long)UINT_MAX)
    {
        return false;
    }
    *value = (unsigned int)parsed;
    return true;
}

static bool parse_positive_double(const char *text, double *value)
{
    char *end = NULL;
    double parsed;

    if (text == NULL || value == NULL || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0'
        || !isfinite(parsed) || parsed <= 0.0)
    {
        return false;
    }
    *value = parsed;
    return true;
}

static double vector_magnitude(vector3_t value)
{
    return sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

static vector3_t vector_add(vector3_t left, vector3_t right)
{
    vector3_t result;

    result.x = left.x + right.x;
    result.y = left.y + right.y;
    result.z = left.z + right.z;
    return result;
}

static vector3_t vector_subtract(vector3_t left, vector3_t right)
{
    vector3_t result;

    result.x = left.x - right.x;
    result.y = left.y - right.y;
    result.z = left.z - right.z;
    return result;
}

static vector3_t vector_scale(vector3_t value, double scale)
{
    vector3_t result;

    result.x = value.x * scale;
    result.y = value.y * scale;
    result.z = value.z * scale;
    return result;
}

static const char *trial_label_name(trial_label_t label)
{
    return label == TRIAL_LABEL_COLLISION ? "collision" : "normal";
}

static bool parse_trial_label(const char *text, trial_label_t *label)
{
    if (strcmp(text, "normal") == 0)
    {
        *label = TRIAL_LABEL_NORMAL;
        return true;
    }
    if (strcmp(text, "collision") == 0)
    {
        *label = TRIAL_LABEL_COLLISION;
        return true;
    }
    return false;
}

static const char *trial_outcome_name(trial_outcome_t outcome)
{
    switch (outcome)
    {
        case TRIAL_OUTCOME_NORMAL:
            return "normal";
        case TRIAL_OUTCOME_NON_ACCIDENT_CONTACT:
            return "non_accident_contact";
        case TRIAL_OUTCOME_IMPACT:
            return "impact";
        case TRIAL_OUTCOME_INVALID:
        default:
            return "invalid";
    }
}

static bool parse_trial_outcome(const char *text, trial_outcome_t *outcome)
{
    trial_outcome_t candidate;

    if (text == NULL || outcome == NULL)
    {
        return false;
    }
    for (candidate = TRIAL_OUTCOME_NORMAL;
         candidate <= TRIAL_OUTCOME_INVALID;
         candidate = (trial_outcome_t)((int)candidate + 1))
    {
        if (strcmp(text, trial_outcome_name(candidate)) == 0)
        {
            *outcome = candidate;
            return true;
        }
    }
    return false;
}

static const char *direction_name(impact_direction_t direction)
{
    static const char *const names[IMPACT_DIRECTION_COUNT] = {
        "front", "rear", "left", "right", "diagonal", "unknown"
    };

    if ((unsigned int)direction >= (unsigned int)IMPACT_DIRECTION_COUNT)
    {
        return "unknown";
    }
    return names[(size_t)direction];
}

static bool parse_direction_name(
    const char *text,
    impact_direction_t *direction
)
{
    size_t index;

    for (index = 0U; index < (size_t)IMPACT_DIRECTION_COUNT; index++)
    {
        impact_direction_t candidate = (impact_direction_t)index;

        if (strcmp(text, direction_name(candidate)) == 0)
        {
            *direction = candidate;
            return true;
        }
    }
    return false;
}

static const char *normal_condition_name(normal_condition_t condition)
{
    static const char *const names[NORMAL_CONDITION_COUNT] = {
        "stationary",
        "constant_straight",
        "acceleration",
        "braking",
        "turning",
        "road_bump",
        "unknown"
    };

    if ((unsigned int)condition >= (unsigned int)NORMAL_CONDITION_COUNT)
    {
        return "unknown";
    }
    return names[(size_t)condition];
}

static bool parse_normal_condition_name(
    const char *text,
    normal_condition_t *condition
)
{
    size_t index;

    for (index = 0U; index < (size_t)NORMAL_CONDITION_COUNT; index++)
    {
        normal_condition_t candidate = (normal_condition_t)index;

        if (strcmp(text, normal_condition_name(candidate)) == 0)
        {
            *condition = candidate;
            return true;
        }
    }
    return false;
}

static const char *event_type_name(bno086_event_type_t type)
{
    switch (type)
    {
        case BNO086_EVENT_ACCELEROMETER:
            return "raw_accel";
        case BNO086_EVENT_LINEAR_ACCELERATION:
            return "linear_accel";
        case BNO086_EVENT_GYROSCOPE:
            return "gyro";
        default:
            return "ignored";
    }
}

static const char *event_units(bno086_event_type_t type)
{
    return type == BNO086_EVENT_GYROSCOPE ? "rad/s" : "m/s^2";
}

static const char *dominant_axis_name(vector3_t value)
{
    double x = fabs(value.x);
    double y = fabs(value.y);
    double z = fabs(value.z);

    if (x >= y && x >= z)
    {
        return value.x >= 0.0 ? "+X" : "-X";
    }
    if (y >= x && y >= z)
    {
        return value.y >= 0.0 ? "+Y" : "-Y";
    }
    return value.z >= 0.0 ? "+Z" : "-Z";
}

static uint64_t counter_difference(uint64_t after, uint64_t before)
{
    return after >= before ? after - before : after;
}

/* ========================================================================== */
/* Metric accumulation                                                        */
/* ========================================================================== */

static void delta_v_window_reset(delta_v_window_t *window)
{
    window->head = 0U;
    window->count = 0U;
    window->sum = (vector3_t){0.0, 0.0, 0.0};
}

static void delta_v_window_initialize(
    delta_v_window_t *window,
    double duration_s,
    double sample_interval_s
)
{
    double required_samples = ceil(duration_s / sample_interval_s);
    size_t limit;

    memset(window, 0, sizeof(*window));
    if (required_samples < 1.0)
    {
        required_samples = 1.0;
    }
    if (required_samples > (double)CAPTURE_DV_QUEUE_CAPACITY)
    {
        required_samples = (double)CAPTURE_DV_QUEUE_CAPACITY;
    }
    limit = (size_t)required_samples;
    window->limit = limit;
}

static double delta_v_window_push(
    delta_v_window_t *window,
    vector3_t contribution,
    double elapsed_s
)
{
    size_t tail;
    double magnitude;

    if (window->count == window->limit)
    {
        vector3_t removed = window->values[window->head];

        window->sum = vector_subtract(window->sum, removed);
        window->head = (window->head + 1U) % CAPTURE_DV_QUEUE_CAPACITY;
        window->count--;
    }

    tail = (window->head + window->count) % CAPTURE_DV_QUEUE_CAPACITY;
    window->values[tail] = contribution;
    window->sum = vector_add(window->sum, contribution);
    window->count++;

    if (window->count < window->limit)
    {
        return NAN;
    }

    magnitude = vector_magnitude(window->sum);
    if (magnitude > window->max_magnitude)
    {
        window->max_magnitude = magnitude;
        window->max_time_s = elapsed_s;
    }
    return magnitude;
}

static void capture_metrics_initialize(
    capture_metrics_t *metrics,
    double linear_interval_s
)
{
    size_t index;

    memset(metrics, 0, sizeof(*metrics));
    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        delta_v_window_initialize(
            &metrics->delta_v_windows[index],
            delta_v_window_seconds[index],
            linear_interval_s
        );
    }
}

static void reset_linear_windows(capture_metrics_t *metrics)
{
    size_t index;

    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        delta_v_window_reset(&metrics->delta_v_windows[index]);
    }
}

static bool derivative_is_contiguous(
    derivative_state_t *state,
    const bno086_event_t *event,
    capture_metrics_t *metrics,
    bool reset_delta_v
)
{
    bool contiguous = false;

    if (state->previous_valid)
    {
        uint8_t expected = (uint8_t)((unsigned int)state->previous_sequence + 1U);

        if (state->previous_generation != event->bno_generation)
        {
            metrics->generation_changes++;
            if (reset_delta_v)
            {
                reset_linear_windows(metrics);
            }
        }
        else if (event->report_sequence != expected)
        {
            metrics->sequence_gaps++;
            if (reset_delta_v)
            {
                reset_linear_windows(metrics);
            }
        }
        else
        {
            contiguous = true;
        }
    }
    return contiguous;
}

static void derivative_update(
    derivative_state_t *state,
    const bno086_event_t *event,
    vector3_t value
)
{
    state->previous_valid = true;
    state->previous_sequence = event->report_sequence;
    state->previous_generation = event->bno_generation;
    state->previous_value = value;
}

static bool sample_buffer_append(
    sample_buffer_t *buffer,
    const capture_sample_t *sample
)
{
    capture_sample_t *resized;
    size_t new_capacity;

    if (buffer->count >= CAPTURE_MAX_SAMPLES)
    {
        buffer->limit_reached = true;
        return false;
    }
    if (buffer->count == buffer->capacity)
    {
        if (buffer->capacity == 0U)
        {
            new_capacity = 4096U;
        }
        else if (buffer->capacity > CAPTURE_MAX_SAMPLES / 2U)
        {
            new_capacity = CAPTURE_MAX_SAMPLES;
        }
        else
        {
            new_capacity = buffer->capacity * 2U;
        }
        if (new_capacity > CAPTURE_MAX_SAMPLES)
        {
            new_capacity = CAPTURE_MAX_SAMPLES;
        }
        if (new_capacity > SIZE_MAX / sizeof(*buffer->items))
        {
            buffer->limit_reached = true;
            return false;
        }
        resized = realloc(buffer->items, new_capacity * sizeof(*buffer->items));
        if (resized == NULL)
        {
            buffer->limit_reached = true;
            return false;
        }
        buffer->items = resized;
        buffer->capacity = new_capacity;
    }
    buffer->items[buffer->count++] = *sample;
    return true;
}

static void sample_buffer_release(sample_buffer_t *buffer)
{
    free(buffer->items);
    memset(buffer, 0, sizeof(*buffer));
}

static bool process_capture_event(
    const bno086_event_t *event,
    uint64_t trial_start_ns,
    uint32_t raw_interval_us,
    uint32_t linear_interval_us,
    uint32_t gyro_interval_us,
    capture_metrics_t *metrics,
    sample_buffer_t *samples
)
{
    capture_sample_t sample;
    derivative_state_t *derivative;
    double interval_s;
    bool reset_delta_v = false;
    bool contiguous;
    size_t index;

    if (event->type != BNO086_EVENT_ACCELEROMETER
        && event->type != BNO086_EVENT_LINEAR_ACCELERATION
        && event->type != BNO086_EVENT_GYROSCOPE)
    {
        return true;
    }

    memset(&sample, 0, sizeof(sample));
    sample.monotonic_ns = event->monotonic_ns;
    sample.elapsed_s = event->monotonic_ns >= trial_start_ns
        ? (double)(event->monotonic_ns - trial_start_ns) / (double)NS_PER_SECOND
        : 0.0;
    sample.type = event->type;
    sample.report_sequence = event->report_sequence;
    sample.accuracy = event->accuracy;
    sample.generation = event->bno_generation;
    sample.value = (vector3_t){event->x, event->y, event->z};
    sample.magnitude = vector_magnitude(sample.value);
    sample.jerk_mps3 = NAN;
    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        sample.delta_v_mps[index] = NAN;
    }

    if (event->type == BNO086_EVENT_ACCELEROMETER)
    {
        derivative = &metrics->raw_derivative;
        interval_s = (double)raw_interval_us / 1000000.0;
        metrics->raw_reports++;
        /*
         * BNO086 accelerometer의 정격 측정 범위(+-8 g) 가까이에 도달한
         * trial은 충돌 peak가 잘렸을 가능성이 있다. 임계값 자료로 쓰지
         * 않도록 축 하나라도 full-scale의 95%에 닿은 report를 센다.
         */
        if (fabs(sample.value.x) >= CAPTURE_ACCEL_NEAR_RAIL_MPS2
            || fabs(sample.value.y) >= CAPTURE_ACCEL_NEAR_RAIL_MPS2
            || fabs(sample.value.z) >= CAPTURE_ACCEL_NEAR_RAIL_MPS2)
        {
            metrics->raw_accel_near_rail_reports++;
        }
        if (sample.magnitude > metrics->peak_raw_accel_mps2)
        {
            metrics->peak_raw_accel_mps2 = sample.magnitude;
            metrics->peak_raw_accel_time_s = sample.elapsed_s;
            metrics->peak_raw_accel_vector = sample.value;
        }
    }
    else if (event->type == BNO086_EVENT_LINEAR_ACCELERATION)
    {
        derivative = &metrics->linear_derivative;
        interval_s = (double)linear_interval_us / 1000000.0;
        reset_delta_v = true;
        metrics->linear_reports++;
        if (sample.magnitude > metrics->peak_linear_accel_mps2)
        {
            metrics->peak_linear_accel_mps2 = sample.magnitude;
            metrics->peak_linear_accel_time_s = sample.elapsed_s;
            metrics->peak_linear_accel_vector = sample.value;
        }
    }
    else
    {
        derivative = &metrics->gyro_derivative;
        interval_s = (double)gyro_interval_us / 1000000.0;
        metrics->gyro_reports++;
        if (sample.magnitude > metrics->peak_gyro_rad_s)
        {
            metrics->peak_gyro_rad_s = sample.magnitude;
            metrics->peak_gyro_time_s = sample.elapsed_s;
        }
    }

    sample.applied_dt_s = interval_s;
    contiguous = derivative_is_contiguous(
        derivative,
        event,
        metrics,
        reset_delta_v
    );
    sample.sequence_contiguous = contiguous;

    if (contiguous)
    {
        vector3_t difference = vector_subtract(
            sample.value,
            derivative->previous_value
        );

        sample.jerk_mps3 = vector_magnitude(difference) / interval_s;
        if (event->type == BNO086_EVENT_ACCELEROMETER
            && sample.jerk_mps3 > metrics->peak_raw_jerk_mps3)
        {
            metrics->peak_raw_jerk_mps3 = sample.jerk_mps3;
            metrics->peak_raw_jerk_time_s = sample.elapsed_s;
        }
        if (event->type == BNO086_EVENT_LINEAR_ACCELERATION)
        {
            vector3_t average = vector_scale(
                vector_add(sample.value, derivative->previous_value),
                0.5
            );
            vector3_t contribution = vector_scale(average, interval_s);

            if (sample.jerk_mps3 > metrics->peak_linear_jerk_mps3)
            {
                metrics->peak_linear_jerk_mps3 = sample.jerk_mps3;
                metrics->peak_linear_jerk_time_s = sample.elapsed_s;
            }
            for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
            {
                sample.delta_v_mps[index] = delta_v_window_push(
                    &metrics->delta_v_windows[index],
                    contribution,
                    sample.elapsed_s
                );
                metrics->max_delta_v_mps[index] =
                    metrics->delta_v_windows[index].max_magnitude;
                metrics->max_delta_v_time_s[index] =
                    metrics->delta_v_windows[index].max_time_s;
            }
        }
    }

    derivative_update(derivative, event, sample.value);
    return sample_buffer_append(samples, &sample);
}

/* ========================================================================== */
/* BNO086 단일 소유 I/O Thread                                                 */
/* ========================================================================== */

static bool imu_worker_stop_requested(imu_worker_t *worker)
{
    bool requested;

    pthread_mutex_lock(&worker->mutex);
    requested = worker->stop_requested;
    pthread_mutex_unlock(&worker->mutex);
    return requested;
}

static void imu_worker_publish_start(
    imu_worker_t *worker,
    bool ready,
    const char *error
)
{
    pthread_mutex_lock(&worker->mutex);
    worker->ready = ready;
    worker->start_finished = true;
    if (error != NULL)
    {
        (void)snprintf(
            worker->start_error,
            sizeof(worker->start_error),
            "%s",
            error
        );
    }
    pthread_cond_broadcast(&worker->condition);
    pthread_mutex_unlock(&worker->mutex);
}

static bool setup_one_feature(
    bno086_t *imu,
    const char *name,
    uint8_t sensor_id,
    uint32_t requested_interval_us,
    uint32_t *applied_interval_us,
    char *error,
    size_t error_size
)
{
    uint32_t applied = 0U;

    if (bno086_enable_feature(
            imu,
            sensor_id,
            requested_interval_us,
            CAPTURE_FEATURE_TIMEOUT_MS
        ) <= 0)
    {
        (void)snprintf(error, error_size, "%s FC 설정 응답 실패", name);
        return false;
    }
    if (bno086_wait_for_sensor_report(
            imu,
            sensor_id,
            CAPTURE_REPORT_TIMEOUT_MS
        ) <= 0)
    {
        (void)snprintf(error, error_size, "%s 실제 report 수신 실패", name);
        return false;
    }
    if (bno086_get_feature_interval(imu, sensor_id, &applied) <= 0
        || applied == 0U)
    {
        (void)snprintf(error, error_size, "%s 실제 FC interval 없음", name);
        return false;
    }
    if (applied_interval_us != NULL)
    {
        *applied_interval_us = applied;
    }
    return true;
}

static void *imu_thread_main(void *argument)
{
    imu_worker_t *worker = argument;
    char error[BNO086_ERROR_TEXT_SIZE] = "";
    char summary[BNO086_ERROR_TEXT_SIZE] = "";
    uint32_t ignored_interval = 0U;

    if (bno086_begin(&worker->imu) <= 0)
    {
        (void)bno086_status_summary(&worker->imu, summary, sizeof(summary));
        (void)snprintf(
            error,
            sizeof(error),
            "BNO086 begin 실패: %.120s",
            summary
        );
        imu_worker_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    /*
     * 사고차 production과 비슷한 bus/report 부하를 재현하기 위해 위치용
     * RV/Mag도 함께 켠다. 수집 CSV에는 사고 판정 후보인 raw acceleration,
     * linear acceleration, gyro만 기록한다.
     */
    if (!setup_one_feature(
            &worker->imu,
            "Rotation Vector",
            BNO086_SENSOR_ROTATION_VECTOR,
            CAPTURE_DEFAULT_INTERVAL_US,
            &ignored_interval,
            error,
            sizeof(error)
        )
        || !setup_one_feature(
            &worker->imu,
            "Gyroscope",
            BNO086_SENSOR_GYROSCOPE_CALIBRATED,
            CAPTURE_DEFAULT_INTERVAL_US,
            &worker->gyro_interval_us,
            error,
            sizeof(error)
        )
        || !setup_one_feature(
            &worker->imu,
            "Magnetometer",
            BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
            20000U,
            &ignored_interval,
            error,
            sizeof(error)
        )
        || !setup_one_feature(
            &worker->imu,
            "Accelerometer",
            BNO086_SENSOR_ACCELEROMETER,
            CAPTURE_DEFAULT_INTERVAL_US,
            &worker->raw_interval_us,
            error,
            sizeof(error)
        )
        || !setup_one_feature(
            &worker->imu,
            "Linear Acceleration",
            BNO086_SENSOR_LINEAR_ACCELERATION,
            CAPTURE_DEFAULT_INTERVAL_US,
            &worker->linear_interval_us,
            error,
            sizeof(error)
        )
        || bno086_start_dynamic_calibration(&worker->imu) <= 0)
    {
        if (error[0] == '\0')
        {
            (void)snprintf(error, sizeof(error), "동적 보정 명령 응답 실패");
        }
        imu_worker_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    imu_worker_publish_start(worker, true, NULL);
    while (!imu_worker_stop_requested(worker))
    {
        (void)bno086_update(&worker->imu);
        sleep_microseconds(CAPTURE_UPDATE_SLEEP_US);
    }

    bno086_close(&worker->imu);
    return NULL;
}

static bool imu_worker_start(imu_worker_t *worker)
{
    bno086_config_t config;
    int result;

    memset(worker, 0, sizeof(*worker));
    if (pthread_mutex_init(&worker->mutex, NULL) != 0)
    {
        return false;
    }
    if (pthread_cond_init(&worker->condition, NULL) != 0)
    {
        pthread_mutex_destroy(&worker->mutex);
        return false;
    }
    worker->synchronization_initialized = true;

    bno086_default_config(&config);
    if (bno086_init(&worker->imu, &config) <= 0)
    {
        pthread_cond_destroy(&worker->condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->synchronization_initialized = false;
        return false;
    }
    worker->imu_initialized = true;

    result = pthread_create(&worker->thread, NULL, imu_thread_main, worker);
    if (result != 0)
    {
        (void)fprintf(stderr, "BNO086 Thread 생성 실패: %s\n", strerror(result));
        bno086_deinit(&worker->imu);
        worker->imu_initialized = false;
        pthread_cond_destroy(&worker->condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->synchronization_initialized = false;
        return false;
    }
    worker->thread_created = true;

    pthread_mutex_lock(&worker->mutex);
    while (!worker->start_finished)
    {
        pthread_cond_wait(&worker->condition, &worker->mutex);
    }
    result = worker->ready ? 1 : 0;
    pthread_mutex_unlock(&worker->mutex);
    return result != 0;
}

static void imu_worker_stop(imu_worker_t *worker)
{
    if (worker->thread_created)
    {
        pthread_mutex_lock(&worker->mutex);
        worker->stop_requested = true;
        pthread_cond_broadcast(&worker->condition);
        pthread_mutex_unlock(&worker->mutex);
        (void)pthread_join(worker->thread, NULL);
        worker->thread_created = false;
    }
    if (worker->imu_initialized)
    {
        bno086_deinit(&worker->imu);
        worker->imu_initialized = false;
    }
    if (worker->synchronization_initialized)
    {
        pthread_cond_destroy(&worker->condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->synchronization_initialized = false;
    }
}

/* ========================================================================== */
/* Session directory and approved-trial CSV                                   */
/* ========================================================================== */

static const char summary_csv_header[] =
    "trial_id,planned_label,planned_direction,planned_normal_condition,"
    "outcome,actual_direction,"
    "quality,eligible,technical_valid,duration_s,sample_count,raw_reports,"
    "linear_reports,gyro_reports,peak_raw_accel_mps2,peak_linear_accel_mps2,"
    "peak_raw_jerk_mps3,peak_linear_jerk_mps3,max_dv_20ms_mps,"
    "max_dv_50ms_mps,max_dv_100ms_mps,max_dv_200ms_mps,peak_gyro_rad_s,"
    "dominant_body_axis,sequence_gaps,generation_changes,cursor_dropped,"
    "startup_losses,stale_events,reset_count,syscall_errors,validation_errors,"
    "other_transport_errors,phase2_null_count,phase2_null_retry_success,"
    "phase2_null_retry_fail,raw_interval_us,linear_interval_us,"
    "gyro_interval_us,raw_accel_near_rail_reports,note,data_file\n";

static bool path_join(
    char *destination,
    size_t destination_size,
    const char *left,
    const char *right
)
{
    int length = snprintf(destination, destination_size, "%s/%s", left, right);

    return length >= 0 && (size_t)length < destination_size;
}

static bool ensure_directory(const char *path)
{
    struct stat status;

    if (mkdir(path, (mode_t)0755) == 0)
    {
        return true;
    }
    if (errno != EEXIST || stat(path, &status) != 0 || !S_ISDIR(status.st_mode))
    {
        (void)fprintf(stderr, "directory 생성 실패 (%s): %s\n", path, strerror(errno));
        return false;
    }
    return true;
}

static bool valid_session_name(const char *name)
{
    size_t index;
    size_t length;

    if (name == NULL)
    {
        return false;
    }
    length = strlen(name);
    if (length == 0U || length >= CAPTURE_SESSION_NAME_SIZE
        || strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
    {
        return false;
    }
    for (index = 0U; index < length; index++)
    {
        unsigned char character = (unsigned char)name[index];

        if (isalnum(character) == 0
            && character != (unsigned char)'_'
            && character != (unsigned char)'-'
            && character != (unsigned char)'.')
        {
            return false;
        }
    }
    return true;
}

static bool make_default_session_name(char *name, size_t name_size)
{
    time_t now = time(NULL);
    struct tm local;
    size_t written;

    if (now == (time_t)-1 || localtime_r(&now, &local) == NULL)
    {
        return false;
    }
    written = strftime(name, name_size, "session_%Y%m%d_%H%M%S", &local);
    return written > 0U;
}

static bool session_append_memory(
    capture_session_t *session,
    const saved_trial_t *trial
)
{
    saved_trial_t *resized;
    size_t new_capacity;

    if (session->trial_count == session->trial_capacity)
    {
        if (session->trial_capacity == 0U)
        {
            new_capacity = 32U;
        }
        else
        {
            if (session->trial_capacity > SIZE_MAX / 2U)
            {
                return false;
            }
            new_capacity = session->trial_capacity * 2U;
        }
        if (new_capacity > SIZE_MAX / sizeof(*session->trials))
        {
            return false;
        }
        resized = realloc(session->trials, new_capacity * sizeof(*session->trials));
        if (resized == NULL)
        {
            return false;
        }
        session->trials = resized;
        session->trial_capacity = new_capacity;
    }
    session->trials[session->trial_count++] = *trial;
    if (trial->trial_id >= session->next_trial_id)
    {
        session->next_trial_id = trial->trial_id == UINT64_MAX
            ? UINT64_MAX
            : trial->trial_id + 1U;
    }
    return true;
}

static bool parse_u64_field(const char *text, uint64_t *value)
{
    char *end = NULL;
    unsigned long long parsed;

    errno = 0;
    parsed = strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0')
    {
        return false;
    }
    *value = (uint64_t)parsed;
    return true;
}

static bool parse_double_field(const char *text, double *value)
{
    char *end = NULL;
    double parsed;

    errno = 0;
    parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed))
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_saved_trial_line(char *line, saved_trial_t *trial)
{
    char *fields[42];
    char *save_pointer = NULL;
    char *token;
    size_t count = 0U;
    unsigned int eligible;
    unsigned int technical_valid;
    unsigned int interval;
    uint64_t id;
    uint64_t integer_value;
    double double_value;

    token = strtok_r(line, ",", &save_pointer);
    while (token != NULL && count < sizeof(fields) / sizeof(fields[0]))
    {
        fields[count++] = token;
        token = strtok_r(NULL, ",", &save_pointer);
    }
    if (count != 42U || token != NULL
        || !parse_u64_field(fields[0], &id)
        || !parse_trial_label(fields[1], &trial->planned_label)
        || !parse_direction_name(fields[2], &trial->planned_direction)
        || !parse_normal_condition_name(
            fields[3],
            &trial->planned_normal_condition
        )
        || !parse_trial_outcome(fields[4], &trial->outcome)
        || !parse_direction_name(fields[5], &trial->actual_direction)
        || fields[6][0] == '\0'
        || !parse_unsigned(fields[7], &eligible)
        || eligible > 1U
        || !parse_unsigned(fields[8], &technical_valid)
        || technical_valid > 1U
        || !parse_double_field(fields[9], &double_value)
        || !parse_u64_field(fields[10], &integer_value)
        || !parse_u64_field(fields[11], &integer_value)
        || !parse_u64_field(fields[12], &integer_value)
        || !parse_u64_field(fields[13], &integer_value)
        || !parse_double_field(fields[14], &trial->peak_raw_accel_mps2)
        || !parse_double_field(fields[15], &trial->peak_linear_accel_mps2)
        || !parse_double_field(fields[16], &trial->peak_raw_jerk_mps3)
        || !parse_double_field(fields[17], &trial->peak_linear_jerk_mps3)
        || !parse_double_field(fields[18], &trial->max_delta_v_mps[0])
        || !parse_double_field(fields[19], &trial->max_delta_v_mps[1])
        || !parse_double_field(fields[20], &trial->max_delta_v_mps[2])
        || !parse_double_field(fields[21], &trial->max_delta_v_mps[3])
        || !parse_double_field(fields[22], &double_value)
        || fields[23][0] == '\0'
        || !parse_u64_field(fields[24], &integer_value)
        || !parse_u64_field(fields[25], &integer_value)
        || !parse_u64_field(fields[26], &integer_value)
        || !parse_u64_field(fields[27], &integer_value)
        || !parse_u64_field(fields[28], &trial->stale_events)
        || !parse_u64_field(fields[29], &integer_value)
        || !parse_u64_field(fields[30], &integer_value)
        || !parse_u64_field(fields[31], &integer_value)
        || !parse_u64_field(fields[32], &integer_value)
        || !parse_u64_field(fields[33], &integer_value)
        || !parse_u64_field(fields[34], &integer_value)
        || !parse_u64_field(fields[35], &integer_value)
        || !parse_unsigned(fields[36], &interval)
        || !parse_unsigned(fields[37], &interval)
        || !parse_unsigned(fields[38], &interval)
        || !parse_u64_field(fields[39], &integer_value)
        || fields[40][0] == '\0'
        || fields[41][0] == '\0')
    {
        return false;
    }
    trial->trial_id = id;
    trial->eligible = eligible != 0U;
    return true;
}

static bool load_session_summary(capture_session_t *session)
{
    char line[CAPTURE_LINE_SIZE];
    bool first_line = true;
    unsigned int line_number = 0U;

    rewind(session->summary_file);
    while (fgets(line, (int)sizeof(line), session->summary_file) != NULL)
    {
        saved_trial_t trial;

        line_number++;
        if (strchr(line, '\n') == NULL && !feof(session->summary_file))
        {
            (void)fprintf(stderr, "summary.csv line %u가 너무 깁니다.\n", line_number);
            return false;
        }
        trim_line(line);
        if (first_line)
        {
            first_line = false;
            if (strncmp(line, "trial_id,", 9U) != 0)
            {
                fputs("기존 summary.csv header가 예상 형식과 다릅니다.\n", stderr);
                return false;
            }
            continue;
        }
        if (line[0] == '\0')
        {
            continue;
        }
        memset(&trial, 0, sizeof(trial));
        if (!parse_saved_trial_line(line, &trial))
        {
            (void)fprintf(
                stderr,
                "summary.csv line %u를 읽지 못했습니다. 손상 방지를 위해 중단합니다.\n",
                line_number
            );
            return false;
        }
        if (!session_append_memory(session, &trial))
        {
            fputs("기존 session summary를 메모리에 올리지 못했습니다.\n", stderr);
            return false;
        }
    }
    if (ferror(session->summary_file) != 0)
    {
        perror("summary.csv read");
        return false;
    }
    return true;
}

static bool capture_session_open(
    capture_session_t *session,
    const char *requested_name
)
{
    char generated_name[CAPTURE_SESSION_NAME_SIZE];
    struct stat status;

    memset(session, 0, sizeof(*session));
    session->next_trial_id = 1U;

    if (requested_name == NULL)
    {
        if (!make_default_session_name(generated_name, sizeof(generated_name)))
        {
            fputs("기본 session 이름을 만들지 못했습니다.\n", stderr);
            return false;
        }
        requested_name = generated_name;
    }
    if (!valid_session_name(requested_name))
    {
        fputs("session 이름은 영문/숫자/점/밑줄/빼기만 사용할 수 있습니다.\n", stderr);
        return false;
    }
    (void)snprintf(session->name, sizeof(session->name), "%s", requested_name);

    if (!ensure_directory(CAPTURE_ROOT_DIRECTORY)
        || !path_join(
            session->directory,
            sizeof(session->directory),
            CAPTURE_ROOT_DIRECTORY,
            session->name
        )
        || !ensure_directory(session->directory)
        || !path_join(
            session->summary_path,
            sizeof(session->summary_path),
            session->directory,
            CAPTURE_SUMMARY_FILE
        ))
    {
        return false;
    }

    if (stat(session->summary_path, &status) == 0)
    {
        session->summary_file = fopen(session->summary_path, "r+");
        if (session->summary_file == NULL)
        {
            return false;
        }
        if (status.st_size == (off_t)0)
        {
            if (fputs(summary_csv_header, session->summary_file) == EOF
                || fflush(session->summary_file) != 0)
            {
                (void)fclose(session->summary_file);
                session->summary_file = NULL;
                return false;
            }
        }
        else if (!load_session_summary(session))
        {
            (void)fclose(session->summary_file);
            session->summary_file = NULL;
            free(session->trials);
            session->trials = NULL;
            return false;
        }
    }
    else if (errno == ENOENT)
    {
        session->summary_file = fopen(session->summary_path, "w+");
        if (session->summary_file == NULL
            || fputs(summary_csv_header, session->summary_file) == EOF
            || fflush(session->summary_file) != 0)
        {
            perror("summary.csv create");
            if (session->summary_file != NULL)
            {
                (void)fclose(session->summary_file);
                session->summary_file = NULL;
            }
            return false;
        }
    }
    else
    {
        perror("summary.csv stat");
        return false;
    }

    return true;
}

static void capture_session_close(capture_session_t *session)
{
    if (session->summary_file != NULL)
    {
        (void)fclose(session->summary_file);
        session->summary_file = NULL;
    }
    free(session->trials);
    session->trials = NULL;
    session->trial_count = 0U;
    session->trial_capacity = 0U;
}

static void sanitize_note(char *note)
{
    size_t index;

    for (index = 0U; note[index] != '\0'; index++)
    {
        unsigned char character = (unsigned char)note[index];

        if (character == (unsigned char)','
            || character == (unsigned char)'\r'
            || character == (unsigned char)'\n')
        {
            note[index] = ' ';
        }
    }
}

static bool write_trial_samples(
    FILE *file,
    const trial_result_t *result,
    const sample_buffer_t *samples
)
{
    size_t index;
    double marker_s = result->marker_ns == 0U
        ? -1.0
        : (double)(result->marker_ns - result->trial_start_ns)
            / (double)NS_PER_SECOND;

    if (fprintf(file, "# planned_label=%s\n", trial_label_name(result->plan.label)) < 0
        || fprintf(file, "# planned_direction=%s\n", direction_name(result->plan.direction)) < 0
        || fprintf(
            file,
            "# planned_normal_condition=%s\n",
            normal_condition_name(result->plan.normal_condition)
        ) < 0
        || fprintf(file, "# outcome=%s\n", trial_outcome_name(result->outcome)) < 0
        || fprintf(file, "# actual_direction=%s\n", direction_name(result->actual_direction)) < 0
        || fprintf(file, "# quality=%s\n", result->quality) < 0
        || fprintf(file, "# eligible=%u\n", result->eligible ? 1U : 0U) < 0
        || fprintf(
            file,
            "# raw_accel_near_rail_reports=%" PRIu64 "\n",
            result->metrics.raw_accel_near_rail_reports
        ) < 0
        || fprintf(
            file,
            "# raw_accel_near_rail_threshold_mps2=%.6f\n",
            CAPTURE_ACCEL_NEAR_RAIL_MPS2
        ) < 0
        || fprintf(file, "# duration_s=%.9f\n", result->duration_s) < 0
        || fprintf(file, "# operator_marker_elapsed_s=%.9f\n", marker_s) < 0
        || fprintf(file, "# note=%s\n", result->note) < 0
        || fputs(
            "elapsed_s,host_monotonic_ns,event_type,units,report_sequence,"
            "generation,accuracy,x,y,z,magnitude,sequence_contiguous,"
            "applied_dt_s,jerk_mps3,dv_20ms_mps,dv_50ms_mps,"
            "dv_100ms_mps,dv_200ms_mps\n",
            file
        ) == EOF)
    {
        return false;
    }

    for (index = 0U; index < samples->count; index++)
    {
        const capture_sample_t *sample = &samples->items[index];

        if (fprintf(
                file,
                "%.9f,%" PRIu64 ",%s,%s,%u,%" PRIu64 ",%u,"
                "%+.9f,%+.9f,%+.9f,%.9f,%u,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%.9f\n",
                sample->elapsed_s,
                sample->monotonic_ns,
                event_type_name(sample->type),
                event_units(sample->type),
                (unsigned int)sample->report_sequence,
                sample->generation,
                (unsigned int)sample->accuracy,
                sample->value.x,
                sample->value.y,
                sample->value.z,
                sample->magnitude,
                sample->sequence_contiguous ? 1U : 0U,
                sample->applied_dt_s,
                sample->jerk_mps3,
                sample->delta_v_mps[0],
                sample->delta_v_mps[1],
                sample->delta_v_mps[2],
                sample->delta_v_mps[3]
            ) < 0)
        {
            return false;
        }
    }
    return ferror(file) == 0;
}

static bool append_summary_row(
    capture_session_t *session,
    const trial_result_t *result,
    const char *data_file_name
)
{
    const capture_metrics_t *metrics = &result->metrics;
    off_t original_size;
    int summary_descriptor = fileno(session->summary_file);
    bool write_ok;

    if (fseek(session->summary_file, 0L, SEEK_END) != 0)
    {
        return false;
    }
    original_size = ftello(session->summary_file);
    if (original_size < (off_t)0 || summary_descriptor < 0)
    {
        return false;
    }
    write_ok = fprintf(
            session->summary_file,
            "%" PRIu64 ",%s,%s,%s,%s,%s,%s,%u,%u,%.9f,%zu,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%s,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu64 ",%s,%s\n",
            result->trial_id,
            trial_label_name(result->plan.label),
            direction_name(result->plan.direction),
            normal_condition_name(result->plan.normal_condition),
            trial_outcome_name(result->outcome),
            direction_name(result->actual_direction),
            result->quality,
            result->eligible ? 1U : 0U,
            result->technical_valid ? 1U : 0U,
            result->duration_s,
            result->sample_count,
            metrics->raw_reports,
            metrics->linear_reports,
            metrics->gyro_reports,
            metrics->peak_raw_accel_mps2,
            metrics->peak_linear_accel_mps2,
            metrics->peak_raw_jerk_mps3,
            metrics->peak_linear_jerk_mps3,
            metrics->max_delta_v_mps[0],
            metrics->max_delta_v_mps[1],
            metrics->max_delta_v_mps[2],
            metrics->max_delta_v_mps[3],
            metrics->peak_gyro_rad_s,
            dominant_axis_name(metrics->peak_linear_accel_vector),
            metrics->sequence_gaps,
            metrics->generation_changes,
            result->cursor_dropped,
            result->startup_losses,
            result->stale_events,
            result->reset_count,
            result->syscall_errors,
            result->validation_errors,
            result->other_transport_errors,
            result->phase2_null_count,
            result->phase2_null_retry_success,
            result->phase2_null_retry_fail,
            result->raw_interval_us,
            result->linear_interval_us,
            result->gyro_interval_us,
            metrics->raw_accel_near_rail_reports,
            result->note,
            data_file_name
        ) >= 0;
    if (fflush(session->summary_file) != 0)
    {
        write_ok = false;
    }
    if (write_ok && fsync(summary_descriptor) != 0)
    {
        write_ok = false;
    }
    if (!write_ok)
    {
        clearerr(session->summary_file);
        (void)ftruncate(summary_descriptor, original_size);
        (void)fseek(session->summary_file, 0L, SEEK_END);
    }
    return write_ok;
}

static bool save_approved_trial(
    capture_session_t *session,
    trial_result_t *result,
    const sample_buffer_t *samples
)
{
    char data_file_name[64];
    char data_path[CAPTURE_PATH_SIZE];
    int descriptor = -1;
    FILE *file = NULL;
    saved_trial_t saved;
    uint64_t candidate_id = session->next_trial_id;
    size_t index;

    for (;;)
    {
        int length = snprintf(
            data_file_name,
            sizeof(data_file_name),
            "trial_%06" PRIu64 ".csv",
            candidate_id
        );

        if (length < 0 || (size_t)length >= sizeof(data_file_name)
            || !path_join(data_path, sizeof(data_path), session->directory, data_file_name))
        {
            return false;
        }
        descriptor = open(
            data_path,
            O_WRONLY | O_CREAT | O_EXCL,
            (mode_t)0644
        );
        if (descriptor >= 0)
        {
            break;
        }
        if (errno != EEXIST || candidate_id == UINT64_MAX)
        {
            (void)fprintf(stderr, "trial CSV 생성 실패: %s\n", strerror(errno));
            return false;
        }
        candidate_id++;
    }

    file = fdopen(descriptor, "w");
    if (file == NULL)
    {
        (void)close(descriptor);
        (void)unlink(data_path);
        return false;
    }

    result->trial_id = candidate_id;
    {
        bool write_ok = write_trial_samples(file, result, samples);

        if (fflush(file) != 0)
        {
            write_ok = false;
        }
        if (write_ok && fsync(fileno(file)) != 0)
        {
            write_ok = false;
        }
        if (fclose(file) != 0)
        {
            write_ok = false;
        }
        file = NULL;
        if (!write_ok)
        {
            (void)unlink(data_path);
            fputs("trial CSV 기록 실패\n", stderr);
            return false;
        }
    }

    if (!append_summary_row(session, result, data_file_name))
    {
        (void)unlink(data_path);
        fputs("summary.csv 기록 실패; trial CSV도 제거했습니다.\n", stderr);
        return false;
    }

    memset(&saved, 0, sizeof(saved));
    saved.trial_id = result->trial_id;
    saved.planned_label = result->plan.label;
    saved.outcome = result->outcome;
    saved.planned_direction = result->plan.direction;
    saved.actual_direction = result->actual_direction;
    saved.planned_normal_condition = result->plan.normal_condition;
    saved.eligible = result->eligible;
    saved.stale_events = result->stale_events;
    saved.peak_raw_accel_mps2 = result->metrics.peak_raw_accel_mps2;
    saved.peak_linear_accel_mps2 = result->metrics.peak_linear_accel_mps2;
    saved.peak_raw_jerk_mps3 = result->metrics.peak_raw_jerk_mps3;
    saved.peak_linear_jerk_mps3 = result->metrics.peak_linear_jerk_mps3;
    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        saved.max_delta_v_mps[index] = result->metrics.max_delta_v_mps[index];
    }
    if (!session_append_memory(session, &saved))
    {
        fputs("CSV는 저장됐지만 현재 session 메모리 summary 추가에 실패했습니다.\n", stderr);
        return false;
    }

    (void)printf("[+] 승인 trial 저장: %s\n", data_path);
    return true;
}

/* ========================================================================== */
/* 실제 trial 수집                                                            */
/* ========================================================================== */

static uint64_t snapshot_timestamp(
    const bno086_snapshot_t *snapshot,
    bno086_event_type_t type
)
{
    switch (type)
    {
        case BNO086_EVENT_ACCELEROMETER:
            return snapshot->last_accel_update_ns;
        case BNO086_EVENT_LINEAR_ACCELERATION:
            return snapshot->last_linear_accel_update_ns;
        case BNO086_EVENT_GYROSCOPE:
            return snapshot->last_gyro_update_ns;
        default:
            return 0U;
    }
}

static uint64_t stale_limit_ns(uint32_t interval_us)
{
    uint64_t limit = (uint64_t)interval_us * UINT64_C(1000) * UINT64_C(10);

    return limit > CAPTURE_STALE_MIN_NS ? limit : CAPTURE_STALE_MIN_NS;
}

static bool snapshot_stream_fresh(
    const bno086_snapshot_t *snapshot,
    bno086_event_type_t type,
    uint64_t now_ns,
    uint32_t interval_us
)
{
    uint64_t timestamp = snapshot_timestamp(snapshot, type);

    return timestamp > 0U
        && now_ns >= timestamp
        && now_ns - timestamp <= stale_limit_ns(interval_us);
}

static bool drain_trial_cursor(
    const imu_worker_t *worker,
    bno086_event_cursor_t *cursor,
    uint64_t start_ns,
    capture_metrics_t *metrics,
    sample_buffer_t *samples
)
{
    bno086_event_t event;
    int result;

    do
    {
        result = bno086_event_next(&worker->imu, cursor, &event, 0);
        if (result > 0
            && !process_capture_event(
                &event,
                start_ns,
                worker->raw_interval_us,
                worker->linear_interval_us,
                worker->gyro_interval_us,
                metrics,
                samples
            ))
        {
            return false;
        }
    }
    while (result > 0);

    return result >= 0;
}

static void finish_trial_diagnostics(
    trial_result_t *result,
    const bno086_diagnostics_t *before,
    const bno086_diagnostics_t *after,
    const bno086_event_cursor_t *cursor
)
{
    result->cursor_dropped = cursor->dropped;
    result->reset_count = counter_difference(after->reset_count, before->reset_count);
    result->syscall_errors = counter_difference(
        after->syscall_bus_error_count,
        before->syscall_bus_error_count
    );
    result->validation_errors = counter_difference(
        after->shtp_validation_error_count,
        before->shtp_validation_error_count
    );
    result->other_transport_errors = counter_difference(
        after->other_transport_error_count,
        before->other_transport_error_count
    );
    result->phase2_null_count = counter_difference(
        after->phase2_null_count,
        before->phase2_null_count
    );
    result->phase2_null_retry_success = counter_difference(
        after->phase2_null_retry_success,
        before->phase2_null_retry_success
    );
    result->phase2_null_retry_fail = counter_difference(
        after->phase2_null_retry_fail,
        before->phase2_null_retry_fail
    );
}

static bool trial_is_technically_valid(
    const trial_result_t *result,
    const sample_buffer_t *samples
)
{
    return !samples->limit_reached
        && result->duration_s > 0.0
        && result->metrics.raw_reports >= 2U
        && result->metrics.linear_reports >= 2U
        && result->metrics.gyro_reports >= 1U
        && result->metrics.raw_accel_near_rail_reports == 0U
        && result->metrics.sequence_gaps == 0U
        && result->metrics.generation_changes == 0U
        && result->cursor_dropped == 0U
        && result->startup_losses == 0U
        && result->stale_events == 0U
        && result->reset_count == 0U
        && result->syscall_errors == 0U
        && result->validation_errors == 0U
        && result->other_transport_errors == 0U
        && result->phase2_null_retry_fail == 0U;
}

static bool run_capture_trial(
    const imu_worker_t *worker,
    const trial_plan_t *plan,
    double maximum_seconds,
    trial_result_t *result,
    sample_buffer_t *samples
)
{
    bno086_event_cursor_t cursor;
    bno086_diagnostics_t diagnostics_before;
    bno086_diagnostics_t diagnostics_after;
    bno086_snapshot_t snapshot;
    uint64_t start_ns;
    uint64_t end_ns;
    uint64_t deadline_ns;
    bool startup_was_ready;
    bool stream_was_fresh[3];
    const bno086_event_type_t monitored_types[3] = {
        BNO086_EVENT_ACCELEROMETER,
        BNO086_EVENT_LINEAR_ACCELERATION,
        BNO086_EVENT_GYROSCOPE
    };
    const uint32_t monitored_intervals[3] = {
        worker->raw_interval_us,
        worker->linear_interval_us,
        worker->gyro_interval_us
    };
    bool complete = false;
    bool aborted = false;
    size_t index;

    memset(result, 0, sizeof(*result));
    memset(samples, 0, sizeof(*samples));
    memset(&cursor, 0, sizeof(cursor));
    memset(&diagnostics_before, 0, sizeof(diagnostics_before));
    memset(&diagnostics_after, 0, sizeof(diagnostics_after));
    memset(&snapshot, 0, sizeof(snapshot));
    result->plan = *plan;
    result->outcome = plan->label == TRIAL_LABEL_COLLISION
        ? TRIAL_OUTCOME_IMPACT
        : TRIAL_OUTCOME_NORMAL;
    result->actual_direction = plan->label == TRIAL_LABEL_COLLISION
        ? plan->direction
        : IMPACT_DIRECTION_UNKNOWN;
    result->raw_interval_us = worker->raw_interval_us;
    result->linear_interval_us = worker->linear_interval_us;
    result->gyro_interval_us = worker->gyro_interval_us;

    if (bno086_event_cursor_init(&worker->imu, &cursor, false) <= 0
        || bno086_get_diagnostics(&worker->imu, &diagnostics_before) <= 0
        || bno086_get_snapshot(&worker->imu, &snapshot) <= 0)
    {
        fputs("trial cursor/diagnostic 초기화 실패\n", stderr);
        return false;
    }

    start_ns = monotonic_ns();
    result->trial_start_ns = start_ns;
    deadline_ns = start_ns + (uint64_t)(maximum_seconds * (double)NS_PER_SECOND);
    capture_metrics_initialize(
        &result->metrics,
        (double)worker->linear_interval_us / 1000000.0
    );
    startup_was_ready = snapshot.startup_ready;
    for (index = 0U; index < 3U; index++)
    {
        stream_was_fresh[index] = snapshot_stream_fresh(
            &snapshot,
            monitored_types[index],
            start_ns,
            monitored_intervals[index]
        );
    }

    puts("\n[RECORDING] RC카를 운전하십시오.");
    puts("  주행/충돌이 끝나는 즉시 차량을 만지거나 들어 올리기 전에 Enter를 누르십시오.");
    puts("  종료 뒤 손으로 잡은 spike가 섞이면 해당 trial을 폐기/재시험해야 합니다.");
    puts("  Enter       : trial 즉시 종료");
    puts("  m + Enter   : 충돌 순간을 수동 표시하고 계속 기록");
    puts("  q + Enter   : 현재 trial 취소");
    fflush(stdout);

    while (stop_requested == 0 && monotonic_ns() < deadline_ns)
    {
        struct pollfd input;
        uint64_t now_ns;
        int poll_result;

        if (!drain_trial_cursor(
                worker,
                &cursor,
                start_ns,
                &result->metrics,
                samples
            ))
        {
            fputs("[!] cursor drain 또는 sample memory 실패\n", stderr);
            aborted = true;
            break;
        }

        now_ns = monotonic_ns();
        if (bno086_get_snapshot(&worker->imu, &snapshot) <= 0)
        {
            aborted = true;
            break;
        }
        if (startup_was_ready && !snapshot.startup_ready)
        {
            result->startup_losses++;
        }
        startup_was_ready = snapshot.startup_ready;
        for (index = 0U; index < 3U; index++)
        {
            bool fresh = snapshot_stream_fresh(
                &snapshot,
                monitored_types[index],
                now_ns,
                monitored_intervals[index]
            );

            if (stream_was_fresh[index] && !fresh)
            {
                result->stale_events++;
            }
            stream_was_fresh[index] = fresh;
        }

        input.fd = STDIN_FILENO;
        input.events = POLLIN;
        input.revents = 0;
        poll_result = poll(&input, 1, 2);
        if (poll_result < 0)
        {
            if (errno != EINTR)
            {
                perror("poll");
                aborted = true;
                break;
            }
        }
        else if (poll_result > 0 && (input.revents & POLLIN) != 0)
        {
            char command[32];

            if (fgets(command, (int)sizeof(command), stdin) == NULL)
            {
                aborted = true;
                break;
            }
            if (strchr(command, '\n') == NULL)
            {
                int character;

                do
                {
                    character = getchar();
                }
                while (character != '\n' && character != EOF);
            }
            trim_line(command);
            if (command[0] == '\0'
                || strcmp(command, "s") == 0
                || strcmp(command, "stop") == 0)
            {
                complete = true;
                break;
            }
            if (strcmp(command, "m") == 0)
            {
                result->marker_ns = monotonic_ns();
                (void)printf(
                    "[MARK] %.3f s\n",
                    (double)(result->marker_ns - start_ns)
                        / (double)NS_PER_SECOND
                );
            }
            else if (strcmp(command, "q") == 0)
            {
                aborted = true;
                break;
            }
            else
            {
                puts("명령은 Enter, m, q 중 하나입니다.");
            }
        }
        else if (poll_result > 0
                 && (input.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        {
            fputs("stdin 연결이 종료되어 trial을 취소합니다.\n", stderr);
            aborted = true;
            break;
        }
    }

    if (!complete && !aborted && stop_requested == 0)
    {
        puts("[*] 설정한 최대 trial 시간이 되어 자동 종료했습니다.");
        complete = true;
    }
    (void)drain_trial_cursor(
        worker,
        &cursor,
        start_ns,
        &result->metrics,
        samples
    );
    end_ns = monotonic_ns();
    result->duration_s = (double)(end_ns - start_ns) / (double)NS_PER_SECOND;
    result->sample_count = samples->count;
    (void)bno086_get_diagnostics(&worker->imu, &diagnostics_after);
    finish_trial_diagnostics(
        result,
        &diagnostics_before,
        &diagnostics_after,
        &cursor
    );
    result->technical_valid = trial_is_technically_valid(result, samples);
    return complete && !aborted && stop_requested == 0;
}

static void print_peak_time_reference(
    const char *name,
    double peak_elapsed_s,
    const trial_result_t *result
)
{
    double before_end_s = result->duration_s - peak_elapsed_s;

    if (before_end_s < 0.0)
    {
        before_end_s = 0.0;
    }
    (void)printf(
        "  %-18s start=%7.3fs  end_before=%7.3fs",
        name,
        peak_elapsed_s,
        before_end_s
    );
    if (result->marker_ns != 0U
        && result->marker_ns >= result->trial_start_ns)
    {
        double marker_elapsed_s =
            (double)(result->marker_ns - result->trial_start_ns)
            / (double)NS_PER_SECOND;

        (void)printf("  marker_delta=%+7.3fs", peak_elapsed_s - marker_elapsed_s);
    }
    putchar('\n');
}

static void print_trial_result(const trial_result_t *result)
{
    const capture_metrics_t *metrics = &result->metrics;
    size_t index;

    puts("\n=== TRIAL RESULT ===");
    (void)printf(
        "plan=%s planned_direction=%s normal_condition=%s "
        "duration=%.3fs samples=%zu\n",
        trial_label_name(result->plan.label),
        direction_name(result->plan.direction),
        normal_condition_name(result->plan.normal_condition),
        result->duration_s,
        result->sample_count
    );
    (void)printf(
        "applied_FC raw=%" PRIu32 "us linear=%" PRIu32
        "us gyro=%" PRIu32 "us\n",
        result->raw_interval_us,
        result->linear_interval_us,
        result->gyro_interval_us
    );
    (void)printf(
        "reports raw=%" PRIu64 " linear=%" PRIu64 " gyro=%" PRIu64 "\n",
        metrics->raw_reports,
        metrics->linear_reports,
        metrics->gyro_reports
    );
    (void)printf(
        "raw_accel_near_rail=%" PRIu64 " (axis threshold=%.3f m/s^2)\n",
        metrics->raw_accel_near_rail_reports,
        CAPTURE_ACCEL_NEAR_RAIL_MPS2
    );
    if (metrics->raw_accel_near_rail_reports > 0U)
    {
        puts("[!] +-8 g 측정 한계 근처의 sample이 있어 threshold 후보에서 제외합니다.");
    }
    (void)printf(
        "peak_raw_accel=%.4f m/s^2 at %.3fs vector=(%+.4f,%+.4f,%+.4f)\n",
        metrics->peak_raw_accel_mps2,
        metrics->peak_raw_accel_time_s,
        metrics->peak_raw_accel_vector.x,
        metrics->peak_raw_accel_vector.y,
        metrics->peak_raw_accel_vector.z
    );
    (void)printf(
        "peak_linear_accel=%.4f m/s^2 at %.3fs vector=(%+.4f,%+.4f,%+.4f)"
        " dominant=%s\n",
        metrics->peak_linear_accel_mps2,
        metrics->peak_linear_accel_time_s,
        metrics->peak_linear_accel_vector.x,
        metrics->peak_linear_accel_vector.y,
        metrics->peak_linear_accel_vector.z,
        dominant_axis_name(metrics->peak_linear_accel_vector)
    );
    (void)printf(
        "peak_jerk raw=%.4f linear=%.4f m/s^3\n",
        metrics->peak_raw_jerk_mps3,
        metrics->peak_linear_jerk_mps3
    );
    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        (void)printf(
            "max_deltaV_%ums=%.5f m/s at %.3fs\n",
            (unsigned int)lround(delta_v_window_seconds[index] * 1000.0),
            metrics->max_delta_v_mps[index],
            metrics->max_delta_v_time_s[index]
        );
    }
    (void)printf(
        "peak_gyro=%.4f rad/s (%.2f deg/s) at %.3fs\n",
        metrics->peak_gyro_rad_s,
        metrics->peak_gyro_rad_s * (180.0 / 3.14159265358979323846),
        metrics->peak_gyro_time_s
    );
    (void)printf(
        "sequence_gaps=%" PRIu64 " generation_changes=%" PRIu64
        " cursor_dropped=%" PRIu64 "\n",
        metrics->sequence_gaps,
        metrics->generation_changes,
        result->cursor_dropped
    );
    (void)printf(
        "reset=%" PRIu64 " startup_losses=%" PRIu64 " stale=%" PRIu64
        " syscall=%" PRIu64 " validation=%" PRIu64 " other=%" PRIu64 "\n",
        result->reset_count,
        result->startup_losses,
        result->stale_events,
        result->syscall_errors,
        result->validation_errors,
        result->other_transport_errors
    );
    (void)printf(
        "phase2_null=%" PRIu64 " retry_success=%" PRIu64
        " retry_fail=%" PRIu64 "\n",
        result->phase2_null_count,
        result->phase2_null_retry_success,
        result->phase2_null_retry_fail
    );
    puts("\n[PEAK TIME REVIEW]");
    puts("end_before가 매우 작으면 종료 후 차량을 만진 동작일 수 있습니다.");
    if (result->marker_ns != 0U
        && result->marker_ns >= result->trial_start_ns)
    {
        (void)printf(
            "operator_marker=%.3fs from start\n",
            (double)(result->marker_ns - result->trial_start_ns)
                / (double)NS_PER_SECOND
        );
    }
    else
    {
        puts("operator_marker=NONE");
    }
    print_peak_time_reference(
        "raw accel",
        metrics->peak_raw_accel_time_s,
        result
    );
    print_peak_time_reference(
        "linear accel",
        metrics->peak_linear_accel_time_s,
        result
    );
    print_peak_time_reference(
        "raw jerk",
        metrics->peak_raw_jerk_time_s,
        result
    );
    print_peak_time_reference(
        "linear jerk",
        metrics->peak_linear_jerk_time_s,
        result
    );
    for (index = 0U; index < CAPTURE_WINDOW_COUNT; index++)
    {
        char name[32];

        (void)snprintf(
            name,
            sizeof(name),
            "deltaV %ums",
            (unsigned int)lround(delta_v_window_seconds[index] * 1000.0)
        );
        print_peak_time_reference(name, metrics->max_delta_v_time_s[index], result);
    }
    print_peak_time_reference("gyro", metrics->peak_gyro_time_s, result);
    (void)printf(
        "technical_valid=%s\n",
        result->technical_valid ? "YES" : "NO"
    );
    if (!result->technical_valid)
    {
        puts("[!] 보관은 가능하지만 threshold 후보 계산에서는 제외됩니다.");
    }
}

/* ========================================================================== */
/* Operator review                                                            */
/* ========================================================================== */

static bool prompt_direction(
    const char *title,
    impact_direction_t default_direction,
    impact_direction_t *selected
)
{
    char input[32];

    for (;;)
    {
        (void)printf(
            "%s [1 front / 2 rear / 3 left / 4 right / 5 diagonal / "
            "6 unknown, Enter=%s]: ",
            title,
            direction_name(default_direction)
        );
        fflush(stdout);
        if (!read_line_prompt(NULL, input, sizeof(input)))
        {
            return false;
        }
        if (input[0] == '\0')
        {
            *selected = default_direction;
            return true;
        }
        if (strcmp(input, "1") == 0)
        {
            *selected = IMPACT_DIRECTION_FRONT;
            return true;
        }
        if (strcmp(input, "2") == 0)
        {
            *selected = IMPACT_DIRECTION_REAR;
            return true;
        }
        if (strcmp(input, "3") == 0)
        {
            *selected = IMPACT_DIRECTION_LEFT;
            return true;
        }
        if (strcmp(input, "4") == 0)
        {
            *selected = IMPACT_DIRECTION_RIGHT;
            return true;
        }
        if (strcmp(input, "5") == 0)
        {
            *selected = IMPACT_DIRECTION_DIAGONAL;
            return true;
        }
        if (strcmp(input, "6") == 0)
        {
            *selected = IMPACT_DIRECTION_UNKNOWN;
            return true;
        }
        puts("방향 번호를 다시 입력하십시오.");
    }
}

static bool prompt_normal_condition(normal_condition_t *selected)
{
    char input[32];

    for (;;)
    {
        puts("정상 주행 조건을 선택하십시오.");
        puts("  1 stationary         : 정지/공회전");
        puts("  2 constant_straight  : 일정 속도 직진");
        puts("  3 acceleration       : 정상 가속");
        puts("  4 braking            : 정상/급제동");
        puts("  5 turning            : 선회");
        puts("  6 road_bump          : 턱/요철 통과");
        if (!read_line_prompt("선택: ", input, sizeof(input)))
        {
            return false;
        }
        if (strcmp(input, "1") == 0)
        {
            *selected = NORMAL_CONDITION_STATIONARY;
            return true;
        }
        if (strcmp(input, "2") == 0)
        {
            *selected = NORMAL_CONDITION_STRAIGHT;
            return true;
        }
        if (strcmp(input, "3") == 0)
        {
            *selected = NORMAL_CONDITION_ACCELERATION;
            return true;
        }
        if (strcmp(input, "4") == 0)
        {
            *selected = NORMAL_CONDITION_BRAKING;
            return true;
        }
        if (strcmp(input, "5") == 0)
        {
            *selected = NORMAL_CONDITION_TURNING;
            return true;
        }
        if (strcmp(input, "6") == 0)
        {
            *selected = NORMAL_CONDITION_ROAD_BUMP;
            return true;
        }
        puts("정상 주행 조건 번호를 다시 입력하십시오.");
    }
}

static bool prompt_outcome(trial_outcome_t *outcome)
{
    char input[32];

    for (;;)
    {
        puts("실제 결과를 선택하십시오.");
        puts("  1 normal                 : 충돌/접촉 없는 정상 주행");
        puts("  2 impact                 : 사고 후보로 인정할 충돌");
        puts("  3 non_accident_contact   : 너무 약하거나 스친 접촉");
        puts("  4 invalid                : 조종 실수/시험 조건 실패");
        if (!read_line_prompt("선택: ", input, sizeof(input)))
        {
            return false;
        }
        if (strcmp(input, "1") == 0)
        {
            *outcome = TRIAL_OUTCOME_NORMAL;
            return true;
        }
        if (strcmp(input, "2") == 0)
        {
            *outcome = TRIAL_OUTCOME_IMPACT;
            return true;
        }
        if (strcmp(input, "3") == 0)
        {
            *outcome = TRIAL_OUTCOME_NON_ACCIDENT_CONTACT;
            return true;
        }
        if (strcmp(input, "4") == 0)
        {
            *outcome = TRIAL_OUTCOME_INVALID;
            return true;
        }
        puts("결과 번호를 다시 입력하십시오.");
    }
}

static void assign_approval_quality(trial_result_t *result)
{
    result->approved = true;
    if (!result->technical_valid)
    {
        result->eligible = false;
        (void)snprintf(result->quality, sizeof(result->quality), "technical_invalid");
    }
    else if (result->outcome == TRIAL_OUTCOME_INVALID)
    {
        result->eligible = false;
        (void)snprintf(result->quality, sizeof(result->quality), "operator_invalid");
    }
    else if (result->outcome == TRIAL_OUTCOME_NON_ACCIDENT_CONTACT)
    {
        /*
         * 약한 접촉은 positive impact에서는 제외하되 boundary-negative로
         * 보존한다. 실제 impact threshold가 이런 접촉을 넘도록 비교한다.
         */
        result->eligible = true;
        (void)snprintf(result->quality, sizeof(result->quality), "boundary_contact");
    }
    else
    {
        result->eligible = true;
        (void)snprintf(result->quality, sizeof(result->quality), "valid");
    }
}

static bool prompt_note(trial_result_t *result)
{
    if (!read_line_prompt(
            "메모(선택, 예: 약한 충돌/조종 실수/비스듬함): ",
            result->note,
            sizeof(result->note)
        ))
    {
        return false;
    }
    sanitize_note(result->note);
    if (result->note[0] == '\0')
    {
        (void)snprintf(result->note, sizeof(result->note), "-");
    }
    return true;
}

static review_action_t review_trial(trial_result_t *result)
{
    char input[32];

    if (result->plan.label == TRIAL_LABEL_COLLISION
        && !prompt_direction(
            "실제로 충돌한 방향",
            result->plan.direction,
            &result->actual_direction
        ))
    {
        return REVIEW_EXIT;
    }

    for (;;)
    {
        puts("\n[중요] PEAK TIME을 보고 종료 뒤 차량을 잡거나 들어 올린 spike가");
        puts("       섞였으면 k를 누르지 말고 d/r 또는 i를 선택하십시오.");
        (void)printf(
            "\n현재 분류: outcome=%s actual_direction=%s\n",
            trial_outcome_name(result->outcome),
            direction_name(result->actual_direction)
        );
        puts("  k : 현재 분류로 승인/저장");
        puts("  l : 실제 outcome/direction 재분류");
        puts("  w : 약한 충돌/스침 -> NON_ACCIDENT_CONTACT로 승인");
        puts("  i : 조종 실수/시험 실패 -> INVALID로 승인 보관");
        puts("  d : 완전히 폐기 (파일에 저장하지 않음)");
        puts("  r : 폐기하고 같은 계획으로 즉시 재시험");
        puts("  q : 폐기하고 프로그램 종료");

        if (!read_line_prompt("판정: ", input, sizeof(input)))
        {
            return REVIEW_EXIT;
        }
        if (strcmp(input, "k") == 0)
        {
            assign_approval_quality(result);
            if (!prompt_note(result))
            {
                return REVIEW_EXIT;
            }
            return REVIEW_KEEP;
        }
        if (strcmp(input, "w") == 0)
        {
            result->outcome = TRIAL_OUTCOME_NON_ACCIDENT_CONTACT;
            if (!prompt_direction(
                    "실제 접촉 방향",
                    result->actual_direction,
                    &result->actual_direction
                ))
            {
                return REVIEW_EXIT;
            }
            assign_approval_quality(result);
            if (!prompt_note(result))
            {
                return REVIEW_EXIT;
            }
            return REVIEW_KEEP;
        }
        if (strcmp(input, "i") == 0)
        {
            result->outcome = TRIAL_OUTCOME_INVALID;
            result->actual_direction = IMPACT_DIRECTION_UNKNOWN;
            assign_approval_quality(result);
            if (!prompt_note(result))
            {
                return REVIEW_EXIT;
            }
            return REVIEW_KEEP;
        }
        if (strcmp(input, "l") == 0)
        {
            if (!prompt_outcome(&result->outcome))
            {
                return REVIEW_EXIT;
            }
            if (result->outcome == TRIAL_OUTCOME_NORMAL
                || result->outcome == TRIAL_OUTCOME_INVALID)
            {
                result->actual_direction = IMPACT_DIRECTION_UNKNOWN;
            }
            else if (!prompt_direction(
                "실제 접촉/충돌 방향",
                result->actual_direction,
                &result->actual_direction
            ))
            {
                return REVIEW_EXIT;
            }
            continue;
        }
        if (strcmp(input, "d") == 0)
        {
            return REVIEW_DISCARD;
        }
        if (strcmp(input, "r") == 0)
        {
            return REVIEW_RETRY;
        }
        if (strcmp(input, "q") == 0)
        {
            return REVIEW_EXIT;
        }
        puts("k/l/w/i/d/r/q 중 하나를 입력하십시오.");
    }
}

/* ========================================================================== */
/* Session coverage and threshold-candidate display                           */
/* ========================================================================== */

static double saved_metric_value(const saved_trial_t *trial, size_t metric)
{
    switch (metric)
    {
        case 0U:
            return trial->peak_raw_accel_mps2;
        case 1U:
            return trial->peak_linear_accel_mps2;
        case 2U:
            return trial->peak_raw_jerk_mps3;
        case 3U:
            return trial->peak_linear_jerk_mps3;
        default:
            return trial->max_delta_v_mps[metric - 4U];
    }
}

static void print_session_summary(
    const capture_session_t *session,
    const program_options_t *options
)
{
    static const char *const metric_names[] = {
        "peak raw accel (m/s^2)",
        "peak linear accel (m/s^2)",
        "peak raw jerk (m/s^3)",
        "peak linear jerk (m/s^3)",
        "max deltaV 20ms (m/s)",
        "max deltaV 50ms (m/s)",
        "max deltaV 100ms (m/s)",
        "max deltaV 200ms (m/s)"
    };
    uint64_t outcome_counts[4] = {0U, 0U, 0U, 0U};
    uint64_t impact_directions[IMPACT_DIRECTION_COUNT] = {
        0U, 0U, 0U, 0U, 0U, 0U
    };
    uint64_t normal_conditions[NORMAL_CONDITION_COUNT] = {
        0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    uint64_t valid_normal_count = 0U;
    uint64_t valid_negative_count = 0U;
    uint64_t valid_impact_count = 0U;
    unsigned int covered_directions = 0U;
    unsigned int covered_normal_conditions = 0U;
    size_t index;
    size_t metric;
    bool coverage_ready;

    for (index = 0U; index < session->trial_count; index++)
    {
        const saved_trial_t *trial = &session->trials[index];

        outcome_counts[(size_t)trial->outcome]++;
        if (!trial->eligible)
        {
            continue;
        }
        if (trial->outcome == TRIAL_OUTCOME_NORMAL)
        {
            valid_normal_count++;
            valid_negative_count++;
            normal_conditions[(size_t)trial->planned_normal_condition]++;
        }
        else if (trial->outcome == TRIAL_OUTCOME_NON_ACCIDENT_CONTACT)
        {
            valid_negative_count++;
        }
        else if (trial->outcome == TRIAL_OUTCOME_IMPACT)
        {
            valid_impact_count++;
            impact_directions[(size_t)trial->actual_direction]++;
        }
    }

    for (index = 0U; index < (size_t)IMPACT_DIRECTION_UNKNOWN; index++)
    {
        if (impact_directions[index]
            >= (uint64_t)options->minimum_per_direction)
        {
            covered_directions++;
        }
    }
    for (index = 0U; index < (size_t)NORMAL_CONDITION_UNKNOWN; index++)
    {
        if (normal_conditions[index] > 0U)
        {
            covered_normal_conditions++;
        }
    }

    puts("\n=== SESSION SUMMARY ===");
    (void)printf("session=%s saved_trials=%zu\n", session->name, session->trial_count);
    (void)printf(
        "NORMAL=%" PRIu64 " CONTACT=%" PRIu64 " IMPACT=%" PRIu64
        " INVALID=%" PRIu64 "\n",
        outcome_counts[(size_t)TRIAL_OUTCOME_NORMAL],
        outcome_counts[(size_t)TRIAL_OUTCOME_NON_ACCIDENT_CONTACT],
        outcome_counts[(size_t)TRIAL_OUTCOME_IMPACT],
        outcome_counts[(size_t)TRIAL_OUTCOME_INVALID]
    );
    (void)printf(
        "eligible normal=%" PRIu64 " boundary-negative(total)=%" PRIu64
        " impact=%" PRIu64 "\n",
        valid_normal_count,
        valid_negative_count,
        valid_impact_count
    );
    puts("유효 NORMAL 주행 조건 coverage:");
    for (index = 0U; index < (size_t)NORMAL_CONDITION_COUNT; index++)
    {
        (void)printf(
            "  %-18s : %" PRIu64 "%s\n",
            normal_condition_name((normal_condition_t)index),
            normal_conditions[index],
            index < (size_t)NORMAL_CONDITION_UNKNOWN
                && normal_conditions[index] > 0U
                ? "  COVERED"
                : ""
        );
    }
    puts("유효 IMPACT 실제 방향 coverage:");
    for (index = 0U; index < (size_t)IMPACT_DIRECTION_COUNT; index++)
    {
        (void)printf(
            "  %-8s : %" PRIu64 "%s\n",
            direction_name((impact_direction_t)index),
            impact_directions[index],
            index < (size_t)IMPACT_DIRECTION_UNKNOWN
                && impact_directions[index]
                    >= (uint64_t)options->minimum_per_direction
                ? "  COVERED"
                : ""
        );
    }

    coverage_ready =
        valid_normal_count >= (uint64_t)options->minimum_normal_trials
        && covered_normal_conditions >= options->minimum_normal_conditions
        && covered_directions >= options->minimum_directions
        && valid_impact_count > 0U;

    (void)printf(
        "candidate gate: normal>=%u, normal_conditions>=%u, "
        "impact_directions>=%u, each_direction>=%u : %s\n",
        options->minimum_normal_trials,
        options->minimum_normal_conditions,
        options->minimum_directions,
        options->minimum_per_direction,
        coverage_ready ? "READY" : "NOT READY"
    );
    if (!coverage_ready)
    {
        puts("임계값 후보를 표시하지 않습니다. 실제 유효 데이터 coverage가 부족합니다.");
        return;
    }

    puts("\n[수동 검토용 분리 후보]");
    puts("다음 숫자는 코드를 변경하지 않으며 자동 확정값도 아닙니다.");
    for (metric = 0U;
         metric < sizeof(metric_names) / sizeof(metric_names[0]);
         metric++)
    {
        double negative_max = -DBL_MAX;
        double impact_min = DBL_MAX;
        bool negative_seen = false;
        bool impact_seen = false;

        for (index = 0U; index < session->trial_count; index++)
        {
            const saved_trial_t *trial = &session->trials[index];
            double value;

            if (!trial->eligible)
            {
                continue;
            }
            value = saved_metric_value(trial, metric);
            if (trial->outcome == TRIAL_OUTCOME_NORMAL
                || trial->outcome == TRIAL_OUTCOME_NON_ACCIDENT_CONTACT)
            {
                if (!negative_seen || value > negative_max)
                {
                    negative_max = value;
                }
                negative_seen = true;
            }
            else if (trial->outcome == TRIAL_OUTCOME_IMPACT)
            {
                if (!impact_seen || value < impact_min)
                {
                    impact_min = value;
                }
                impact_seen = true;
            }
        }

        if (negative_seen && impact_seen && negative_max < impact_min)
        {
            (void)printf(
                "  %-29s negative_max=%.6f impact_min=%.6f"
                " midpoint=%.6f  SEPARATED\n",
                metric_names[metric],
                negative_max,
                impact_min,
                (negative_max + impact_min) * 0.5
            );
        }
        else
        {
            (void)printf("  %-29s OVERLAP - 후보 표시 안 함\n", metric_names[metric]);
        }
    }
}

/* ========================================================================== */
/* Hardware-free implementation self-test                                     */
/* ========================================================================== */

static bool nearly_equal(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static int run_self_test(void)
{
    capture_metrics_t metrics;
    sample_buffer_t samples;
    bno086_event_t event;
    uint64_t start_ns = UINT64_C(1000000000);
    char summary_line[] =
        "42,normal,unknown,braking,normal,unknown,valid,1,1,1.0,100,"
        "10,10,10,9.0,2.0,3.0,4.0,0.1,0.2,0.3,0.4,5.0,+X,"
        "0,0,0,0,7,0,0,0,0,0,0,0,10000,10000,10000,0,-,trial.csv\n";
    saved_trial_t parsed_trial;
    bool passed = true;

    memset(&samples, 0, sizeof(samples));
    memset(&event, 0, sizeof(event));
    memset(&parsed_trial, 0, sizeof(parsed_trial));
    capture_metrics_initialize(&metrics, 0.010);
    event.type = BNO086_EVENT_LINEAR_ACCELERATION;
    event.bno_generation = 1U;

    event.report_sequence = 1U;
    event.monotonic_ns = start_ns;
    event.x = 0.0;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    event.report_sequence = 2U;
    event.monotonic_ns += UINT64_C(10000000);
    event.x = 1.0;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    event.report_sequence = 3U;
    event.monotonic_ns += UINT64_C(10000000);
    event.x = 1.0;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    /*
     * sequence 4를 일부러 건너뛴다. 값이 크게 바뀌어도 gap 경계에서는
     * jerk/delta-V를 이어 계산하지 않아야 한다.
     */
    event.report_sequence = 5U;
    event.monotonic_ns += UINT64_C(10000000);
    event.x = 100.0;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    event.report_sequence = 6U;
    event.monotonic_ns += UINT64_C(10000000);
    event.x = 100.0;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    event.type = BNO086_EVENT_ACCELEROMETER;
    event.report_sequence = 1U;
    event.monotonic_ns += UINT64_C(10000000);
    event.x = CAPTURE_ACCEL_NEAR_RAIL_MPS2;
    passed = process_capture_event(
        &event, start_ns, 10000U, 10000U, 10000U, &metrics, &samples
    ) && passed;

    passed = passed
        && samples.count == 6U
        && metrics.raw_reports == 1U
        && metrics.linear_reports == 5U
        && metrics.raw_accel_near_rail_reports == 1U
        && metrics.sequence_gaps == 1U
        && nearly_equal(metrics.peak_linear_jerk_mps3, 100.0, 1.0e-9)
        && nearly_equal(metrics.max_delta_v_mps[0], 0.015, 1.0e-9)
        && parse_saved_trial_line(summary_line, &parsed_trial)
        && parsed_trial.trial_id == 42U
        && parsed_trial.planned_normal_condition == NORMAL_CONDITION_BRAKING
        && parsed_trial.outcome == TRIAL_OUTCOME_NORMAL
        && parsed_trial.actual_direction == IMPACT_DIRECTION_UNKNOWN
        && parsed_trial.eligible
        && parsed_trial.stale_events == 7U
        && nearly_equal(parsed_trial.max_delta_v_mps[3], 0.4, 1.0e-9);

    puts("=== COLLISION CAPTURE SELF-TEST ===");
    puts("이 self-test의 인공 값은 구현 검사용이며 임계값 자료가 아닙니다.");
    (void)printf(
        "FC_dt=0.010000s sequence_gap=%" PRIu64
        " peak_jerk=%.6f deltaV20=%.6f near_rail=%" PRIu64 "\n",
        metrics.sequence_gaps,
        metrics.peak_linear_jerk_mps3,
        metrics.max_delta_v_mps[0],
        metrics.raw_accel_near_rail_reports
    );
    (void)printf(
        "resume_parser normal_condition=%s deltaV200=%.6f\n",
        normal_condition_name(parsed_trial.planned_normal_condition),
        parsed_trial.max_delta_v_mps[3]
    );
    (void)printf("RESULT=%s\n", passed ? "PASS" : "FAIL");
    sample_buffer_release(&samples);
    return passed ? 0 : 1;
}

/* ========================================================================== */
/* CLI and interactive session                                                */
/* ========================================================================== */

static void print_usage(FILE *stream, const char *program)
{
    (void)fprintf(
        stream,
        "Usage:\n"
        "  %s [--session NAME] [--min-normal N]\n"
        "     [--min-normal-conditions N] [--min-directions N]\n"
        "     [--min-per-direction N] [--max-duration SECONDS]\n"
        "  %s --self-test\n\n"
        "기본 candidate gate: normal 6개, 서로 다른 정상조건 6개,\n"
        "충돌방향 4개, 각 방향 2개입니다.\n"
        "같은 --session NAME을 다시 사용하면 기존 summary.csv를 읽어 이어갑니다.\n",
        program,
        program
    );
}

static bool parse_options(
    int argc,
    char **argv,
    program_options_t *options
)
{
    int index;

    memset(options, 0, sizeof(*options));
    options->minimum_normal_trials = 6U;
    options->minimum_normal_conditions = 6U;
    options->minimum_directions = 4U;
    options->minimum_per_direction = 2U;
    options->maximum_trial_seconds = CAPTURE_DEFAULT_MAX_SECONDS;

    for (index = 1; index < argc; index++)
    {
        const char *argument = argv[index];

        if (strcmp(argument, "--self-test") == 0)
        {
            options->self_test = true;
        }
        else if (strcmp(argument, "--session") == 0 && index + 1 < argc)
        {
            options->session_name = argv[++index];
        }
        else if (strcmp(argument, "--min-normal") == 0 && index + 1 < argc)
        {
            if (!parse_unsigned(argv[++index], &options->minimum_normal_trials)
                || options->minimum_normal_trials == 0U)
            {
                return false;
            }
        }
        else if (strcmp(argument, "--min-directions") == 0 && index + 1 < argc)
        {
            if (!parse_unsigned(argv[++index], &options->minimum_directions)
                || options->minimum_directions == 0U
                || options->minimum_directions
                    > (unsigned int)IMPACT_DIRECTION_UNKNOWN)
            {
                return false;
            }
        }
        else if (strcmp(argument, "--min-normal-conditions") == 0
                 && index + 1 < argc)
        {
            if (!parse_unsigned(
                    argv[++index],
                    &options->minimum_normal_conditions
                )
                || options->minimum_normal_conditions == 0U
                || options->minimum_normal_conditions
                    > (unsigned int)NORMAL_CONDITION_UNKNOWN)
            {
                return false;
            }
        }
        else if (strcmp(argument, "--min-per-direction") == 0
                 && index + 1 < argc)
        {
            if (!parse_unsigned(argv[++index], &options->minimum_per_direction)
                || options->minimum_per_direction == 0U)
            {
                return false;
            }
        }
        else if (strcmp(argument, "--max-duration") == 0 && index + 1 < argc)
        {
            if (!parse_positive_double(
                    argv[++index],
                    &options->maximum_trial_seconds
                )
                || options->maximum_trial_seconds > CAPTURE_MAX_SECONDS)
            {
                return false;
            }
        }
        else if (strcmp(argument, "--help") == 0
                 || strcmp(argument, "-h") == 0)
        {
            options->help = true;
        }
        else
        {
            return false;
        }
    }

    if (options->self_test && argc != 2)
    {
        return false;
    }
    if (options->help && argc != 2)
    {
        return false;
    }
    return true;
}

static bool choose_planned_trial(trial_label_t label, trial_plan_t *plan)
{
    plan->label = label;
    plan->direction = IMPACT_DIRECTION_UNKNOWN;
    plan->normal_condition = NORMAL_CONDITION_UNKNOWN;
    if (label == TRIAL_LABEL_COLLISION)
    {
        return prompt_direction(
            "계획한 충돌 방향",
            IMPACT_DIRECTION_FRONT,
            &plan->direction
        );
    }
    return prompt_normal_condition(&plan->normal_condition);
}

static int run_interactive_session(const program_options_t *options)
{
    capture_session_t session;
    imu_worker_t worker;
    bool worker_started = false;
    bool session_opened = false;
    bool keep_running = true;
    char input[32];
    int exit_status = 0;

    memset(&session, 0, sizeof(session));
    memset(&worker, 0, sizeof(worker));

    puts("=== BNO086 RC CAR COLLISION CAPTURE ===");
    puts("실제 정상 주행/접촉/충돌 값을 모으는 계측기입니다.");
    puts("사람이나 단단한 위험물에 충돌시키지 말고 저속과 완충재부터 시작하십시오.");
    puts("프로그램은 production threshold나 source code를 자동 변경하지 않습니다.");

    puts("\n[*] BNO086 startup 및 production feature 설정 중...");
    if (!imu_worker_start(&worker))
    {
        (void)fprintf(
            stderr,
            "[!] BNO086 준비 실패: %s\n",
            worker.start_error[0] != '\0' ? worker.start_error : "unknown"
        );
        imu_worker_stop(&worker);
        return 1;
    }
    worker_started = true;

    if (!capture_session_open(&session, options->session_name))
    {
        imu_worker_stop(&worker);
        return 1;
    }
    session_opened = true;

    (void)printf(
        "[+] BNO086 ready: raw=%" PRIu32 "us linear=%" PRIu32
        "us gyro=%" PRIu32 "us\n",
        worker.raw_interval_us,
        worker.linear_interval_us,
        worker.gyro_interval_us
    );
    (void)printf(
        "[+] session=%s resumed_trials=%zu output=%s\n",
        session.name,
        session.trial_count,
        session.directory
    );

    while (keep_running && stop_requested == 0)
    {
        trial_plan_t plan;
        bool retry;

        puts("\n[N] 정상 주행 trial");
        puts("[C] 충돌 계획 trial");
        puts("[S] session coverage/후보 확인");
        puts("[Q] 저장 후 종료");
        if (!read_line_prompt("선택: ", input, sizeof(input)))
        {
            break;
        }
        if (strcmp(input, "s") == 0 || strcmp(input, "S") == 0)
        {
            print_session_summary(&session, options);
            continue;
        }
        if (strcmp(input, "q") == 0 || strcmp(input, "Q") == 0)
        {
            break;
        }
        if (strcmp(input, "n") == 0 || strcmp(input, "N") == 0)
        {
            if (!choose_planned_trial(TRIAL_LABEL_NORMAL, &plan))
            {
                break;
            }
        }
        else if (strcmp(input, "c") == 0 || strcmp(input, "C") == 0)
        {
            if (!choose_planned_trial(TRIAL_LABEL_COLLISION, &plan))
            {
                break;
            }
        }
        else
        {
            puts("N/C/S/Q 중 하나를 입력하십시오.");
            continue;
        }

        retry = true;
        while (retry && keep_running && stop_requested == 0)
        {
            trial_result_t result;
            sample_buffer_t samples;
            review_action_t action;

            retry = false;
            (void)printf(
                "\n계획: %s / direction=%s / normal_condition=%s\n",
                trial_label_name(plan.label),
                direction_name(plan.direction),
                normal_condition_name(plan.normal_condition)
            );
            puts("주행/충돌 완료 즉시 차량을 만지거나 들어 올리기 전에 Enter를 누르십시오.");
            if (!read_line_prompt(
                    "차량과 시험장을 준비한 뒤 Enter (b=메뉴 복귀): ",
                    input,
                    sizeof(input)
                ))
            {
                keep_running = false;
                break;
            }
            if (strcmp(input, "b") == 0 || strcmp(input, "B") == 0)
            {
                break;
            }

            if (!run_capture_trial(
                    &worker,
                    &plan,
                    options->maximum_trial_seconds,
                    &result,
                    &samples
                ))
            {
                sample_buffer_release(&samples);
                if (stop_requested != 0)
                {
                    keep_running = false;
                }
                else
                {
                    puts("[*] 취소된 trial은 어떤 파일에도 저장하지 않았습니다.");
                }
                break;
            }

            print_trial_result(&result);
            action = review_trial(&result);
            if (action == REVIEW_KEEP)
            {
                if (!save_approved_trial(&session, &result, &samples))
                {
                    fputs("[!] 승인 trial 저장 실패\n", stderr);
                    exit_status = 1;
                }
                else
                {
                    print_session_summary(&session, options);
                }
            }
            else if (action == REVIEW_RETRY)
            {
                puts("[*] 현재 trial은 저장하지 않고 같은 계획으로 재시험합니다.");
                retry = true;
            }
            else if (action == REVIEW_DISCARD)
            {
                puts("[*] 현재 trial을 저장하지 않고 폐기했습니다.");
            }
            else
            {
                puts("[*] 현재 trial을 저장하지 않고 종료합니다.");
                keep_running = false;
            }
            sample_buffer_release(&samples);
        }
    }

    if (session_opened)
    {
        print_session_summary(&session, options);
        capture_session_close(&session);
    }
    if (worker_started)
    {
        imu_worker_stop(&worker);
    }
    puts("수집을 종료했습니다. production threshold는 변경하지 않았습니다.");
    return exit_status;
}

int main(int argc, char **argv)
{
    program_options_t options;
    struct sigaction action;

    if (!parse_options(argc, argv, &options))
    {
        print_usage(stderr, argv[0]);
        return 2;
    }
    if (options.help)
    {
        print_usage(stdout, argv[0]);
        return 0;
    }
    if (options.self_test)
    {
        return run_self_test();
    }

    memset(&action, 0, sizeof(action));
    action.sa_handler = handle_signal;
    (void)sigemptyset(&action.sa_mask);
    (void)sigaction(SIGINT, &action, NULL);
    (void)sigaction(SIGTERM, &action, NULL);
    return run_interactive_session(&options);
}
