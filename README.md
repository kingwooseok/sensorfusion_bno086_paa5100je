# BNO086 + PAA5100JE Sensor Fusion

Raspberry Pi에서 개발한 BNO086 IMU + PAA5100JE optical-flow 위치 융합 구현을 보존하는 독립 저장소다. C/pthread 구현, 이식 기준이 된 Python 구현, 센서 진단 및 회귀시험을 함께 담는다.

 이 저장소는 기존 센서 코드를 계속 검토하고 재사용하기 위한 보존본이며, ArUco 구현은 포함하지 않는다. GitHub의 저장소 잠금(Archive) 기능을 적용하는 의미는 아니다.

원본 작업 경로는 `~/pmw3901_optical_flow`다. 새 저장소용 README, 빌드/제외 설정, calibration 예제 외에 `.c`, `.h`, `.py` 파일은 기존 내용을 그대로 보존한다. 대회 프로젝트의 파일과 IPC 소스는 복사하지 않는다.

### 실측에서 확인한 한계

사용자의 실제 RC카 왕복 시험에서는 PAA raw tick이 저속에서 잘 상쇄되었지만, 속도를 높이면 같은 거리에서도 tick 합계와 왕복 대칭성이 크게 달라졌다. 약 30 cm/s 부근부터 오차가 커졌다는 관측이 있었으며, 이는 당시 바닥·높이·조명·장착 조건에서의 결과다. 센서의 공식 속도 한계로 일반화하거나 정확한 위치 추적 성능을 보증하지 않는다.

BNO086의 안정적인 report 수신, 수학/모의시험 통과와 실제 주행 위치 정확도는 구분한다. 아래의 원래 설계 및 시험 설명은 재사용을 위한 기록이다.

### 독립 저장소에서 시작하기

```bash
git clone https://github.com/kingwooseok/sensorfusion_bno086_paa5100je.git
cd sensorfusion_bno086_paa5100je
make check-core
make check-python
```

Linux, C11 컴파일러, `make`, pthread/libm을 사용한다. `check-core`는 센서 하드웨어와 대회 프로젝트 IPC 없이 C 드라이버·융합·충돌 수집기의 기존 모의시험을 실행한다. Python 시험은 Python 3와 `lgpio`, `spidev` 모듈이 필요하다. Raspberry Pi OS에서는 필요할 때 `sudo apt install build-essential python3-lgpio python3-spidev`로 준비할 수 있다. 이 코드는 Raspberry Pi/Linux 센서 구현이며 Windows ArUco 프로그램으로 이식한 것은 아니다.

`make all`, `make check-c`, `make check`의 차량 프로세스 부분에는 외부 ACE IPC 소스가 필요하다. 해당 경로를 `ACE_IPC_DIR`로 지정하는 방법은 아래 Build 절에 설명한다. `make check-core`의 성공이 외부 프로세스와의 통합 검증까지 의미하지는 않는다.

실기기를 사용할 때 `config.json`이 아직 없다면 다음처럼 예제를 복사한다.

```bash
cp -n config.example.json config.json
```

예제는 보존 당시의 높이/scaler 값이며 다른 차량의 보정값이 아니다. 실제 센서 높이와 거리 보정에 맞게 설정해야 한다. 기존 `config.json`은 유지하며 Git에는 포함하지 않는다. 실행 파일, Python cache, GPIO 임시 pipe, 충돌 수집 자료와 로그도 Git에서 제외한다.

---

## 기존 Sensor Process 설계

이 모듈은 차량에 연결된 **BNO086 IMU**와 **PAA5100JE Optical Flow Sensor**를 읽고, 차량 역할에 맞는 센서 정보를 다른 프로세스로 전달한다.

관찰차량과 사고차량은 필요한 센서, 계산 내용, IPC 목적지가 서로 다르다. 따라서 하나의 실행 파일에서 mode만 바꾸는 구조가 아니라 다음 두 프로세스로 분리한다.

| 실행 파일 | 차량 역할 | 사용 센서 | 주요 출력 | IPC 목적지 |
|---|---|---|---|---|
| `witness_sensor` | 관찰차량 | BNO086 | 고정 X/Y와 현재 관측 Heading | Witness Filter |
| `accident_sensor` | 사고차량 | BNO086 + PAA5100JE | 위치 추정 및 충돌 발생 이벤트 | P2P |

이 구현은 원래 `2026ESWContest_mobility_ACE/sensor` 통합을 준비하며 개발했다. 보존된 차량 프로세스의 IPC 자료형과 송수신 함수는 외부 프로젝트의 `ipc` 모듈을 참조한다. 아래의 역할과 IPC 계약은 당시 구현에 대한 설명이며, 현재 ArUco 방식으로 전환한 프로젝트의 구조를 뜻하지 않는다.

---

## 1. 차량별 역할

### 1.1 관찰차량

관찰차량은 테스트베드의 정해진 위치에 고정하여 사용한다. 따라서 자신의 위치를 센서로 다시 측정할 필요가 없다.

프로세스가 수행하는 일은 다음과 같다.

1. BNO086 Rotation Vector와 Magnetometer를 활성화한다.
2. 동적 calibration을 시작하고 RV/Mag accuracy가 모두 2 이상이 될 때까지 기다린다.
3. 사용자가 차량을 테스트베드의 지정 방향에 맞춘 뒤 Enter를 누른다.
4. Enter 이후 새로 들어온 quaternion을 상대 Heading의 0° 기준으로 저장한다.
5. 하드코딩된 X/Y 좌표와 현재 Heading을 `vehicle_state_t`로 만든다.
6. `/tmp/witness_sensor.sock`을 통해 Witness Filter로 20 Hz 주기로 전송한다.

```text
                        관찰차량

                  BNO086 RV / Mag
                         │
                         ▼
                 상대 Heading 계산
                         │
          ┌──────────────┴──────────────┐
          │                             │
          ▼                             ▼
  하드코딩된 고정 X/Y             현재 관측 방향
          │                             │
          └──────────────┬──────────────┘
                         ▼
                   vehicle_state_t
                         │
                         │ /tmp/witness_sensor.sock
                         ▼
                   Witness Filter
```

관찰차량에서는 다음 기능을 사용하지 않는다.

- PAA5100JE Optical Flow
- 위치 적분
- 충돌 판정
- P2P 사고 이벤트 송신

### 1.2 사고차량

사고차량은 움직이므로 현재 위치를 계속 추정해야 하며, 동시에 가속도 데이터로 충돌 여부도 판단해야 한다.

프로세스가 수행하는 일은 다음과 같다.

1. PAA5100JE에서 차량 body 좌표계의 이동량을 읽는다.
2. BNO086 Rotation Vector와 Gyroscope로 이동 당시의 차량 방향을 구한다.
3. body 좌표의 이동량을 테스트베드 world 좌표로 회전하여 X/Y 위치에 적분한다.
4. BNO086 Raw/Linear Acceleration에서 acceleration, jerk, delta-V 지표를 계산한다.
5. 실측으로 정한 여러 충돌 조건이 가까운 시간 안에 함께 성립하는지 판단한다.
6. 충돌 rising edge가 발생하면 그 순간의 위치와 Heading을 사고 payload에 넣는다.
7. `/tmp/p2p.sock`을 통해 P2P 프로세스로 전달한다.

```text
                             사고차량

        PAA5100JE                              BNO086
       motion burst               ┌──────────────┴──────────────┐
            │                      │                             │
            │                      ▼                             ▼
            │               최신 IMU Snapshot            Acceleration Events
            │                      │                             │
            └──────────┬───────────┘                             │
                       ▼                                         ▼
              Position Fusion                         Collision Detector
                       │                                         │
                       ▼                                         │
             Current X/Y/Heading ────────────────────────────────┘
                                                                 │
                                                                 ▼
                                                     accident_event_payload_t
                                                                 │
                                                                 │ /tmp/p2p.sock
                                                                 ▼
                                                           P2P Process
```

사고차량에는 Witness Filter가 없다. 위치와 충돌을 센서 프로세스 내부에서 계산하고, 사고가 판정된 경우에만 P2P 프로세스로 이벤트를 보낸다.

---

## 2. Thread Architecture

### 2.1 관찰차량 Thread

```text
                 witness_sensor
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
    BNO I/O Thread             Main / IPC Thread
          │                         │
   SH-2 packet drain          Snapshot 복사
   Feature 설정              상대 Heading 계산
   Dynamic calibration       Filter 재연결/송신
```

BNO I/O Thread만 BNO086 장치를 직접 소유한다. Main Thread는 드라이버가 mutex로 보호하는 snapshot만 복사한다.

이렇게 분리하면 Witness Filter가 종료되었거나 IPC 송신이 지연되더라도 BNO packet은 계속 읽을 수 있다. IPC 문제 때문에 센서 report가 밀려 stale 상태가 되는 것을 방지하기 위한 구조다.

### 2.2 사고차량 Thread

```text
                         accident_sensor
                                │
       ┌────────────────────────┼────────────────────────┐
       ▼                        ▼                        ▼
 BNO I/O Thread          Main / Position Thread    Collision Thread
       │                        │                        │
 SH-2 packet drain       PAA 단독 소유              Event cursor
 Snapshot 갱신           Position Fusion            충돌 지표 계산
 Event ring 기록         위치 publication           Rising-edge 판정
                                                          │
                                                          ▼
                                                   IPC Sender Thread
                                                          │
                                                   P2P 재연결/송신
```

각 Thread의 센서 및 자료 소유권은 다음과 같다.

| 대상 | 소유자 | 다른 Thread의 접근 방법 |
|---|---|---|
| BNO086 I2C/GPIO | BNO I/O Thread | Snapshot 또는 Event Cursor |
| PAA5100JE SPI | Main / Position Thread | 직접 공유하지 않음 |
| 현재 X/Y/Heading | Main / Position Thread | Mutex로 보호된 사본 |
| 충돌 판정 상태 | Collision Thread | 직접 공유하지 않음 |
| P2P 소켓 | IPC Sender Thread | Bounded Queue로 payload 전달 |

충돌 판정과 IPC 송신을 분리한 이유는 P2P 프로세스가 아직 실행되지 않았거나 소켓 송신이 느릴 때도 가속도 sample을 계속 소비하기 위해서다. Queue가 가득 차면 센서 Thread를 정지시키지 않고 drop 횟수를 기록한다.

---

## 3. BNO086 데이터 공유 방식

BNO086 드라이버는 같은 센서 데이터를 두 가지 형태로 제공한다.

### Snapshot

Snapshot은 각 report 종류의 **가장 최근 값 한 벌**이다.

위치 계산처럼 “현재 자세와 각속도가 무엇인가?”가 필요한 경우 사용한다. 오래된 중간 sample을 모두 처리할 필요가 없어 짧게 mutex를 잡고 구조체 하나를 복사한다.

### SPMC Event Ring

Event Ring은 report를 들어온 순서대로 저장한다. 충돌 판정 Thread는 독립 cursor를 사용하여 Raw/Linear Acceleration sample을 차례대로 읽는다.

Jerk는 앞뒤 sample의 차이를 사용하고 delta-V는 일정 시간 동안의 sample을 적분하므로 중간 sample을 임의로 건너뛰면 안 된다. Ring이 가득 차 cursor가 sample을 놓치면 `dropped`가 증가하며, 충돌 판정기는 이전 jerk/delta-V 상태를 초기화하고 새 구간을 시작한다.

```text
                         BNO I/O Thread
                               │
                 ┌─────────────┴─────────────┐
                 ▼                           ▼
          Latest Snapshot               Event Ring
                 │                           │
                 ▼                           ▼
          Position Thread             Collision Thread
```

`bno_generation`은 BNO086 reset 또는 새 startup 경계마다 증가한다. Generation이 다른 두 sample은 물리적으로 연속된 자료로 간주하지 않는다. Reset 전후 값을 빼서 거대한 가짜 jerk나 Heading 변화로 만드는 것을 막기 위해서다.

---

## 4. 위치 융합

PAA5100JE는 절대 좌표를 측정하지 않는다. 직전 read 이후 바닥 무늬가 센서에서 얼마나 움직였는지를 `dx/dy tick`으로 알려준다.

거리 환산은 다음 관계를 사용한다.

```text
distance_cm = ticks × height_cm / scaler
```

예를 들어 `100 tick`, 높이 `3.5 cm`, scaler `400.8`이면 계산값은 약 `0.873 cm`다. 이는 계산 예일 뿐이고 실제 scaler는 PAA5100JE를 연결한 뒤 알려진 거리를 왕복 이동하여 정해야 한다.

### Body 좌표와 World 좌표

PAA 이동량은 차량 기준 body 좌표다. 차량이 회전하면 같은 “전진”이라도 테스트베드에서는 다른 방향의 이동이 된다.

```text
PAA body displacement
        │
        ▼
현재 BNO relative Heading
        │
        ▼
2-D body → world rotation
        │
        ▼
testbed X/Y 누적
```

한 motion burst는 다음 순서로 처리한다.

1. Raw tick을 진단 누계에 기록한다.
2. 아주 작은 정지 진동은 deadband로 0 처리한다.
3. SQUAL이 낮으면 위치에는 반영하지 않는다.
4. BNO Rotation Vector와 Gyroscope가 fresh한지 확인한다.
5. 이전 Heading과 현재 Heading의 가장 짧은 변화량을 계산한다.
6. 두 Heading의 midpoint를 해당 이동의 대표 방향으로 사용한다.
7. 학습된 lever-arm 회전 flow가 있으면 body 이동에서 감산한다.
8. body 이동을 world X/Y로 회전하여 누적한다.

### Accuracy와 Stale의 차이

두 상태는 같은 의미가 아니다.

- `accuracy < 2`: SH-2의 현재 calibration 품질 등급이 낮아진 상태
- `stale`: 새 센서 report가 제한 시간 안에 도착하지 않은 상태

시작 기준을 잡을 때는 RV/Mag accuracy가 모두 2 이상이어야 한다. 그러나 운행 중 accuracy가 잠시 `2 → 1`로 떨어졌다는 이유만으로 바로 flow를 버리거나 Heading을 재기준화하지 않는다. 저하 횟수와 지속 시간만 진단값으로 기록한다.

반대로 Rotation Vector 또는 Gyroscope가 stale이면 이동 방향을 알 수 없으므로 해당 flow는 적분하지 않는다. Fresh 상태가 회복된 첫 sample에서는 stale 구간 전체의 Heading 차이를 사용하지 않고 현재 Heading으로 다시 연결한다.

### Lever-arm Calibration

PAA5100JE가 차량의 정확한 회전 중심에서 떨어져 있으면 제자리 회전만 해도 센서가 작은 원호를 이동한다. PAA에는 이것이 translation처럼 보일 수 있다.

운행 중 `c`를 누르면 이 회전 flow를 학습한다.

1. 첫 `c`: calibration 시작
2. 차량을 제자리에서 CW/CCW 양방향으로 회전
3. calibration 중 PAA burst는 계속 읽지만 좌표에는 적분하지 않음
4. 다시 `c`: 표본 수와 양방향 조건을 확인하고 보정값 고정
5. 종료 시 fresh BNO Heading으로 `last_flow_heading`을 다시 설정
6. 마지막 PAA delta를 flush한 뒤 정상 주행으로 복귀

따라서 calibration에서 크게 회전해도 좌표가 움직이지 않으며, calibration 후 첫 직선 이동이 보정 전후 전체 Heading 차이의 midpoint 영향을 받지 않는다.

---

## 5. 충돌 판정

충돌 판정은 BNO086의 다음 두 feature를 사용한다.

- Accelerometer: 중력을 포함한 Raw Acceleration
- Linear Acceleration: SH-2가 중력 성분을 제거한 Acceleration

현재 구현이 계산할 수 있는 지표는 다음과 같다.

| 지표 | 단위 | 의미 |
|---|---:|---|
| Raw Acceleration | m/s² | 중력을 포함하여 순간적으로 받은 전체 가속도 크기 |
| Linear Acceleration | m/s² | 중력을 제외한 차량 운동 가속도 크기 |
| Raw Jerk | m/s³ | Raw Acceleration이 얼마나 갑자기 바뀌었는지 |
| Linear Jerk | m/s³ | Linear Acceleration이 얼마나 갑자기 바뀌었는지 |
| Linear delta-V | m/s | 짧은 시간창에서 Linear Acceleration을 적분한 속도 변화량 |

하나의 peak만으로 충돌을 판정하지 않는다. `collision_config_t`에서 선택한 지표 중 `minimum_evidence_count`개 이상이 `coincidence_window_ms` 안에서 함께 임계값을 넘어야 충돌 후보가 된다.

한 번 충돌이 검출되면 latch하여 같은 충격의 진동 peak를 여러 사고로 전송하지 않는다. 다음 사고를 받을 수 있으려면 cooldown 시간이 끝나고, 추가 evidence가 없는 quiet 시간도 지나야 한다.

### 현재 기본 상태

`accident_sensor.c`의 `collision_config`는 의도적으로 다음 상태다.

```text
enabled = false
metric_mask = 0
thresholds = 0
```

RC카의 실제 정상 주행과 충돌 데이터가 아직 확정되지 않았으므로 충돌 IPC는 기본 비활성이다. 임계값을 추측하여 활성화하지 않는다.

실기기 데이터를 검토한 후 다음 항목을 확정해야 한다.

- 사용할 지표 조합인 `metric_mask`
- 동시에 필요한 최소 지표 수
- 각 acceleration, jerk, delta-V 임계값
- delta-V 시간창
- coincidence window
- cooldown 및 quiet rearm 시간
- 유효 위치가 있을 때만 전송할지 여부

---

## 6. 좌표계와 설정값

테스트베드의 로컬 좌표는 millimeter 단위를 사용한다.

```text
              0° / +Y
                 ↑
                 │
270° / -X  ←─────┼─────→  90° / +X
                 │
                 ↓
             180° / -Y
```

Heading은 0°가 +Y이고 시계방향으로 증가하는 Witness Filter 규약에 맞춘다.

### 관찰차량 설정

`witness_sensor.c`에서 차량별로 다음 상수를 설정한다.

```c
WITNESS_FIXED_X_MM
WITNESS_FIXED_Y_MM
WITNESS_INITIAL_HEADING_DEG
WITNESS_RELATIVE_HEADING_SIGN
```

- `WITNESS_FIXED_X_MM`, `WITNESS_FIXED_Y_MM`: 테스트베드에 고정한 관찰차량 위치
- `WITNESS_INITIAL_HEADING_DEG`: 사용자가 Enter를 누를 때 차량이 바라보는 테스트베드 방향
- `WITNESS_RELATIVE_HEADING_SIGN`: BNO 장착축과 테스트베드 Heading 증가 방향을 맞추는 `+1` 또는 `-1`

### 사고차량 설정

`accident_sensor.c`에서 다음 상수를 설정한다.

```c
ACCIDENT_START_X_MM
ACCIDENT_START_Y_MM
ACCIDENT_START_HEADING_DEG
ACCIDENT_FUSION_TO_TESTBED_DEG
ACCIDENT_RELATIVE_HEADING_SIGN
```

융합 내부에서는 출발 위치를 `(0, 0)` cm, 출발 방향을 상대 Heading 0°로 사용한다. 위 상수들은 이 상대 좌표를 실제 테스트베드 좌표로 공개할 때 적용한다.

### PAA 거리 설정

`config.json`에는 다음 두 값이 저장된다.

```json
{
    "height_cm": 2.35,
    "scaler": 549.7581782929655
}
```

- `height_cm`: PAA5100JE 렌즈와 바닥 사이 높이
- `scaler`: tick을 실제 거리로 바꾸는 실측 환산값

`accident_sensor`의 시작 메뉴에서 높이 또는 scaler calibration을 변경하면 이 파일에 저장한다. 차량 구조나 센서 높이가 바뀌면 기존 scaler를 그대로 신뢰하지 말고 다시 측정한다.

---

## 7. 하드웨어 인터페이스

### BNO086

기본 설정은 다음과 같다.

| 항목 | 값 |
|---|---|
| I2C device | `/dev/i2c-1` |
| I2C address | `0x4B` |
| H_INTN | GPIO line 18, active-low |
| Reset | GPIO line 23, active-low |

사고차량에서 사용하는 report 주기는 다음과 같다.

| Report | 요청 주기 | 목표 rate |
|---|---:|---:|
| Rotation Vector | 10,000 us | 약 100 Hz |
| Gyroscope | 10,000 us | 약 100 Hz |
| Magnetometer | 20,000 us | 약 50 Hz |
| Accelerometer | 10,000 us | 약 100 Hz |
| Linear Acceleration | 10,000 us | 약 100 Hz |

관찰차량은 Rotation Vector 100 Hz와 Magnetometer 50 Hz만 사용한다.

SH-2가 요청 rate를 실제 지원 주기로 조정할 수 있으므로 코드에서는 요청값과 정확히 같은지보다 FC에서 확인된 실제 interval이 non-zero인지 확인한다. 이후 실제 Channel 3/4 report가 들어오는 것까지 확인해야 feature 설정 성공으로 처리한다.

### PAA5100JE

기본 SPI 설정은 다음과 같다.

| 항목 | 값 |
|---|---|
| SPI device | `/dev/spidev0.1` |
| SPI mode | Mode 3 |
| Clock | 2 MHz |
| MOSI | Raspberry Pi physical pin 19 |
| MISO | Raspberry Pi physical pin 21 |
| SCLK | Raspberry Pi physical pin 23 |
| CE | Raspberry Pi physical pin 26, CE1 |

Register read와 motion burst는 address와 data 사이에 turnaround delay가 필요하다. 구현은 두 segment를 하나의 `SPI_IOC_MESSAGE(2)`로 보내므로 delay 동안에도 hardware CS가 LOW로 유지된다.

초기화 시 Product ID `0x49`와 inverse Product ID `0xB6`을 모두 확인한다. 둘 중 하나라도 다르면 다른 SPI 장치, 잘못된 배선 또는 floating MISO일 수 있으므로 시작하지 않는다.

> 전원 전압과 전원 배선은 사용 중인 PAA5100JE breakout/module 사양을 확인한다. Raspberry Pi GPIO 번호만 보고 임의 전압을 연결하지 않는다.

---

## 8. 파일 구성

### Production C 소스

| 파일 | 역할 |
|---|---|
| `witness_sensor.c` | 관찰차량 BNO Heading 및 Witness Filter IPC |
| `accident_sensor.c` | 사고차량 위치 융합, 충돌 판정, P2P IPC |
| `bno086.c`, `bno086.h` | BNO086 SH-2/SHTP I2C 드라이버 |
| `paa5100je.c`, `paa5100je.h` | PAA5100JE Linux spidev 드라이버 |
| `position_fusion.c`, `position_fusion.h` | 상대 Heading, body/world 변환, 위치 적분, lever-arm 보정 |
| `Makefile` | 핵심 독립 시험, 외부 IPC 연동 프로세스 빌드 및 회귀시험 |
| `config.example.json` | 보존 당시 PAA 높이/scaler 예제 |
| `config.json` | 실행 환경별 PAA 보정값; 로컬에만 유지 |

### 실측 및 회귀시험

| 파일/디렉터리 | 역할 |
|---|---|
| `test_collision_capture.c` | 실제 RC카 정상 주행/충돌 자료 수집 및 수동 검토 |
| `scratch/` | C/Python 회귀시험과 BNO 진단 도구 |

### Python 기준 구현

`bno086.py`, `pmw3901.py`, `read_flow.py`, `calibrate.py`는 C 이식의 기준이 된 기존 구현이다. Production 프로세스는 C 파일이며 Python 코드는 알고리즘 비교와 실기기 진단에 남겨 둔다.

C 소스에서는 실제 부품명에 맞춰 `paa5100je` 이름을 사용한다. 기존 Python 파일의 `pmw3901` 이름은 기준 구현과 기존 시험 호환성을 위해 유지한다.

---

## 9. Build

보존된 차량 프로세스의 빌드는 외부 프로젝트의 IPC 소스를 기본적으로 다음 상대 경로에서 참조한다. 이 저장소에는 해당 파일을 포함하지 않는다.

```text
../2026ESWContest_mobility_ACE/ipc
```

위 경로에 호환되는 `ipc.c`, `ipc.h`, `protocol.h`, `p2p_protocol.h`, `sensor_protocol.h`가 준비되어 있으면 두 실행 파일을 빌드한다.

```bash
cd ~/sensorfusion_bno086_paa5100je
make all
```

생성 결과:

```text
./witness_sensor
./accident_sensor
```

외부 IPC가 다른 위치에 있다면 경로를 명시한다. 기존 프로젝트의 `sensor` 디렉터리에서 빌드하는 경우 `../ipc`를 사용할 수 있었다.

```bash
make ACE_IPC_DIR=../ipc all
```

Linux 사용자가 `/dev/i2c-1`, `/dev/gpiochip0`, `/dev/spidev0.1`에 접근할 권한이 있어야 한다. 권한 오류와 실제 센서 통신 오류를 혼동하지 않도록 실행 전에 장치 파일과 사용자 그룹 설정을 확인한다.

---

## 10. 실행 방법

### 10.1 관찰차량

먼저 차량별 테스트베드 X/Y와 초기 방향 상수를 확정한다.

```bash
./witness_sensor
```

실행 순서:

1. BNO086 startup과 RV/Mag feature 설정을 기다린다.
2. 센서를 주변 자성체와 전원선에서 가능한 한 떨어뜨린다.
3. 천천히 여러 축으로 움직여 RV/Mag accuracy를 모두 2 이상으로 올린다.
4. 차량을 `WITNESS_INITIAL_HEADING_DEG` 방향에 맞춘다.
5. Enter를 누른다.
6. 프로그램은 Enter 이후의 새 quaternion을 기준으로 사용한다.
7. Witness Filter가 없으면 0.5초 간격으로 재연결하며 BNO 수신은 계속한다.

BNO reset, startup loss 또는 RV/Mag stale이 발생하면 마지막 Heading을 계속 보내지 않는다. IPC 전송을 중지하고 사용자가 차량을 지정 방향에 다시 맞춰 reference를 잡을 때까지 기다린다.

운행 중 accuracy가 순간적으로 2 미만이 된 것만으로는 reference를 폐기하지 않는다.

### 10.2 사고차량

사고차량은 PAA5100JE와 BNO086이 모두 연결되어야 한다.

```bash
./accident_sensor
```

시작 메뉴:

```text
[Enter/1] config.json의 저장된 height/scaler를 그대로 사용해 시작
[2] 알려진 거리로 PAA5100JE scaler 보정 후 시작
[3] 센서 높이 변경 후 시작
```

평소 반복 시험에서는 시작 메뉴에서 숫자를 입력하지 않고 Enter만 누른다.
그러면 높이를 다시 묻거나 거리 이동 보정을 실행하지 않고 이전에 저장된 값을
그대로 사용한다. 2번 또는 3번을 명시적으로 선택했을 때만 보정값을 변경하며,
변경 결과는 다음 실행을 위해 `config.json`에 다시 저장된다.

메뉴 선택과 BNO 동적 보정이 끝나면 위치 적분 전에 실제 Heading 방향을 한 번
확인한다. 차량을 출발 자세에 놓고 기준을 잡은 다음, 차량 위에서 내려다봤을 때
차량 전체를 반시계(CCW) 방향으로 약 90° 돌린다. 프로그램은 기대값 `+90°`와
실측 상대 Heading 및 오차를 출력하며, 사람 손으로 돌리는 오차를 고려해
`+90° ±15°`일 때만 통과시킨다. 반대 부호나 큰 축 오차가 나오면 위치 융합을
시작하지 않는다.

보정 단계에서 RV/Mag 2 이상을 한 번 확인한 뒤에는 accuracy가 일시적으로
2 미만이 되어도 품질값을 경고로 표시할 뿐 fresh RV Heading을 숨기거나 점검을
중단하지 않는다. 90° 회전 및 최종 복귀 중 순간 하락은 실제 운행 중 정책과
동일하게 다루며, stale 또는 startup loss와는 구분한다.

검사가 통과하면 차량을 원래 출발 방향으로 되돌린다. 복귀 Heading도 0°에서
±15° 안인지 확인한 뒤, 복귀 시점의 fresh quaternion을 최종 Heading 0°로
다시 저장한다. 90° 시험 중 PAA에 누적된 delta는 기존 시작 flush에서 버리므로
시험 회전이 실제 좌표에 적분되지는 않는다.

융합이 시작되면 사용할 수 있는 명령은 다음과 같다.

| 명령 | 동작 |
|---|---|
| `r` | 위치 원점과, 가능하면 fresh Heading 기준을 재설정 |
| `c` | Lever-arm calibration 시작 또는 종료 시도 |
| `q` | 측정 종료 및 최종 요약 출력 |
| `Ctrl-C` | 안전 종료 |

사고차량은 PAA 초기화에 실패하거나 BNO feature/report 확인에 실패하면 한 센서만 사용하는 mode로 강등하지 않고 종료한다. 잘못된 위치나 충돌 정보를 정상 데이터처럼 보내지 않기 위한 동작이다.

현재 `collision_config.enabled`가 `false`이므로 위치 융합은 실행되지만 실제 충돌 IPC는 발생하지 않는다. RC카 실측으로 임계값을 확정한 뒤에만 활성화한다.

---

## 11. RC카 충돌 데이터 수집

`test_collision_capture.c`는 production 사고 판정기가 아니라 임계값을 정하기 위한 실기기 계측 프로그램이다. 인공 파형을 사고 데이터로 사용하지 않고 실제 RC카를 정상 주행하거나 완충재에 저속 충돌시켜 값을 수집한다.

### Build

```bash
gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror \
    -I. test_collision_capture.c bno086.c -pthread -lm \
    -o test_collision_capture
```

### 코드 동작 Self-test

```bash
./test_collision_capture --self-test
```

Self-test의 인공 값은 계산과 상태 전이 검사용이다. Production 임계값 자료로 사용하지 않는다.

### 실제 수집 Session

```bash
./test_collision_capture --session rc_test_01
```

같은 session 이름을 다시 사용하면 기존 `summary.csv`를 읽고 이어서 수집한다.

메뉴에서 다음 trial을 선택할 수 있다.

- 정상 주행: 정지, 일정 속도 직진, 정상 가속, 제동, 선회, 턱 통과
- 충돌: 전방, 후방, 좌측, 우측, 대각선 등 계획 방향

기록 중 조작:

| 입력 | 동작 |
|---|---|
| `Enter` | 현재 trial 기록 종료 |
| `m` + Enter | 실제 충돌 순간을 수동 표시하고 기록 계속 |
| `q` + Enter | 현재 trial 취소 |

기록이 끝나면 사용자가 결과를 직접 검토한다.

| 입력 | 의미 |
|---|---|
| `k` | 현재 분류로 승인하고 저장 |
| `l` | 실제 outcome 또는 방향 재분류 |
| `w` | 약한 충돌/스침으로 승인 |
| `i` | 조종 실수나 시험 실패 자료로 보관 |
| `d` | 완전히 폐기하여 저장하지 않음 |
| `r` | 폐기하고 같은 계획으로 다시 시험 |
| `q` | 폐기하고 프로그램 종료 |

수집 결과는 `collision_captures/<session>/` 아래 CSV로 저장한다. 프로그램이 threshold나 production 소스를 자동으로 변경하지 않는다. `summary.csv`의 후보값, 실제 파형과 시험 메모를 팀이 검토한 뒤 `collision_config_t`에 수동으로 반영한다.

### 시험 안전

- 사람, 유리, 벽 또는 단단한 위험물에 충돌시키지 않는다.
- 낮은 속도와 충분한 완충재부터 시작한다.
- 충돌 직후 차량을 손으로 잡거나 들어 올리기 전에 먼저 기록을 종료한다.
- 종료 후 손으로 잡은 spike가 섞였으면 해당 trial을 폐기하거나 invalid로 분류한다.
- 너무 약한 접촉, 조종 실수, 계획과 다른 방향의 충돌을 억지로 정상 데이터에 포함하지 않는다.

---

## 12. 검증 명령

### 전체 정적/모의 회귀시험

대회 프로젝트 IPC 없이 핵심 구현만 검사하려면 다음을 실행한다.

```bash
make check-core
make check-python
```

`check-core`에는 Position Fusion, PAA SPI, BNO 기본/확장 모의시험과 충돌 수집기 `--self-test`가 포함된다. 아래 전체 시험은 외부 IPC 소스가 준비된 환경에서 실행한다.

```bash
make check
```

또는 C와 Python 시험을 나누어 실행한다.

```bash
make check-c
make check-python
```

현재 C 회귀시험 범위:

- Position Fusion 수학과 상태 경계
- PAA5100JE segmented SPI 및 signed motion decode
- BNO086 Phase-2 null recovery, FC race, packet parser, event cursor
- 사고차량 collision/IPC 및 calibration 종료 경계
- 관찰차량 Heading/reference/IPC 동작

### 실제 BNO086 C 시험

```bash
make bno-test-hw

/tmp/test_bno086_c_hw feature 10
/tmp/test_bno086_c_hw stability 60
/tmp/test_bno086_c_hw accel 30
```

- `feature 10`: Feature 설정 race를 10회 반복
- `stability 60`: RV/Gyro/Mag streaming을 60초 측정
- `accel 30`: Raw/Linear Acceleration streaming을 30초 측정

실제 시험에서는 단순 `io_error_count`만 보지 않고 다음 값을 구분한다.

| 진단값 | 의미 |
|---|---|
| `syscall_bus_error_count` | Linux read/write/ioctl, NACK, timeout, short transfer 계열 |
| `shtp_validation_error_count` | continuation/channel/sequence/length protocol 불일치 |
| `phase2_null_count` | 정확히 `00 00 00 00`인 transient Phase-2 header 발생 횟수 |
| `phase2_null_retry_success` | null 직후 1회 재읽기로 정상 continuation 복구 |
| `phase2_null_retry_fail` | 제한된 1회 재읽기에도 복구 실패 |

`phase2_null_count`가 있다는 사실만으로 물리 I2C 오류라고 단정하지 않는다. `retry_success`와 `retry_fail`, 실제 bus 오류와 validation 오류를 함께 확인한다.

### 실제 PAA5100JE 왕복 raw tick 시험

위치 오차가 IMU/world 변환 때문인지 PAA 자체의 양방향 tick 비대칭 때문인지
분리하려면 `accident_sensor`를 먼저 종료하고 다음을 실행한다.

```bash
make paa-roundtrip-test
```

화면 안내에 따라 같은 직선을 전진한 뒤 차체 방향을 바꾸지 않고 그대로 후진하여
시작점으로 복귀한다. 차를 돌려서 돌아오거나 조향하면 body-frame raw tick을 직접
상쇄할 수 없으므로 해당 trial은 유효하지 않다. 이 시험은 BNO086,
Heading, scaler와 위치 융합을 전혀 사용하지 않고 두 구간의 PAA raw `dx/dy`만
합산한다. `actual_return`은 `expected_return=-outbound`와 가까워야 하며,
`uncancelled`와 `raw_tick_closure_error`가 클수록 PAA 측정 또는 실제 경로가
왕복 대칭이 아니었다는 뜻이다. `max_read_gap`, `gap_over_20ms`, SQUAL과
단일 burst의 peak도 함께 출력하므로 read 지연이나 accumulator 포화 징후를
구분할 수 있다. 사람의 복귀 거리 오차가 포함되므로 시험기가 임의 PASS 임계값을
적용하지는 않는다.

---

## 13. 보존 이전 검증 이력과 실기기 한계

2026-09-10 저장소 분리본에서 하드웨어 및 외부 ACE IPC 없이 다시 실행한 결과:

| 명령 | 시험 | 결과 |
|---|---|---|
| `make check-core` | C Position Fusion | 18/18 PASS |
| `make check-core` | C PAA5100JE SPI 모의시험 | 4/4 PASS |
| `make check-core` | C BNO086 기본 모의시험 | 7/7 PASS |
| `make check-core` | C BNO086 확장 모의시험 | 37/37 PASS |
| `make check-core` | 충돌 수집기 self-test | PASS |
| `make check-python` | Python fusion math | PASS |
| `make check-python` | Python 위치 pipeline | 18/18 PASS |
| `make check-python` | Python SPI protocol | 8/8 PASS |

차량 프로세스의 외부 IPC 연동 시험과 센서 실측은 이 저장소 분리 작업에서 재실행하지 않았다. 아래 목록은 그 이전 개발 기록이다.

개발 과정에서 확인한 범위이며, 저장소 분리 시 모든 실기기 시험을 재실행한 것은 아니다:

- 새 BNO086에서 startup 및 실제 sensor report streaming 확인
- Feature 설정 race 반복 시험 성공
- RV/Gyro 약 100 Hz, Magnetometer 약 25/50 Hz 설정별 streaming 확인
- BNO reset/startup loss/stale 없는 60초 안정성 확인
- Phase-2 null header의 제한된 1회 recovery 검증
- C/Python Position Fusion 회귀시험 통과
- 관찰차량/사고차량 역할 분리 및 IPC frame 모의시험 통과
- Collision detector 상태 전이와 calibration 종료 경계 회귀시험 통과

이후 PAA5100JE도 실제 연결하여 Product ID `0x49`, inverse ID `0xB6`, burst/SQUAL 수신과 전후진 raw tick을 확인했다. 저속 왕복에서는 작은 잔차가 관측됐지만 빠른 이동에서는 큰 비대칭이 발생했고, 위치 추정을 ArUco로 전환했다. 현재 보장할 수 없으며 재사용 환경에서 다시 검증해야 할 항목:

- 실제 Optical Flow scale 정확도
- 실제 바닥 재질과 조명에 따른 SQUAL 영향
- PAA와 BNO report의 실제 시간 정렬 오차
- 실제 차량 회전 중심 lever-arm 학습 성능
- 실제 주행 후 원점 복귀 오차
- PAA signed dx/dy와 차량 장착축의 최종 부호
- 실제 RC카 충돌/정상 주행을 구분하는 threshold

PAA5100JE를 다른 환경에서 재사용할 때 최소 확인 항목:

1. Product ID `0x49`와 inverse ID `0xB6`
2. Motion burst와 SQUAL이 지속적으로 갱신되는지
3. 정지 상태 dx/dy noise
4. 전진/후진 및 좌/우 이동의 signed dx/dy
5. 같은 거리 왕복 시 scale과 부호 대칭성
6. ±90° 회전 후 body/world 축 변환
7. CW/CCW lever-arm calibration과 종료 후 첫 이동
8. 정사각형 또는 왕복 실제 주행 원점 복귀

---

## 14. 기존 프로젝트 IPC 계약 기록

보존된 차량 프로세스는 다음 계약을 사용한다. 향후 재사용할 때 상대 프로세스와 자료형/단위/메시지 형식의 호환성을 확인한다.

### 관찰차량

```text
witness_sensor
    → IPC_MSG_SENSOR_DATA
    → /tmp/witness_sensor.sock
    → witness_filter
```

Payload는 기존 `sensor_protocol.h`의 `vehicle_state_t`를 사용한다.

### 사고차량

```text
accident_sensor
    → IPC_MSG_ACCIDENT_EVENT
    → /tmp/p2p.sock
    → broadcast_sender
```

Payload는 기존 `p2p_protocol.h`의 `accident_event_payload_t`를 사용한다. Station ID, Action ID와 최종 DENM reference time은 네트워크 역할을 소유한 P2P 계층이 채운다. Sensor Process는 감지 시각과 로컬 위치/Heading을 제공한다.

두 프로세스의 IPC 목적지와 payload를 서로 바꾸거나 하나의 공통 sensor 실행 파일로 합치지 않는다.

---

## 15. 종료 및 실패 원칙

- `SIGINT`, `SIGTERM`을 받으면 worker 종료 요청 후 Thread를 join하고 장치와 mutex를 정리한다.
- BNO086 startup이나 실제 report 확인에 실패하면 센서 준비 완료로 표시하지 않는다.
- 사고차량에서 PAA 또는 BNO 하나가 실패하면 나머지 하나만 사용하여 위치를 추측하지 않는다.
- 관찰차량에서 Heading reference가 무효화되면 마지막 값을 계속 전송하지 않는다.
- Collision IPC queue가 포화되면 센서 수신을 막지 않고 drop을 계수한다.
- Collision threshold가 확정되지 않은 상태에서는 사고 이벤트 전송을 활성화하지 않는다.

이 원칙은 일부 값이라도 계속 출력하는 것보다, 유효하지 않은 위치·방향·사고 정보를 다른 프로세스가 정상 자료로 오해하지 않도록 하는 것을 우선한다.
