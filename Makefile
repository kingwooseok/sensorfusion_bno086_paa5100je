CC ?= gcc
CPPFLAGS ?= -I.
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror
LDLIBS ?= -pthread -lm
ACE_IPC_DIR ?= ../2026ESWContest_mobility_ACE/ipc
SENSOR_CPPFLAGS = $(CPPFLAGS) -I$(ACE_IPC_DIR)
SENSOR_IPC_SOURCE = $(ACE_IPC_DIR)/ipc.c

ACCIDENT_SOURCES = accident_sensor.c bno086.c paa5100je.c position_fusion.c
WITNESS_SOURCES = witness_sensor.c bno086.c position_fusion.c

.PHONY: all check check-c check-core check-python bno-test-hw paa-roundtrip-test

all: accident_sensor witness_sensor

accident_sensor: $(ACCIDENT_SOURCES) bno086.h paa5100je.h position_fusion.h \
		$(ACE_IPC_DIR)/ipc.h $(ACE_IPC_DIR)/protocol.h \
		$(ACE_IPC_DIR)/p2p_protocol.h $(SENSOR_IPC_SOURCE)
	$(CC) $(SENSOR_CPPFLAGS) $(CFLAGS) $(ACCIDENT_SOURCES) \
		$(SENSOR_IPC_SOURCE) $(LDLIBS) -o $@

witness_sensor: $(WITNESS_SOURCES) bno086.h position_fusion.h \
		$(ACE_IPC_DIR)/ipc.h $(ACE_IPC_DIR)/protocol.h \
		$(ACE_IPC_DIR)/sensor_protocol.h $(SENSOR_IPC_SOURCE)
	$(CC) $(SENSOR_CPPFLAGS) $(CFLAGS) $(WITNESS_SOURCES) \
		$(SENSOR_IPC_SOURCE) $(LDLIBS) -o $@

check: check-c check-python

check-c: check-core
	$(CC) $(SENSOR_CPPFLAGS) $(CFLAGS) scratch/test_accident_sensor_c.c \
		bno086.c paa5100je.c position_fusion.c $(SENSOR_IPC_SOURCE) $(LDLIBS) \
		-o /tmp/test_accident_sensor_c
	/tmp/test_accident_sensor_c
	$(CC) $(SENSOR_CPPFLAGS) $(CFLAGS) scratch/test_witness_sensor_c.c \
		bno086.c position_fusion.c $(SENSOR_IPC_SOURCE) $(LDLIBS) \
		-o /tmp/test_witness_sensor_c
	/tmp/test_witness_sensor_c

# Standalone, hardware-free checks; no ACE IPC source is needed.
check-core:
	$(CC) $(CPPFLAGS) $(CFLAGS) scratch/test_position_fusion_c.c \
		position_fusion.c $(LDLIBS) -o /tmp/test_position_fusion_c
	/tmp/test_position_fusion_c
	$(CC) $(CPPFLAGS) $(CFLAGS) scratch/test_paa5100je_c.c \
		paa5100je.c $(LDLIBS) -o /tmp/test_paa5100je_c
	/tmp/test_paa5100je_c
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBNO086_TESTING \
		scratch/test_bno086_c.c bno086.c $(LDLIBS) -o /tmp/test_bno086_c
	/tmp/test_bno086_c mock
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBNO086_TESTING \
		scratch/test_bno086_c_mock.c bno086.c $(LDLIBS) -o /tmp/test_bno086_c_mock
	/tmp/test_bno086_c_mock
	$(CC) $(CPPFLAGS) $(CFLAGS) test_collision_capture.c bno086.c \
		$(LDLIBS) -o /tmp/test_collision_capture
	/tmp/test_collision_capture --self-test

check-python:
	PYTHONDONTWRITEBYTECODE=1 python3 scratch/test_fusion_math.py
	PYTHONDONTWRITEBYTECODE=1 python3 scratch/test_position_pipeline.py
	PYTHONDONTWRITEBYTECODE=1 python3 scratch/test_pmw3901_spi_protocol.py

bno-test-hw:
	$(CC) $(CPPFLAGS) $(CFLAGS) scratch/test_bno086_c.c \
		bno086.c $(LDLIBS) -o /tmp/test_bno086_c_hw

paa-roundtrip-test:
	$(CC) $(CPPFLAGS) $(CFLAGS) scratch/test_paa_roundtrip.c \
		paa5100je.c $(LDLIBS) -o /tmp/test_paa_roundtrip
	/tmp/test_paa_roundtrip
