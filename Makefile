CC = gcc
CFLAGS = -Wall -Wextra -O2 -Iinclude
SRC = src/can_protocol.c src/sensor_fusion.c src/safety_ecu.c

all: adas_ecu_demo test

adas_ecu_demo: $(SRC) src/main.c
	$(CC) $(CFLAGS) $^ -o $@

test: $(SRC) tests/test_ecu.c
	$(CC) $(CFLAGS) $^ -o tests/test_ecu
	./tests/test_ecu

clean:
	rm -f adas_ecu_demo tests/test_ecu *.exe tests/*.exe
