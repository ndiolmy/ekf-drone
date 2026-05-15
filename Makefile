CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -Iinclude
SRCS    = src/matrix.c src/ekf.c src/imu_model.c src/sensor_models.c
OBJS    = $(SRCS:.c=.o)
TARGET  = ekf_drone

all: $(TARGET)

$(TARGET): $(OBJS) src/main.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test: $(OBJS) tests/test_matrix.c
	$(CC) $(CFLAGS) -o test_matrix $^ -lm
	./test_matrix

clean:
	rm -f $(OBJS) $(TARGET) test_matrix
