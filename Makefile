CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -Iinclude

SRCS    = src/matrix.c src/ekf.c src/imu_model.c src/sensor_models.c src/main.c
OBJS    = $(SRCS:.c=.o)

TARGET  = ekf_drone

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	./$(TARGET)

test: $(filter-out src/main.o,$(OBJS)) tests/test_matrix.c
	$(CC) $(CFLAGS) -o test_matrix $^ -lm
	./test_matrix

clean:
	rm -f $(OBJS) $(TARGET) test_matrix
