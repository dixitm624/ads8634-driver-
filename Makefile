CC = gcc
CFLAGS = -Wall -Wextra -Werror -g
TARGET = spi
SRC =  ads8634.c
HDR = ads8634.h

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) -lm

clean:
	rm -f $(TARGET)
