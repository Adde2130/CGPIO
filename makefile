CC = gcc
CFLAGS = -Wall -Wextra -fPIC -pedantic -I./include -I../misc
LDFLAGS = -shared

BUILD_DIR := build
SRCS := $(wildcard src/*.c)
OBJS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SRCS))
LIB_NAME = libgpio
LIB_DIR = lib

$(shell mkdir -p $(BUILD_DIR))
$(shell mkdir -p $(LIB_DIR))

.PHONY: all clean

$(BUILD_DIR)/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

all: dynamic static

dynamic: $(OBJS)
	$(CC) $(LDFLAGS) -o $(LIB_DIR)/$(LIB_NAME).so $(OBJS)

static: $(OBJS)
	ar rcs $(LIB_DIR)/$(LIB_NAME).a $(OBJS)

clean:
	rm -f $(OBJS) $(LIB_DIR)/$(LIB_NAME).so $(LIB_DIR)/$(LIB_NAME).a

