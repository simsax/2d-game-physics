TARGET_EXE = 2d-physics
INCDIRS = ./src
CODEDIRS = ./src ./src/physics
BUILD_DIR = ./build
CC = cc
CFLAGS = -std=c11 -Wall -Wextra -pedantic -MP -MMD
LDLIBS = -Wl,-Bstatic -lraylib -Wl,-Bdynamic -lm

all: debug
debug: CFLAGS += -O0 -g
debug: $(BUILD_DIR)/$(TARGET_EXE)
release: CFLAGS += -O3 -DNDEBUG
release: $(BUILD_DIR)/$(TARGET_EXE)

run:
	$(BUILD_DIR)/$(TARGET_EXE)

CFLAGS += $(foreach D,$(INCDIRS),-I$(D))

SRCS = $(foreach D,$(CODEDIRS),$(wildcard $(D)/*.c))
OBJS = $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)

$(BUILD_DIR)/$(TARGET_EXE): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDLIBS)

$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)

# all targets that don't represent files go here
.PHONY: all clean run

-include $(DEPS)
