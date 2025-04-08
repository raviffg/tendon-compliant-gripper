CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
INCLUDES  = -Isrc

BUILD    := build
TARGET   := $(BUILD)/libtendon_coupling.a
OBJS     := $(BUILD)/tendon_coupling.o

PYTHON   ?= python

.PHONY: all clean eval test

all: $(TARGET)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(TARGET): $(OBJS)
	ar rcs $@ $^

eval:
	$(PYTHON) scripts/grasp_stiffness_eval.py

test: all
	@echo "no unit tests wired yet"

clean:
	rm -rf $(BUILD)
