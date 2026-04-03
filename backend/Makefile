NPROC    := $(shell sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
MAKEFLAGS += -j$(NPROC)

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic -D_USE_MATH_DEFINES
CXXFLAGS += -MMD -MP
ifdef QUIET
CXXFLAGS += -DNDEBUG_LOG
endif
INCLUDES := -Iinclude -Ithird_party

SRC_DIR  := src
TEST_DIR := tests
CLI_DIR  := cli
LIB_DIR  := lib

PCH_SRC  := third_party/nlohmann/json.hpp
PCH_OUT  := $(LIB_DIR)/nlohmann_json.hpp.gch

# Core sources (no platform-specific networking)
CORE_SRC := constants.cpp ray_wall.cpp distance_localization.cpp \
            mcl/mcl_engine.cpp mcl/mcl_controller.cpp \
            mcl/localization_controller.cpp \
            sim/field.cpp sim/physics.cpp \
            sim/sim_harness.cpp \
            noise/noise_generator.cpp sim/sensor_model.cpp \
            ray/ray_cast_obstacles.cpp \
            state/session_recorder.cpp state/replay_loader.cpp \
            state/session_analytics.cpp \
            stress/stress_generators.cpp \
            noise/failure_injector.cpp noise/failure_config.cpp \
            sim/sim_chassis.cpp \
            sim/sim_sensor_provider.cpp \
            pursuit/waypoint_follower.cpp

# Full sources (adds server/api which depends on httplib/platform networking)
SRC := $(CORE_SRC) server/api.cpp

# Test sources
TEST_SRC := doctest_main.cpp test_ray_cast.cpp \
            test_mcl_stationary.cpp \
            test_physics.cpp test_mcl_moving.cpp \
            test_noise_generator.cpp test_sensor_model.cpp \
            test_mcl_odom_noise.cpp test_mcl_noisy_sensors.cpp \
            test_ray_cast_obstacles.cpp test_mcl_obstacles.cpp \
            test_tick_state.cpp test_session_recorder.cpp \
            test_replay_loader.cpp test_failure_injector.cpp \
            test_step_continuous.cpp test_failure_injector_ext.cpp \
            test_random_failures.cpp test_session_continuous.cpp \
            test_confidence_gate.cpp \
            test_cluster_stats.cpp \
            test_mcl_controller.cpp \
            test_ray_cast_shapes.cpp \
            test_sim_harness.cpp \
            test_localization_controller.cpp \
            test_session_analytics.cpp \
            test_stress_generators.cpp \
            test_pose_correction_controller.cpp \
            test_sim_chassis.cpp \
            test_sim_sensor_provider.cpp \
            test_navigation.cpp

# Object files (all under lib/)
CORE_OBJ := $(patsubst %.cpp,$(LIB_DIR)/%.o,$(CORE_SRC))
OBJ      := $(patsubst %.cpp,$(LIB_DIR)/%.o,$(SRC))
TEST_OBJ := $(patsubst %.cpp,$(LIB_DIR)/tests/%.o,$(TEST_SRC))

TEST_BIN := test_runner
SERVER_BIN := server_bin

SERVER_MAIN := $(LIB_DIR)/cli/server_main.o

STRESS_MAIN := $(LIB_DIR)/cli/stress_main.o
STRESS_BIN := stress_bin

.PHONY: all test server stress clean

all: $(TEST_BIN)

test: $(TEST_BIN)
	./$(TEST_BIN)

stress: $(STRESS_BIN)

$(STRESS_BIN): $(CORE_OBJ) $(STRESS_MAIN)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TEST_BIN): $(CORE_OBJ) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(PCH_OUT): $(PCH_SRC)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -x c++-header -o $@ $<

$(LIB_DIR)/%.o: $(SRC_DIR)/%.cpp $(PCH_OUT)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -include-pch $(PCH_OUT) -c $< -o $@

$(LIB_DIR)/tests/%.o: $(TEST_DIR)/%.cpp $(PCH_OUT)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -include-pch $(PCH_OUT) -c $< -o $@

$(LIB_DIR)/cli/%.o: $(CLI_DIR)/%.cpp $(PCH_OUT)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -include-pch $(PCH_OUT) -c $< -o $@

server: $(SERVER_BIN)

$(SERVER_BIN): $(OBJ) $(SERVER_MAIN)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lpthread

# Auto-generated dependency files (.d) next to object files.
-include $(CORE_OBJ:.o=.d) $(OBJ:.o=.d) $(TEST_OBJ:.o=.d) \
         $(SERVER_MAIN:.o=.d) $(STRESS_MAIN:.o=.d)

clean:
	rm -rf $(LIB_DIR) $(TEST_BIN) $(SERVER_BIN) $(STRESS_BIN)
