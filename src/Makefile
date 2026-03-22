CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -I../include -Iinclude -I../third_party -Ithird_party -I.

# Sources
SRC := constants.cpp ray_wall.cpp distance_localization.cpp \
       mcl/mcl_engine.cpp mcl/mcl_controller.cpp \
       sim/field.cpp sim/physics.cpp \
       noise/noise_generator.cpp sim/sensor_model.cpp \
       ray/ray_cast_obstacles.cpp \
       pursuit/pure_pursuit.cpp pursuit/route.cpp pursuit/route_runner.cpp \
       state/sim_session.cpp state/replay_loader.cpp \
       noise/failure_injector.cpp \
       server/api.cpp

# Test sources
TEST_SRC := ../tests/doctest_main.cpp ../tests/test_ray_cast.cpp \
            ../tests/test_mcl_stationary.cpp \
            ../tests/test_physics.cpp ../tests/test_mcl_moving.cpp \
            ../tests/test_noise_generator.cpp ../tests/test_sensor_model.cpp \
            ../tests/test_mcl_odom_noise.cpp ../tests/test_mcl_noisy_sensors.cpp \
            ../tests/test_ray_cast_obstacles.cpp ../tests/test_mcl_obstacles.cpp \
            ../tests/test_tick_state.cpp \
            ../tests/test_replay_loader.cpp ../tests/test_failure_injector.cpp \
            ../tests/test_step_continuous.cpp ../tests/test_pure_pursuit.cpp \
            ../tests/test_route_def.cpp ../tests/test_failure_injector_ext.cpp \
            ../tests/test_random_failures.cpp ../tests/test_session_continuous.cpp \
            ../tests/test_route_runner.cpp \
            ../tests/test_confidence_gate.cpp \
            ../tests/test_cluster_stats.cpp \
            ../tests/test_mcl_controller.cpp

# Object files
OBJ      := $(SRC:.cpp=.o)
TEST_OBJ := $(TEST_SRC:.cpp=.o)

TEST_BIN := test_runner
SERVER_BIN := server_bin
ROUTE_BIN := route_bin

SERVER_MAIN := ../cli/server_main.o
ROUTE_MAIN := ../cli/route_main.o

.PHONY: test server route clean

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(OBJ) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

server: $(SERVER_BIN)

route: $(ROUTE_BIN)

$(SERVER_BIN): $(OBJ) $(SERVER_MAIN)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lpthread

$(ROUTE_BIN): $(OBJ) $(ROUTE_MAIN)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lpthread

clean:
	rm -f $(OBJ) $(TEST_OBJ) $(TEST_BIN) $(SERVER_BIN) $(ROUTE_BIN) $(SERVER_MAIN) $(ROUTE_MAIN)
