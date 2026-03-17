CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -Iinclude -Ithird_party

# Phase 1 sources
SRC := constants.cpp ray_wall.cpp distance_localization.cpp

# Test sources
TEST_SRC := tests/doctest_main.cpp tests/test_ray_cast.cpp

# Object files
OBJ      := $(SRC:.cpp=.o)
TEST_OBJ := $(TEST_SRC:.cpp=.o)

TEST_BIN := test_runner

.PHONY: test server chaos clean

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(OBJ) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

server:
	@echo "Server target not yet implemented (Phase 7)"

chaos:
	@echo "Chaos target not yet implemented (Phase 7)"

clean:
	rm -f $(OBJ) $(TEST_OBJ) $(TEST_BIN)
