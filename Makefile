CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror
CPPFLAGS += -Iinclude
HEADERS := $(wildcard include/ovelo/*.hpp)

.PHONY: all test example sanitize pros-smoke clean
all: test
build:
	mkdir -p build
build/tests: tests/test.cpp $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@
build/localization-tests: tests/localization_test.cpp $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@
build/simulator: examples/desktop/simulator.cpp $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@
test: build/tests build/localization-tests
	./build/tests
	./build/localization-tests
	python3 -m unittest discover -s tests -p 'test_*.py' -v
example: build/simulator
	./build/simulator
sanitize: | build
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer tests/test.cpp -o build/tests-sanitize
	./build/tests-sanitize
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer tests/localization_test.cpp -o build/localization-sanitize
	./build/localization-sanitize
# This uses the REAL PROS kernel/toolchain, never a local stub.
# Set PROS_PROJECT to a new directory outside this source checkout.
PROS_PROJECT ?= /tmp/ovelo-v5
pros-smoke:
	python3 tools/create_pros_project.py "$(PROS_PROJECT)"
	cd "$(PROS_PROJECT)" && pros make
clean:
	$(RM) -r build
