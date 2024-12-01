# Compiler
CXX = g++

# C++ standard
CXXFLAGS = -std=c++11

# Output binary name
TARGET = input_simulator

# Source files
SRCS = input_simulator.cpp

# Frameworks for macOS
FRAMEWORKS = -framework ApplicationServices

# Build rule
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS) $(FRAMEWORKS)

# Clean rule to remove the binary
clean:
	rm -f $(TARGET)
