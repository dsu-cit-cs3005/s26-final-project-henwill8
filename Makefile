# Compiler
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic

# Targets
all: test_robot RobotWarz

RobotBase.o: RobotBase.cpp RobotBase.h
	$(CXX) $(CXXFLAGS) -fPIC -c RobotBase.cpp

Arena.o: Arena.cpp Arena.h RadarObj.h RobotBase.h
	$(CXX) $(CXXFLAGS) -c Arena.cpp

RobotWarz: RobotWarz.cpp Arena.o RobotBase.o
	$(CXX) $(CXXFLAGS) RobotWarz.cpp Arena.o RobotBase.o -ldl -o RobotWarz

test_robot: test_robot.cpp RobotBase.o
	$(CXX) $(CXXFLAGS) test_robot.cpp RobotBase.o -ldl -o test_robot

clean:
	rm -f *.o test_robot RobotWarz *.so robots/*.so lib*.so 2>/dev/null; true
