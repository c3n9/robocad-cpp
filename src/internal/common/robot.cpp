#include "robot.hpp"
#include "robot_configuration.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <time.h>

Robot::Robot(bool real_robot, RobotConfiguration* conf)
{
	on_real_robot = real_robot;
	power = 0;

	const std::string& log_path = on_real_robot ? conf->real_log_path : conf->sim_log_path;
	log_file.open(log_path, std::ios::out | std::ios::trunc);
	if (!log_file.is_open()) {
		std::cerr << "Failed to open log file: " << log_path << std::endl;
	}

	robot_info = new RobotInfo();
}

Robot::~Robot() {
	if (log_file.is_open()) {
		log_file.close();
	}
	delete robot_info;
	robot_info = NULL;
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about the date/time format
    strftime(buf, sizeof(buf), "%X", &tstruct);

    return buf;
}

void Robot::write_log(std::string s)
{
	std::lock_guard<std::mutex> lock(log_mutex);

	if (!log_file.is_open()) return;

	// TODO: view current thread and other shite
    std::ostringstream oss;
    oss << currentDateTime() << " " << s;
    log_file << oss.str() << std::endl;
}