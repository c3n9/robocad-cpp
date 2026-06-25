#pragma once

#include "robot_configuration.hpp"
#include <string>
#include <mutex>
#include <atomic>
#include <fstream>
	
class RobotInfo
{
public:
	std::atomic<float> spi_time_dev = 0;
	std::atomic<float> rx_spi_time_dev = 0;
	std::atomic<float> tx_spi_time_dev = 0;
	std::atomic<float> spi_count_dev = 0;
	std::atomic<float> com_time_dev = 0;
	std::atomic<float> rx_com_time_dev = 0;
	std::atomic<float> tx_com_time_dev = 0;
	std::atomic<float> com_count_dev = 0;
	std::atomic<float> temperature = 0;
	std::atomic<float> memory_load = 0;
	std::atomic<float> cpu_load = 0;
};

class Robot 
{
public:
	std::atomic<bool> on_real_robot;
	std::atomic<float> power;
	RobotInfo* robot_info;

	Robot(bool on_real_robot, RobotConfiguration* conf);
	virtual ~Robot();

	void write_log(std::string text);

protected:
	std::ofstream log_file;
	std::mutex log_mutex;
};