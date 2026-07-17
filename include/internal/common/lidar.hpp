#pragma once

#include <string>
#include <stdint.h>
#include <vector>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include "robot.hpp"
#include "lidar_base.hpp"

// Port of robocad-py YDLidarX2 (lidar.py). Reads the YDLidar X2 serial stream,
// decodes start/cloud packets and exposes 360 per-degree distances (mm).
// NOTE: this is a fresh C++ implementation of the Python algorithm and needs
// validation against real hardware.
class YDLidarX2 : public LidarBase
{
public:
    YDLidarX2(Robot* robot, std::string port, int chunk_size = 2000);
    ~YDLidarX2();

    void start() override;
    void stop() override;

    bool connect();
    bool start_scan();
    bool stop_scan();
    void disconnect();

    // 360 distances (one per degree); out_of_range value where no data.
    std::vector<float> get_data() override;

private:
    Robot* robot;
    std::string port;
    int chunk_size;

    int fd = -1;
    std::atomic<bool> is_connected{false};
    std::atomic<bool> is_scanning{false};
    std::atomic<bool> scan_is_active{false};

    static const int MIN_RANGE = 10;
    static const int MAX_RANGE = 8000;
    static const int MAX_DATA = 20;
    static const int OUT_OF_RANGE = 32768;

    std::thread scan_thread;
    std::mutex data_mutex;

    // [360][MAX_DATA] collected distances + per-angle counter
    std::vector<std::array<uint32_t, MAX_DATA>> distances;
    std::vector<uint32_t> distances_pnt;
    std::vector<int32_t> result;            // 360 final values
    std::vector<double> corrections;        // 0..8000 angle corrections
    std::vector<uint8_t> last_chunk;        // leftover bytes between reads
    bool has_last_chunk = false;

    void scan_loop();
    void decode_packet(const std::vector<uint8_t>& d, std::vector<uint32_t>& dpnt);
};
