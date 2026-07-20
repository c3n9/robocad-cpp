#pragma once

#include <string>
#include <stdint.h>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include "robot.hpp"
#include "lidar_base.hpp"

// Port of robocad-py N10Lidar (n10_lidar.py). LDROBOT N10: header 0xA5 0x5A,
// 58-byte packet, big-endian fields, angles in hundredths of a degree.
//  [0..1] header, [5..6] start angle, [7..54] 16 points x 3 bytes
//  (2 distance + 1 intensity), [55..56] end angle.
// NOTE: validated against the Python decoder on synthetic packets, not against
// real hardware.
class N10Lidar : public LidarBase
{
public:
    N10Lidar(Robot* robot, std::string port, int baud = 230400);
    ~N10Lidar();

    void start() override;
    std::vector<float> get_data() override;
    void stop() override;

private:
    static const uint8_t PKG_HEADER_0 = 0xA5;
    static const uint8_t PKG_HEADER_1 = 0x5A;
    static const int MIN_PAYLOAD = 58;
    static const int POINT_PER_PACK = 16;

    // Drop the backlog if parsing falls this far behind the sensor.
    static const int MAX_BUFFERED_PACKETS = 100;

    Robot* robot;
    std::string port;
    int baud;

    int fd = -1;
    std::atomic<bool> shutdown_flag{false};
    std::thread scan_thread;
    std::mutex data_mutex;

    std::vector<float> data;      // 360 final values
    std::vector<uint8_t> buffer;  // bytes not yet parsed

    void scan_loop();
    void parse_buffer();
    void decode_packet();
};
