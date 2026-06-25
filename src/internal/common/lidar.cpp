#include "lidar.hpp"

#include <cmath>
#include <cstring>
#include <chrono>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

// Port of robocad-py YDLidarX2 (lidar.py). See header for the caveat.

YDLidarX2::YDLidarX2(Robot* robot, std::string port, int chunk_size)
    : robot(robot), port(port), chunk_size(chunk_size)
{
    distances.assign(360, std::array<uint32_t, MAX_DATA>{});
    for (auto& row : distances) row.fill(OUT_OF_RANGE);
    distances_pnt.assign(360, 0);
    result.assign(360, OUT_OF_RANGE);

    // corrections[0] = 0; corrections[dist] for dist 1..8000
    corrections.assign(8001, 0.0);
    for (int dist = 1; dist <= 8000; dist++)
    {
        corrections[dist] = std::atan(21.8 * ((155.3 - dist) / (155.3 * dist))) * (180.0 / M_PI);
    }
}

YDLidarX2::~YDLidarX2()
{
    stop_scan();
    disconnect();
}

bool YDLidarX2::connect()
{
#ifndef _WIN32
    if (is_connected) return true;
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        robot->write_log("LiDAR: failed to open " + port);
        is_connected = false;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        robot->write_log("LiDAR: tcgetattr failed");
        ::close(fd);
        fd = -1;
        return false;
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 data bits
    tty.c_cflag &= ~PARENB;                        // no parity
    tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // no HW flow control
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;                           // 1s read timeout
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        robot->write_log("LiDAR: tcsetattr failed");
        ::close(fd);
        fd = -1;
        return false;
    }

    is_connected = true;
    return true;
#else
    robot->write_log("LiDAR: serial not supported on this platform");
    return false;
#endif
}

void YDLidarX2::disconnect()
{
#ifndef _WIN32
    if (is_connected && fd >= 0)
    {
        ::close(fd);
        fd = -1;
        is_connected = false;
    }
#endif
}

bool YDLidarX2::start_scan()
{
    if (!is_connected) return false;
    is_scanning = true;
    scan_thread = std::thread(&YDLidarX2::scan_loop, this);
    return true;
}

bool YDLidarX2::stop_scan()
{
    if (!is_scanning) return false;
    is_scanning = false;
    if (scan_thread.joinable()) scan_thread.join();
    return true;
}

// Decode a single packet (bytes between two 0xAA 0x55 headers).
void YDLidarX2::decode_packet(const std::vector<uint8_t>& d, std::vector<uint32_t>& dpnt)
{
    size_t l = d.size();
    if (l < 10) return;

    int sample_cnt = d[1];
    if (sample_cnt == 0) return;

    double start_angle = ((d[2] + 256 * d[3]) >> 1) / 64.0;
    double end_angle = ((d[4] + 256 * d[5]) >> 1) / 64.0;

    auto store = [&](long dist, double angle_f) {
        if (dist > MIN_RANGE)
        {
            if (dist > MAX_RANGE) dist = MAX_RANGE;
            long angle = std::lround(angle_f + corrections[dist]);
            if (angle < 0) angle += 360;
            if (angle >= 360) angle -= 360;
            if (angle < 0 || angle >= 360) return;
            uint32_t& pnt = dpnt[angle];
            if (pnt < (uint32_t)MAX_DATA)
            {
                distances[angle][pnt] = (uint32_t)dist;
                if (pnt < (uint32_t)(MAX_DATA - 1)) pnt += 1;
            }
        }
    };

    if (sample_cnt == 1)
    {
        long dist = std::lround((d[8] + 256 * d[9]) / 4.0);
        store(dist, start_angle);
    }
    else
    {
        if (start_angle == end_angle) return;
        if (l != (size_t)(8 + 2 * sample_cnt)) return;

        double step_angle;
        if (end_angle < start_angle)
            step_angle = (end_angle + 360.0 - start_angle) / (sample_cnt - 1);
        else
            step_angle = (end_angle - start_angle) / (sample_cnt - 1);

        size_t pnt = 8;
        double cur_angle = start_angle;
        while (pnt + 1 < l)
        {
            long dist = std::lround((d[pnt] + 256 * d[pnt + 1]) / 4.0);
            store(dist, cur_angle);
            cur_angle += step_angle;
            if (cur_angle >= 360.0) cur_angle -= 360.0;
            pnt += 2;
        }
    }
}

void YDLidarX2::scan_loop()
{
#ifndef _WIN32
    scan_is_active = true;
    std::vector<uint8_t> buf(chunk_size);

    while (is_scanning)
    {
        ssize_t n = ::read(fd, buf.data(), chunk_size);
        if (n <= 0) continue;

        // Prepend leftover from previous read.
        std::vector<uint8_t> data;
        data.reserve((has_last_chunk ? last_chunk.size() : 0) + (size_t)n);
        if (has_last_chunk) data.insert(data.end(), last_chunk.begin(), last_chunk.end());
        data.insert(data.end(), buf.begin(), buf.begin() + n);

        // Split on header 0xAA 0x55, like Python's split(b"\xaa\x55").
        std::vector<std::vector<uint8_t>> packets;
        std::vector<uint8_t> cur;
        for (size_t i = 0; i < data.size(); i++)
        {
            if (i + 1 < data.size() && data[i] == 0xAA && data[i + 1] == 0x55)
            {
                packets.push_back(cur);
                cur.clear();
                i++; // skip the 0x55
            }
            else
            {
                cur.push_back(data[i]);
            }
        }
        // The final fragment is incomplete; keep it for next iteration (Python: data.pop()).
        last_chunk = cur;
        has_last_chunk = true;

        std::vector<uint32_t> dpnt(360, 0);
        for (auto& p : packets) decode_packet(p, dpnt);

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            for (int angle = 0; angle < 360; angle++)
            {
                if (dpnt[angle] == 0)
                {
                    result[angle] = OUT_OF_RANGE;
                }
                else
                {
                    uint64_t sum = 0;
                    for (uint32_t k = 0; k < dpnt[angle]; k++) sum += distances[angle][k];
                    result[angle] = (int32_t)(sum / dpnt[angle]);
                }
            }
            distances_pnt = dpnt;
        }
    }
    scan_is_active = false;
#endif
}

std::vector<float> YDLidarX2::get_data()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    std::vector<float> out(360);
    for (int i = 0; i < 360; i++) out[i] = (float)result[i];
    return out;
}
