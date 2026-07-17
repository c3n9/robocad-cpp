#include "n10_lidar.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

// Port of robocad-py N10Lidar (n10_lidar.py). See header for the caveat.

N10Lidar::N10Lidar(Robot* robot, std::string port, int baud)
    : robot(robot), port(port), baud(baud)
{
    data.assign(360, 0.0f);
}

N10Lidar::~N10Lidar()
{
    stop();
}

void N10Lidar::start()
{
#ifndef _WIN32
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        robot->write_log("LiDAR N10: failed to open " + port);
        return;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        robot->write_log("LiDAR N10: tcgetattr failed");
        ::close(fd);
        fd = -1;
        return;
    }

    speed_t speed = B230400;
    if (baud == 115200) speed = B115200;
    else if (baud == 460800) speed = B460800;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
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
        robot->write_log("LiDAR N10: tcsetattr failed");
        ::close(fd);
        fd = -1;
        return;
    }

    shutdown_flag = false;
    scan_thread = std::thread(&N10Lidar::scan_loop, this);
#else
    robot->write_log("LiDAR N10: serial not supported on this platform");
#endif
}

void N10Lidar::stop()
{
    shutdown_flag = true;
    if (scan_thread.joinable()) scan_thread.join();
#ifndef _WIN32
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
#endif
}

std::vector<float> N10Lidar::get_data()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return data;
}

void N10Lidar::scan_loop()
{
#ifndef _WIN32
    std::vector<uint8_t> buf(MIN_PAYLOAD * 16);
    while (!shutdown_flag)
    {
        ssize_t n = ::read(fd, buf.data(), buf.size());
        if (n <= 0) continue;

        buffer.insert(buffer.end(), buf.begin(), buf.begin() + n);
        if (static_cast<int>(buffer.size()) > MIN_PAYLOAD * MAX_BUFFERED_PACKETS)
            buffer.clear();

        parse_buffer();
    }
#endif
}

void N10Lidar::parse_buffer()
{
    while (true)
    {
        size_t start = buffer.size();
        for (size_t i = 0; i + 1 < buffer.size(); i++)
        {
            if (buffer[i] == PKG_HEADER_0 && buffer[i + 1] == PKG_HEADER_1)
            {
                start = i;
                break;
            }
        }

        if (start == buffer.size())
        {
            // No header yet. Keep the last byte: it may be a header split across reads.
            if (buffer.size() > 1)
                buffer.erase(buffer.begin(), buffer.end() - 1);
            return;
        }

        if (start > 0) buffer.erase(buffer.begin(), buffer.begin() + start);
        if (static_cast<int>(buffer.size()) < MIN_PAYLOAD) return;

        decode_packet();
        buffer.erase(buffer.begin(), buffer.begin() + MIN_PAYLOAD);
    }
}

void N10Lidar::decode_packet()
{
    int start_angle = buffer[5] * 256 + buffer[6];
    int end_angle = buffer[55] * 256 + buffer[56];

    double step = ((end_angle + 36000 - start_angle) % 36000) / static_cast<double>(POINT_PER_PACK - 1) / 100.0;
    double start_deg = start_angle / 100.0;

    std::lock_guard<std::mutex> lock(data_mutex);
    for (int i = 0; i < POINT_PER_PACK; i++)
    {
        int o = 7 + i * 3;
        int dist = buffer[o] * 256 + buffer[o + 1];

        // std::nearbyint with the default rounding mode matches Python's
        // banker's rounding, which round() in the reference decoder uses.
        int angle = static_cast<int>(std::nearbyint(start_deg + step * i)) % 360;
        if (angle < 0) angle += 360;

        data[angle] = static_cast<float>(dist);
    }
}
