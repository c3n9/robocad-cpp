#pragma once

#include <vector>

enum class LidarTypes
{
    N10_LIDAR = 0,
    YD_LIDAR_X2 = 1
};

class LidarBase
{
public:
    virtual ~LidarBase() {}

    virtual void start() = 0;
    // 360 distances, one per degree.
    virtual std::vector<float> get_data() = 0;
    virtual void stop() = 0;
};
