#pragma once

#include <stdint.h>
#include <vector>

struct SMissionData
{
    struct Point
    {
        double x, y;
    };

    std::vector<Point> path;
    uint32_t hash;
};