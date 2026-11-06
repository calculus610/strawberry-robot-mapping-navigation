// A return just inside 3 m must keep its 3 m-external raw support.
#include "native_cloud_filters.hpp"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

struct Point {
    float x, y, z;
    int id;
};

struct Cloud {
    std::vector<Point> points;
    std::size_t width = 0;
    std::size_t height = 1;
    bool is_dense = true;
};

struct Identity {
    double operator()(std::size_t row, std::size_t column) const
    {
        return row == column ? 1.0 : 0.0;
    }
};

bool contains(const Cloud& cloud, int id)
{
    return std::any_of(cloud.points.begin(), cloud.points.end(), [=](const Point& point) {
        return point.id == id;
    });
}

int main()
{
    Cloud isolated;
    for (int i = -7; i <= 7; ++i) {
        for (int j = 0; j < 15; ++j) {
            isolated.points.push_back({1.50f, 0.025f * i, 0.025f * j, 0});
        }
    }
    isolated.points.push_back({2.965f, 0.0f, 0.20f, 1});
    auto supported = isolated;
    supported.points.push_back({3.035f, 0.0f, 0.20f, 2});
    native_filter::Options options;
    options.smoothing = true;
    const auto unsupported_stats = native_filter::filter(isolated, Identity{}, options);
    const auto supported_stats = native_filter::filter(supported, Identity{}, options);
    std::cout << "N without support=" << unsupported_stats.eligible_count
              << ", N with external support=" << supported_stats.eligible_count
              << ", isolated point retained=" << contains(isolated, 1)
              << ", supported point retained=" << contains(supported, 1)
              << ", external point entered matching=" << contains(supported, 2) << '\n';
    return unsupported_stats.eligible_count == supported_stats.eligible_count
        && !contains(isolated, 1) && contains(supported, 1) && !contains(supported, 2) ? 0 : 1;
}
