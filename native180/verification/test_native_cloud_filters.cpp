#include "native_cloud_filters.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

struct Point {
    float x;
    float y;
    float z;
    float intensity;
    float curvature;
    float normal_x;
};

struct Cloud {
    std::vector<Point> points;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool is_dense = false;
};

struct Rotation {
    double rows[3][3];
    double operator()(std::size_t row, std::size_t column) const { return rows[row][column]; }
};

const Rotation identity{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

void append(Cloud& cloud, double x, double y, double z)
{
    const auto index = static_cast<float>(cloud.points.size());
    cloud.points.push_back({
        static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
        index, index * 0.125f, index * 0.25f
    });
}

bool same_position(const Point& a, const Point& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

void verify_payload(const Cloud& result, const Cloud& original)
{
    for (const auto& point : result.points) {
        const auto& before = original.points.at(static_cast<std::size_t>(point.intensity));
        assert(point.intensity == before.intensity);
        assert(point.curvature == before.curvature);
        assert(point.normal_x == before.normal_x);
    }
}

bool contains(const Cloud& cloud, std::size_t index)
{
    return std::any_of(cloud.points.begin(), cloud.points.end(), [&](const Point& point) {
        return static_cast<std::size_t>(point.intensity) == index;
    });
}

void front_and_range()
{
    Cloud source;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    append(source, 1.0, 0.0, 0.5);
    append(source, -1.0, 0.0, 0.5);
    append(source, 0.0, -3.0, 0.0);
    append(source, 3.0, 0.0, 0.0);
    append(source, std::nextafter(3.0f, 4.0f), 0.0, 0.0);
    append(source, 1.0, 0.0, nan);
    append(source, 1.0, infinity, 0.0);
    append(source, 0.0, 1.0, 2.0);
    append(source, 0.0, 1.0, -2.0);
    append(source, -0.00001, 0.0, 0.0);
    append(source, 0.01, 2.2, 2.2);

    native_filter::Options options;
    options.retention_percent = 100;
    Cloud cloud = source;
    const auto counts = native_filter::filter(cloud, identity, options);
    assert(counts.input_count == 11 && counts.eligible_count == 5 && counts.sampled_count == 5);
    assert(counts.output_count == 5 && counts.smoothed_count == 0 && counts.outlier_removed == 0);
    assert(cloud.width == 5 && cloud.height == 1 && cloud.is_dense);
    const std::size_t expected[] = {0, 2, 3, 7, 8};
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        assert(cloud.points[i].intensity == expected[i]);
        assert(same_position(cloud.points[i], source.points[expected[i]]));
    }
    verify_payload(cloud, source);

    const Rotation forward_is_y{{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
    cloud = source;
    native_filter::filter(cloud, forward_is_y, options);
    assert(contains(cloud, 1));
    assert(!contains(cloud, 2));
}

void exact_midpoint_sampling()
{
    Cloud source;
    for (std::size_t n = 0; n <= 128; ++n) {
        for (int percent = 0; percent <= 100; ++percent) {
            auto cloud = source;
            native_filter::Options options;
            options.retention_percent = percent;
            const auto stats = native_filter::filter(cloud, identity, options);
            const std::size_t expected = n * percent / 100;
            assert(stats.eligible_count == n && stats.sampled_count == expected);
            assert(stats.output_count == expected);
            for (std::size_t i = 0; i < expected; ++i) {
                const auto midpoint = ((2 * i + 1) * n) / (2 * expected);
                assert(cloud.points[i].intensity == midpoint);
            }
        }
        append(source, 1.0, 0.0, 0.0);
    }
}

double noise(int row, int column)
{
    return 0.001 * ((row * 113 + column * 47 + row * column * 3) % 19 - 9);
}

Cloud wall()
{
    Cloud cloud;
    for (int row = 0; row < 25; ++row) {
        for (int column = 0; column < 31; ++column) {
            append(cloud, 1.2 + noise(row, column), 0.018 * (column - 15), 0.018 * row);
        }
    }
    return cloud;
}

double wall_rms(const Cloud& cloud)
{
    double sum = 0.0;
    for (const auto& point : cloud.points) {
        const double error = point.x - 1.2;
        sum += error * error;
    }
    return std::sqrt(sum / cloud.points.size());
}

void wall_smoothing()
{
    auto source = wall();
    // A dense wall behind the robot cannot influence the fit or reenter it.
    for (int row = 0; row < 25; ++row) {
        for (int column = 0; column < 31; ++column) {
            append(source, -1.2, 0.018 * (column - 15), 0.018 * row);
        }
    }
    native_filter::Options options;
    auto baseline = source;
    const auto sampled = native_filter::filter(baseline, identity, options);
    assert(sampled.eligible_count == 775 && sampled.sampled_count == 542);
    assert(sampled.output_count == 542 && sampled.smoothed_count == 0);

    options.smoothing = true;
    auto filtered = source;
    const auto stats = native_filter::filter(filtered, identity, options);
    assert(stats.sampled_count == sampled.sampled_count && stats.output_count == sampled.output_count);
    assert(stats.smoothed_count > stats.output_count / 2 && stats.outlier_removed == 0);
    const double before = wall_rms(baseline);
    const double after = wall_rms(filtered);
    assert(after < 0.60 * before);
    verify_payload(filtered, source);
    for (const auto& point : filtered.points) {
        const auto& original = source.points[static_cast<std::size_t>(point.intensity)];
        assert(point.x > 0.0 && point.z == original.z);
        const double shift = std::hypot(static_cast<double>(point.x) - original.x,
                                        static_cast<double>(point.y) - original.y);
        assert(shift <= 0.015);
    }
    std::cout << "Noisy wall RMS " << before << " m -> " << after << " m, "
              << stats.smoothed_count << " moved\n";
}

void protect_features()
{
    Cloud source;
    for (int row = 0; row < 15; ++row) {
        for (int column = 0; column < 13; ++column) {
            const double horizontal = 0.018 * column;
            append(source, 1.2 + noise(row, column), -horizontal, 0.018 * row);
            append(source, 1.2 + horizontal, noise(row, column), 0.018 * row);
        }
    }
    const auto corner_end = source.points.size();
    for (int row = 0; row < 15; ++row) {
        for (int column = 0; column < 13; ++column) {
            const double horizontal = 0.018 * column;
            append(source, 1.65 + noise(row, column), -0.048 - horizontal, 0.018 * row);
            append(source, 1.65 + noise(row, column), 0.048 + horizontal, 0.018 * row);
        }
    }
    const auto rod_start = source.points.size();
    for (int row = 0; row < 30; ++row) {
        const double angle = row * 0.8;
        append(source, 2.1 + 0.008 * std::cos(angle), 0.5 + 0.008 * std::sin(angle), 0.013 * row);
    }

    native_filter::Options options;
    options.retention_percent = 100;
    options.smoothing = true;
    auto filtered = source;
    const auto stats = native_filter::filter(filtered, identity, options);
    assert(stats.output_count == source.points.size() && stats.smoothed_count > 0);
    for (const auto& point : filtered.points) {
        const auto i = static_cast<std::size_t>(point.intensity);
        const auto& original = source.points[i];
        if (i < corner_end && i % 26 < 4) assert(same_position(point, original));
        if (i >= corner_end && i < rod_start && (i - corner_end) % 26 < 4) {
            assert(same_position(point, original));
        }
        if (i >= rod_start) assert(same_position(point, original));
    }
    verify_payload(filtered, source);
}

void outliers_need_presampling_veto()
{
    auto source = wall();
    for (int i = 0; i < 30; ++i) append(source, 1.2, 0.03, 0.015 * i);
    auto sampled = source;
    native_filter::Options options;
    native_filter::filter(sampled, identity, options);
    std::size_t supported_index = 0;
    std::size_t support_index = 0;
    std::size_t isolated_index = 0;
    for (std::size_t i = 775; i < source.points.size() - 1; ++i) {
        if (contains(sampled, i) && !contains(sampled, i + 1)) {
            supported_index = i;
            support_index = i + 1;
            break;
        }
    }
    for (std::size_t i = support_index + 1; i < source.points.size(); ++i) {
        if (contains(sampled, i)) {
            isolated_index = i;
            break;
        }
    }
    assert(supported_index >= 775 && isolated_index > support_index);
    source.points[supported_index].x = 2.2f;
    source.points[supported_index].y = 0.5f;
    source.points[support_index].x = 2.23f;
    source.points[support_index].y = 0.5f;
    source.points[isolated_index].x = 2.65f;
    source.points[isolated_index].y = -0.5f;

    options.smoothing = true;
    const auto stats = native_filter::filter(source, identity, options);
    assert(stats.sampled_count == sampled.points.size());
    assert(stats.outlier_candidates == 2 && stats.outlier_removed == 1);
    assert(stats.output_count == stats.sampled_count - 1);
    assert(contains(source, supported_index) && !contains(source, isolated_index));
}

void crop_preserves_outlier_support()
{
    auto source = wall();
    const auto edge_index = source.points.size();
    append(source, 3.0, 0.0, 0.0);
    const auto support_index = source.points.size();
    append(source, 3.04, 0.0, 0.0);
    auto with_support = source;
    source.points[support_index].y = 0.2f;

    native_filter::Options options;
    options.retention_percent = 100;
    options.smoothing = true;
    const auto isolated = native_filter::filter(source, identity, options);
    const auto supported = native_filter::filter(with_support, identity, options);
    assert(isolated.sampled_count == supported.sampled_count);
    assert(isolated.outlier_candidates == 1 && supported.outlier_candidates == 1);
    assert(isolated.outlier_removed == 1 && supported.outlier_removed == 0);
    assert(!contains(source, edge_index) && contains(with_support, edge_index));
    assert(!contains(source, support_index) && !contains(with_support, support_index));
}

void smoothing_respects_crop()
{
    Cloud source;
    for (int row = 0; row < 15; ++row) {
        for (int column = 0; column < 21; ++column) {
            append(source, 2.992 + noise(row, column), 0.014 * (column - 10), 0.012 * (row - 7));
            append(source, 0.008 + noise(row, column), 0.014 * (column - 10), 0.012 * (row - 7));
        }
    }
    native_filter::Options options;
    options.retention_percent = 100;
    options.smoothing = true;
    auto filtered = source;
    const auto stats = native_filter::filter(filtered, identity, options);
    assert(stats.smoothed_count > 0 && stats.output_count == stats.eligible_count);
    for (const auto& point : filtered.points) {
        const double x = point.x;
        const double y = point.y;
        const double z = point.z;
        assert(x >= 0.0 && x * x + y * y + z * z <= 9.0);
    }
}

}  // namespace

int main()
{
    front_and_range();
    exact_midpoint_sampling();
    wall_smoothing();
    protect_features();
    outliers_need_presampling_veto();
    crop_preserves_outlier_support();
    smoothing_respects_crop();
    std::cout << "All native cloud filter checks passed\n";
}
