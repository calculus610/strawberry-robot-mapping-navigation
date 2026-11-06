#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace native_filter {

struct Options {
    double max_range = 3.0;
    int retention_percent = 70;
    bool smoothing = false;
};

struct Stats {
    std::size_t input_count = 0;
    std::size_t eligible_count = 0;
    std::size_t sampled_count = 0;
    std::size_t smoothed_count = 0;
    std::size_t outlier_candidates = 0;
    std::size_t outlier_removed = 0;
    std::size_t output_count = 0;
};

namespace detail {

using XYZ = std::array<double, 3>;
using Rotation = std::array<XYZ, 3>;

constexpr double smooth_radius = 0.12;
constexpr double max_shift = 0.015;
constexpr double plane_residual_limit = 0.020;
constexpr double support_radius = 0.10;
constexpr double isolation_radius = 0.15;
constexpr std::size_t outlier_k = 10;
constexpr std::size_t max_neighbors = 32;
constexpr double unlimited = std::numeric_limits<double>::infinity();

template <typename Point>
XYZ coordinates(const Point& point)
{
    return {{point.x, point.y, point.z}};
}

inline XYZ subtract(const XYZ& a, const XYZ& b)
{
    return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

inline double dot(const XYZ& a, const XYZ& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline double squared_distance(const XYZ& a, const XYZ& b)
{
    const auto offset = subtract(a, b);
    return dot(offset, offset);
}

inline bool is_forward(const XYZ& point, const XYZ& forward)
{
    return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])
        && dot(forward, point) >= 0.0;
}

inline bool is_eligible(const XYZ& point, const XYZ& forward, double range_squared)
{
    return dot(point, point) <= range_squared && is_forward(point, forward);
}

struct Neighbor {
    double distance_squared;
    std::size_t index;
};

inline bool nearer(const Neighbor& a, const Neighbor& b)
{
    return a.distance_squared < b.distance_squared;
}

struct Neighborhood {
    std::array<Neighbor, max_neighbors> neighbors{};
    std::size_t size = 0;
};

// Median-split kd-tree over immutable deskewed coordinates. Positions in
// order_ form an implicit tree, and each query uses a fixed-size heap.
class SpatialIndex {
public:
    SpatialIndex(const std::vector<XYZ>& positions, std::vector<std::size_t> indices)
        : positions_(positions), order_(std::move(indices)), axes_(order_.size())
    {
        build(0, order_.size());
    }

    Neighborhood nearest(std::size_t query_index, std::size_t limit,
                         double radius_squared) const
    {
        Neighborhood result;
        search(0, order_.size(), query_index, limit, radius_squared, result);
        std::sort_heap(result.neighbors.begin(), result.neighbors.begin() + result.size, nearer);
        return result;
    }

private:
    void build(std::size_t first, std::size_t last)
    {
        if (first == last) return;
        XYZ low = positions_[order_[first]];
        XYZ high = low;
        for (std::size_t i = first + 1; i < last; ++i) {
            const auto& point = positions_[order_[i]];
            for (std::size_t axis = 0; axis < 3; ++axis) {
                low[axis] = std::min(low[axis], point[axis]);
                high[axis] = std::max(high[axis], point[axis]);
            }
        }
        const auto span = subtract(high, low);
        const auto axis = std::max_element(span.begin(), span.end()) - span.begin();
        const auto middle = first + (last - first) / 2;
        std::nth_element(order_.begin() + first, order_.begin() + middle,
                         order_.begin() + last, [&](std::size_t a, std::size_t b) {
            if (positions_[a][axis] == positions_[b][axis]) return a < b;
            return positions_[a][axis] < positions_[b][axis];
        });
        axes_[middle] = static_cast<unsigned char>(axis);
        build(first, middle);
        build(middle + 1, last);
    }

    void search(std::size_t first, std::size_t last, std::size_t query_index,
                std::size_t limit, double radius_squared, Neighborhood& result) const
    {
        if (first == last) return;
        const auto middle = first + (last - first) / 2;
        const auto index = order_[middle];
        const auto axis = axes_[middle];
        const auto& query = positions_[query_index];
        const double delta = query[axis] - positions_[index][axis];
        const double distance_squared = squared_distance(query, positions_[index]);
        auto heap_begin = result.neighbors.begin();
        if (index != query_index && distance_squared <= radius_squared) {
            if (result.size < limit) {
                result.neighbors[result.size++] = {distance_squared, index};
                std::push_heap(heap_begin, heap_begin + result.size, nearer);
            } else if (distance_squared < heap_begin->distance_squared) {
                std::pop_heap(heap_begin, heap_begin + limit, nearer);
                result.neighbors[limit - 1] = {distance_squared, index};
                std::push_heap(heap_begin, heap_begin + limit, nearer);
            }
        }
        const bool left = delta < 0.0;
        const auto near_first = left ? first : middle + 1;
        const auto near_last = left ? middle : last;
        search(near_first, near_last, query_index, limit, radius_squared, result);
        if (delta * delta <= radius_squared
            && (result.size < limit || delta * delta < heap_begin->distance_squared)) {
            const auto far_first = left ? middle + 1 : first;
            const auto far_last = left ? last : middle;
            search(far_first, far_last, query_index, limit, radius_squared, result);
        }
    }

    const std::vector<XYZ>& positions_;
    std::vector<std::size_t> order_;
    std::vector<unsigned char> axes_;
};

// First-order local weighted fit, constrained to a body-vertical plane.
// As in the offline MLS-style fit, only the horizontal normal can move the
// query. Independent height span, two-sided nearby support, and low residuals
// protect thin features and mixed planes near corners or separated walls.
inline XYZ correction(const std::vector<XYZ>& positions, std::size_t index,
                      const Neighborhood& neighbors, const Rotation& rotation)
{
    if (neighbors.size < 10) return {};

    std::array<XYZ, max_neighbors> offsets{};
    std::array<double, max_neighbors> weights{};
    XYZ centroid{};
    double weight_sum = 0.0;
    for (std::size_t i = 0; i < neighbors.size; ++i) {
        const auto& neighbor = neighbors.neighbors[i];
        const auto offset = subtract(positions[neighbor.index], positions[index]);
        const double weight = std::exp(-neighbor.distance_squared / (smooth_radius * smooth_radius));
        weights[i] = weight;
        weight_sum += weight;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            offsets[i][axis] = dot(rotation[axis], offset);
            centroid[axis] += weight * offsets[i][axis];
        }
    }
    for (auto& coordinate : centroid) coordinate /= weight_sum;

    XYZ covariance_xy{};
    for (std::size_t i = 0; i < neighbors.size; ++i) {
        const auto centered = subtract(offsets[i], centroid);
        covariance_xy[0] += weights[i] * centered[0] * centered[0];
        covariance_xy[1] += weights[i] * centered[0] * centered[1];
        covariance_xy[2] += weights[i] * centered[1] * centered[1];
    }
    const double angle = 0.5 * std::atan2(2.0 * covariance_xy[1], covariance_xy[0] - covariance_xy[2]);
    const XYZ tangent{{std::cos(angle), std::sin(angle), 0.0}};
    const XYZ normal{{-tangent[1], tangent[0], 0.0}};
    const double shift = dot(normal, centroid);
    if (std::abs(shift) <= 1e-9 || std::abs(shift) > max_shift) return {};

    double along_variance = 0.0;
    double height_variance = 0.0;
    double thickness = 0.0;
    double along_height = 0.0;
    double normal_height = 0.0;
    std::size_t left_support = 0;
    std::size_t right_support = 0;
    for (std::size_t i = 0; i < neighbors.size; ++i) {
        const double along = dot(tangent, offsets[i]);
        const auto centered = subtract(offsets[i], centroid);
        const double normal_residual = dot(normal, centered);
        if (std::abs(normal_residual) > plane_residual_limit) return {};
        if (along >= -0.075 && along <= -0.025) ++left_support;
        if (along >= 0.025 && along <= 0.075) ++right_support;
        const double centered_along = dot(tangent, centered);
        const double height = centered[2];
        const double weight = weights[i];
        along_variance += weight * centered_along * centered_along;
        height_variance += weight * height * height;
        thickness += weight * normal_residual * normal_residual;
        along_height += weight * centered_along * height;
        normal_height += weight * normal_residual * height;
    }
    if (left_support < 2 || right_support < 2 || along_variance <= 0.0) return {};

    // Remove height variation explained by the horizontal tangent. A line
    // diagonal in XYZ has almost no independent height variance.
    const double independent_height = height_variance - along_height * along_height / along_variance;
    const double min_span = std::min(along_variance, independent_height);
    const double max_span = std::max(along_variance, independent_height);
    constexpr double tan_15_degrees = 0.267949192431;
    if (min_span < 0.10 * max_span || thickness > 0.10 * min_span
        || std::abs(normal_height) > tan_15_degrees * independent_height) {
        return {};
    }
    // Inverse of the input rotation. Preserve height along the body Z axis.
    return {{
        shift * (normal[0] * rotation[0][0] + normal[1] * rotation[1][0]),
        shift * (normal[0] * rotation[0][1] + normal[1] * rotation[1][1]),
        shift * (normal[0] * rotation[0][2] + normal[1] * rotation[1][2])
    }};
}

struct Refinement {
    XYZ offset{};
    bool remove = false;
};

inline std::vector<Refinement> refine(const std::vector<XYZ>& positions,
                                     const std::vector<std::size_t>& sampled,
                                     const Rotation& rotation, Stats& stats)
{
    std::vector<Refinement> result(sampled.size());
    if (sampled.size() <= outlier_k) return result;

    const SpatialIndex index(positions, sampled);
    std::vector<double> mean_distances;
    mean_distances.reserve(sampled.size());
    std::vector<std::size_t> isolated;
    double mean = 0.0;
    double deviation_sum = 0.0;
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        const auto point = sampled[i];
        const auto local = index.nearest(point, max_neighbors, smooth_radius * smooth_radius);
        result[i].offset = correction(positions, point, local, rotation);

        const auto nearest = local.size >= outlier_k
            ? local : index.nearest(point, outlier_k, unlimited);
        double distance = 0.0;
        for (std::size_t j = 0; j < outlier_k; ++j) {
            distance += std::sqrt(nearest.neighbors[j].distance_squared);
        }
        distance /= outlier_k;
        mean_distances.push_back(distance);
        const double difference = distance - mean;
        mean += difference / (i + 1);
        deviation_sum += difference * (distance - mean);
        if (nearest.neighbors.front().distance_squared >= isolation_radius * isolation_radius) {
            isolated.push_back(i);
        }
    }

    // The offline SOR-style rule also requires absolute isolation and no
    // neighboring original return. Build the raw index only for candidates.
    const double stddev = std::sqrt(std::max(0.0, deviation_sum) / (sampled.size() - 1));
    const double threshold = mean + 3.0 * stddev;
    isolated.erase(std::remove_if(isolated.begin(), isolated.end(), [&](std::size_t i) {
        return mean_distances[i] <= threshold;
    }), isolated.end());
    stats.outlier_candidates = isolated.size();
    if (isolated.empty()) return result;

    std::vector<std::size_t> raw(positions.size());
    std::iota(raw.begin(), raw.end(), std::size_t{0});
    const SpatialIndex raw_index(positions, std::move(raw));
    for (const auto i : isolated) {
        if (raw_index.nearest(sampled[i], 1, support_radius * support_radius).size == 0) {
            result[i].remove = true;
            ++stats.outlier_removed;
        }
    }
    return result;
}

}  // namespace detail

// Call on the deskewed forward cloud, before the original surface voxel filter.
// The cloud uses the PCL points/width/height/is_dense interface and RotationMatrix
// accepts an Eigen::Matrix3d. Range is measured from the deskewed LiDAR origin.
template <typename Cloud, typename RotationMatrix>
Stats filter(Cloud& cloud, const RotationMatrix& lidar_to_body, const Options& options = Options{})
{
    const double range_squared = options.max_range * options.max_range;
    if (!(range_squared > 0.0 && std::isfinite(range_squared) && options.max_range > 0.0)
        || options.retention_percent < 0 || options.retention_percent > 100) {
        throw std::invalid_argument("invalid native point cloud filter options");
    }
    detail::Rotation rotation{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            const double value = lidar_to_body(row, column);
            if (!std::isfinite(value)) throw std::invalid_argument("invalid lidar to body rotation");
            rotation[row][column] = value;
        }
    }

    Stats stats;
    stats.input_count = cloud.points.size();
    std::vector<std::size_t> eligible;
    eligible.reserve(stats.input_count);
    std::vector<detail::XYZ> uncropped_support;
    for (std::size_t i = 0; i < stats.input_count; ++i) {
        const auto point = detail::coordinates(cloud.points[i]);
        if (!detail::is_forward(point, rotation[0])) continue;
        if (detail::dot(point, point) <= range_squared) {
            eligible.push_back(i);
        } else if (options.smoothing) {
            uncropped_support.push_back(point);
        }
    }
    stats.eligible_count = eligible.size();
    const auto retention = static_cast<std::size_t>(options.retention_percent);
    stats.sampled_count = (eligible.size() / 100) * retention + (eligible.size() % 100) * retention / 100;

    // Take the midpoint of each equal interval in eligible scan order. An
    // integer accumulator avoids overflow from multiplying two point counts.
    std::vector<std::size_t> sampled;
    sampled.reserve(stats.sampled_count);
    if (stats.sampled_count > 0) {
        auto accumulator = (eligible.size() - 1) / 2;
        const auto step_to_select = eligible.size() - stats.sampled_count;
        for (std::size_t i = 0; i < eligible.size(); ++i) {
            if (accumulator >= step_to_select) {
                sampled.push_back(i);
                accumulator -= step_to_select;
            } else {
                accumulator += stats.sampled_count;
            }
        }
    }

    std::vector<detail::Refinement> refinements;
    if (options.smoothing) {
        std::vector<detail::XYZ> positions;
        positions.reserve(eligible.size() + uncropped_support.size());
        for (const auto i : eligible) positions.push_back(detail::coordinates(cloud.points[i]));
        // These original returns only protect edge points from outlier
        // removal. They cannot enter matching, sampling or a plane fit.
        positions.insert(positions.end(), uncropped_support.begin(), uncropped_support.end());
        refinements = detail::refine(positions, sampled, rotation, stats);
    }

    // Stable compaction keeps the complete point, including its deskew time,
    // intensity and normals. Test limits using the coordinates as stored.
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        if (options.smoothing && refinements[i].remove) continue;
        auto point = cloud.points[eligible[sampled[i]]];
        if (options.smoothing) {
            const auto& offset = refinements[i].offset;
            if (detail::dot(offset, offset) > 0.0) {
                const auto original = detail::coordinates(point);
                auto corrected = point;
                corrected.x = static_cast<decltype(point.x)>(original[0] + offset[0]);
                corrected.y = static_cast<decltype(point.y)>(original[1] + offset[1]);
                corrected.z = static_cast<decltype(point.z)>(original[2] + offset[2]);
                const auto proposed = detail::coordinates(corrected);
                const double shift_squared = detail::squared_distance(proposed, original);
                if (shift_squared > 0.0 && shift_squared <= detail::max_shift * detail::max_shift
                    && detail::is_eligible(proposed, rotation[0], range_squared)) {
                    point = corrected;
                    ++stats.smoothed_count;
                }
            }
        }
        cloud.points[stats.output_count++] = point;
    }
    cloud.points.resize(stats.output_count);
    cloud.width = static_cast<decltype(cloud.width)>(stats.output_count);
    cloud.height = 1;
    cloud.is_dense = true;
    return stats;
}

}  // namespace native_filter
