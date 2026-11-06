#ifndef FAST_LIO_NATIVE_TEMPORAL_FILTER_HPP
#define FAST_LIO_NATIVE_TEMPORAL_FILTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace native_temporal
{

struct Point
{
    double x, y, z;

    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct Options
{
    double voxel_size = 0.05;
    double max_range = 3.0;
    double min_clear_range = 0.5;
    double endpoint_margin = 0.15;
    double hit_seconds = 1.0;
    double free_seconds = 0.3;
    double candidate_gap = 0.5;
    unsigned hit_frames = 5;
    unsigned free_frames = 4;
    std::size_t max_voxels = 500000;
    std::size_t max_frame_points = 40000;
    std::size_t max_clear_rays = 2048;
};

enum class Status
{
    ok, invalid_stamp, nonincreasing_stamp, invalid_origin, invalid_cloud, frame_capacity
};

struct Stats
{
    Status status = Status::ok;
    std::size_t raw_hits = 0;
    std::size_t candidate_hits = 0;
    std::size_t available_clear_rays = 0;
    std::size_t clear_rays = 0;
    std::size_t capacity_drops = 0;
};

// World-frame inputs share a scan-end pose. Use genuine front-half deskewed
// returns for raw_world, including returns beyond 3 m. The caller must disable
// clearing when the scan-end origin is a poor approximation to ray origins.
// The caller owns the native tree and applies deletions by the returned indices.
class TemporalFilter
{
public:
    explicit TemporalFilter(Options options = {}) : options_(options)
    {
        const auto positive = [](double n) { return std::isfinite(n) && n > 0; };
        const auto nonnegative = [](double n) { return std::isfinite(n) && n >= 0; };
        if (!positive(options_.voxel_size) || !positive(options_.max_range) ||
            options_.max_range > 3.0 || options_.max_range / options_.voxel_size > 128 ||
            !positive(options_.endpoint_margin) || !positive(options_.candidate_gap) ||
            !nonnegative(options_.min_clear_range) || options_.min_clear_range >= options_.max_range ||
            !nonnegative(options_.hit_seconds) || !nonnegative(options_.free_seconds) ||
            !options_.hit_frames || !options_.free_frames || !options_.max_voxels ||
            !options_.max_frame_points || !options_.max_clear_rays)
            throw std::invalid_argument("invalid native temporal filter options");
    }

    const Options &options() const { return options_; }

    // Call only for an explicit new scan timestamp epoch.
    void resetEvidence()
    {
        invalidateFrame();
        has_stamp_ = false;
        ray_phase_ = 0;
    }

    Stats observe(const std::vector<Point> &raw_world, const std::vector<Point> &candidate_world,
                  Point origin, double stamp, bool allow_clear)
    {
        if (!std::isfinite(stamp)) return reject(Status::invalid_stamp);
        if (has_stamp_ && stamp <= stamp_) return reject(Status::nonincreasing_stamp);
        if (!worldPoint(origin, options_.max_range)) return reject(Status::invalid_origin);
        if (raw_world.size() > options_.max_frame_points || candidate_world.size() > options_.max_frame_points)
            return reject(Status::frame_capacity);

        const bool continuous = has_stamp_ && stamp - stamp_ <= options_.candidate_gap + time_epsilon();
        if (!continuous) invalidateFrame();
        if (!allow_clear) free_.clear();
        clearFrame();
        stamp_ = stamp;
        has_stamp_ = true;
        origin_ = origin;
        expire(hits_);
        expire(free_);

        if (continuous && allow_clear) angular_nearest_.assign(azimuth_bins() * elevation_bins(), Ray{});
        const double protection_range = options_.max_range + 2 * std::sqrt(3.0) * options_.voxel_size;
        const double protection_sq = protection_range * protection_range;
        std::size_t valid_returns = 0;
        for (const auto &point : raw_world)
        {
            if (!worldPoint(point)) continue;
            const Point offset = difference(point, origin_);
            const double distance_sq = dot(offset, offset);
            if (distance_sq <= 0 || !std::isfinite(distance_sq)) continue;
            ++valid_returns;
            if (distance_sq <= protection_sq) raw_hits_.insert(key(cell(point)));
            if (!angular_nearest_.empty()) addReturn(offset, std::sqrt(distance_sq));
        }
        if (!valid_returns) return reject(Status::invalid_cloud);

        Stats stats;
        stats.raw_hits = raw_hits_.size();
        if (!angular_nearest_.empty()) selectRays(stats);
        for (const auto &point : candidate_world)
        {
            if (!inRange(point)) continue;
            const Key candidate_key = key(cell(point));
            if (!candidate_hits_.insert(candidate_key).second) continue;
            auto evidence = hits_.find(candidate_key);
            if (evidence == hits_.end())
            {
                if (hits_.size() + free_.size() >= options_.max_voxels)
                {
                    ++stats.capacity_drops;
                    continue;
                }
                evidence = hits_.emplace(candidate_key, Evidence{}).first;
            }
            evidence->second.add(stamp_, options_.candidate_gap);
        }
        // A raw hit vetoes older free evidence even when the current scan
        // has no eligible clear rays and the caller skips the tree snapshot.
        for (auto it = free_.begin(); it != free_.end(); )
        {
            if (protectedCell(cell(it->first))) it = free_.erase(it);
            else ++it;
        }
        stats.candidate_hits = candidate_hits_.size();
        return stats;
    }

    bool admit(Point world) const
    {
        if (!inRange(world)) return false;
        const Key candidate_key = key(cell(world));
        if (!candidate_hits_.count(candidate_key)) return false;
        const auto evidence = hits_.find(candidate_key);
        return evidence != hits_.end() && evidence->second.meets(options_.hit_frames, options_.hit_seconds);
    }

    bool needsRemovalCandidates() const { return !rays_.empty(); }

    // Call at most once per observed frame before mutating the tree. An empty
    // snapshot still lets a true crossing contradict old admission evidence.
    std::vector<std::size_t> collectRemovals(const std::vector<Point> &native_world)
    {
        std::vector<std::size_t> removals;
        if (rays_.empty()) return removals;
        if (native_world.size() > options_.max_frame_points)
        {
            rays_.clear();
            free_.clear();
            return removals;
        }

        const std::size_t end = native_world.size();
        std::unordered_map<Key, std::size_t> targets;
        targets.reserve(end);
        std::vector<std::size_t> next(end, end);
        std::vector<bool> witnessed(end, false);
        for (std::size_t i = 0; i < end; ++i)
        {
            if (!inRange(native_world[i])) continue;
            const auto head = targets.emplace(key(cell(native_world[i])), i);
            if (head.second) continue;
            next[i] = head.first->second;
            head.first->second = i;
        }

        for (const auto &ray : rays_)
        {
            traverse(ray, [&](Cell voxel, double entry, double exit) {
                const Key voxel_key = key(voxel);
                const auto target = targets.find(voxel_key);
                const auto hit = hits_.find(voxel_key);
                if (target == targets.end() && hit == hits_.end()) return;
                if (protectedCell(voxel)) return;
                if (hit != hits_.end()) hits_.erase(hit);
                if (target == targets.end()) return;
                for (auto i = target->second; i != end; i = next[i])
                {
                    if (witnessed[i]) continue;
                    const Point offset = difference(native_world[i], origin_);
                    const double projection = dot(offset, ray.direction);
                    if (projection < entry || projection > exit) continue;
                    // Keep points elsewhere in a crossed voxel unless the
                    // actual ray also passes close to this exact point.
                    const double tolerance = 0.4 * options_.voxel_size;
                    const Point residual = difference(offset, scale(ray.direction, projection));
                    witnessed[i] = dot(residual, residual) <= tolerance * tolerance;
                }
            });
        }
        rays_.clear();

        for (std::size_t i = 0; i < end; ++i)
        {
            if (!witnessed[i]) continue;
            const Point point = native_world[i];
            auto evidence = free_.find(point);
            if (evidence == free_.end())
            {
                if (hits_.size() + free_.size() >= options_.max_voxels) continue;
                evidence = free_.emplace(point, Evidence{}).first;
            }
            evidence->second.add(stamp_, options_.candidate_gap);
            if (evidence->second.meets(options_.free_frames, options_.free_seconds)) removals.push_back(i);
        }
        for (const auto i : removals) free_.erase(native_world[i]);
        return removals;
    }

private:
    using Key = std::uint64_t;
    using Cell = std::array<int, 3>;

    static constexpr double time_epsilon() { return 1e-6; }
    static constexpr int key_bias() { return 1 << 20; }
    static constexpr int azimuth_bins() { return 629; }
    static constexpr int elevation_bins() { return 315; }
    static constexpr double pi() { return 3.14159265358979323846; }

    struct Evidence
    {
        double first = 0;
        double last = 0;
        unsigned count = 0;

        void add(double stamp, double gap)
        {
            if (count && stamp == last) return;
            if (!count || stamp - last > gap + time_epsilon())
            {
                first = stamp;
                count = 0;
            }
            last = stamp;
            if (count < std::numeric_limits<unsigned>::max()) ++count;
        }

        bool meets(unsigned frames, double seconds) const
        {
            return count >= frames && last - first + time_epsilon() >= seconds;
        }
    };

    struct PointHash
    {
        std::size_t operator()(Point point) const
        {
            std::size_t hash = 0;
            for (double axis : {point.x, point.y, point.z})
                hash ^= std::hash<double>{}(axis) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
            return hash;
        }
    };

    struct Ray
    {
        Point direction{0, 0, 0};
        double length = std::numeric_limits<double>::infinity();
    };

    static Point difference(Point a, Point b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    static Point scale(Point a, double factor) { return {a.x * factor, a.y * factor, a.z * factor}; }
    static double dot(Point a, Point b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

    bool worldPoint(Point point, double padding = 0) const
    {
        const double limit = (key_bias() - 2) * options_.voxel_size - padding;
        return std::abs(point.x) < limit && std::abs(point.y) < limit && std::abs(point.z) < limit;
    }

    bool inRange(Point point) const
    {
        if (!worldPoint(point)) return false;
        const Point offset = difference(point, origin_);
        const double distance_sq = dot(offset, offset);
        return distance_sq > 0 && distance_sq <= options_.max_range * options_.max_range;
    }

    Cell cell(Point point) const
    {
        return {{static_cast<int>(std::floor(point.x / options_.voxel_size)),
                 static_cast<int>(std::floor(point.y / options_.voxel_size)),
                 static_cast<int>(std::floor(point.z / options_.voxel_size))}};
    }

    static Key key(Cell voxel)
    {
        return (static_cast<Key>(voxel[0] + key_bias()) << 42) |
               (static_cast<Key>(voxel[1] + key_bias()) << 21) |
               static_cast<Key>(voxel[2] + key_bias());
    }

    bool protectedCell(Cell voxel) const
    {
        if (candidate_hits_.count(key(voxel))) return true;
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z)
                    if (raw_hits_.count(key({{voxel[0] + x, voxel[1] + y, voxel[2] + z}}))) return true;
        return false;
    }

    void addReturn(Point offset, double distance)
    {
        const double azimuth = std::atan2(offset.y, offset.x);
        const double elevation = std::atan2(offset.z, std::hypot(offset.x, offset.y));
        const int az = static_cast<int>((azimuth + pi()) * azimuth_bins() / (2 * pi())) % azimuth_bins();
        const int el = std::min(elevation_bins() - 1,
                                static_cast<int>((elevation + pi() / 2) * elevation_bins() / pi()));
        auto &nearest = angular_nearest_[el * azimuth_bins() + az];
        if (distance < nearest.length) nearest = {scale(offset, 1 / distance), distance};
    }

    void selectRays(Stats &stats)
    {
        std::vector<Ray> available;
        for (int el = 0; el < elevation_bins(); ++el)
        {
            for (int az = 0; az < azimuth_bins(); ++az)
            {
                auto ray = angular_nearest_[el * azimuth_bins() + az];
                if (!std::isfinite(ray.length)) continue;
                // Look at all raw directions before enforcing the ray budget.
                for (int dy = -1; dy <= 1; ++dy)
                {
                    const int other_el = el + dy;
                    if (other_el < 0 || other_el >= elevation_bins()) continue;
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        const int other_az = (az + dx + azimuth_bins()) % azimuth_bins();
                        ray.length = std::min(ray.length, angular_nearest_[other_el * azimuth_bins() + other_az].length);
                    }
                }
                ray.length = std::min(options_.max_range, ray.length - options_.endpoint_margin);
                if (ray.length > options_.min_clear_range) available.push_back(ray);
            }
        }
        stats.available_clear_rays = available.size();
        if (available.size() <= options_.max_clear_rays) rays_.swap(available);
        else
        {
            for (std::size_t i = 0; i < options_.max_clear_rays; ++i)
            {
                const auto slot = i * available.size() / options_.max_clear_rays;
                rays_.push_back(available[(slot + ray_phase_) % available.size()]);
            }
            ray_phase_ = (ray_phase_ + 1) % available.size();
        }
        stats.clear_rays = rays_.size();
    }

    template <typename Visit>
    void traverse(const Ray &ray, const Visit &visit) const
    {
        Cell voxel = cell(origin_);
        if (raw_hits_.count(key(voxel))) return;
        const Point origin = origin_;
        const double position[] = {origin.x, origin.y, origin.z};
        const double direction[] = {ray.direction.x, ray.direction.y, ray.direction.z};
        int step[3];
        double boundary[3], delta[3];
        for (int axis = 0; axis < 3; ++axis)
        {
            step[axis] = (direction[axis] > 0) - (direction[axis] < 0);
            boundary[axis] = delta[axis] = std::numeric_limits<double>::infinity();
            if (!step[axis]) continue;
            const double edge = (voxel[axis] + (step[axis] > 0)) * options_.voxel_size;
            boundary[axis] = std::max(0.0, (edge - position[axis]) / direction[axis]);
            delta[axis] = options_.voxel_size / std::abs(direction[axis]);
        }

        int axis = std::min_element(boundary, boundary + 3) - boundary;
        while (boundary[axis] < ray.length)
        {
            const double entry = boundary[axis];
            voxel[axis] += step[axis];
            if (raw_hits_.count(key(voxel))) return;
            boundary[axis] += delta[axis];
            axis = std::min_element(boundary, boundary + 3) - boundary;
            const double exit = boundary[axis];
            if (entry >= options_.min_clear_range && exit <= ray.length &&
                exit - entry >= 0.5 * options_.voxel_size)
                visit(voxel, entry, exit);
        }
    }

    template <typename Map>
    void expire(Map &map)
    {
        for (auto it = map.begin(); it != map.end(); )
        {
            if (stamp_ - it->second.last > options_.candidate_gap + time_epsilon()) it = map.erase(it);
            else ++it;
        }
    }

    void clearFrame()
    {
        raw_hits_.clear();
        candidate_hits_.clear();
        rays_.clear();
        angular_nearest_.clear();
    }

    void invalidateFrame()
    {
        clearFrame();
        hits_.clear();
        free_.clear();
    }

    Stats reject(Status status)
    {
        invalidateFrame();
        Stats stats;
        stats.status = status;
        return stats;
    }

    Options options_;
    Point origin_{0, 0, 0};
    double stamp_ = 0;
    bool has_stamp_ = false;
    std::size_t ray_phase_ = 0;
    std::unordered_set<Key> raw_hits_, candidate_hits_;
    std::unordered_map<Key, Evidence> hits_;
    std::unordered_map<Point, Evidence, PointHash> free_;
    std::vector<Ray> angular_nearest_, rays_;
};

}  // namespace native_temporal

#endif  // FAST_LIO_NATIVE_TEMPORAL_FILTER_HPP
