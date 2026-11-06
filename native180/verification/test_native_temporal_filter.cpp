#include "native_temporal_filter.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
using native_temporal::Options;
using native_temporal::Point;
using native_temporal::Status;
using native_temporal::TemporalFilter;

constexpr Point origin{0.026, 0.026, 0.026};
constexpr Point obstacle{1.026, 0.026, 0.026};
constexpr Point background{4.026, 0.026, 0.026};
constexpr double epoch = 1760000000.0;

void require(bool condition, const std::string &message)
{
    if (condition) return;
    std::cerr << message << '\n';
    std::exit(1);
}

std::vector<std::size_t> scan(TemporalFilter &filter, double t,
                              const std::vector<Point> &raw,
                              const std::vector<Point> &candidates,
                              const std::vector<Point> &native = {}, bool allow_clear = true)
{
    require(filter.observe(raw, candidates, origin, epoch + t, allow_clear).status == Status::ok,
            "scan rejected");
    return filter.collectRemovals(native);
}

void movingAndStopped()
{
    TemporalFilter filter;
    for (int i = 0; i <= 12; ++i)
    {
        const Point moving{0.626 + i * 0.11, obstacle.y, obstacle.z};
        scan(filter, i * 0.1, {moving, background}, {moving, moving});
        require(!filter.admit(moving), "moving point admitted or duplicate hits counted as frames");
    }
    for (int i = 13; i <= 18; ++i)
    {
        scan(filter, i * 0.1, {obstacle, background}, {obstacle});
        require(!filter.admit(obstacle), "brief stop admitted before one second");
    }
    scan(filter, 1.9, {background}, {});
    require(!filter.admit(obstacle), "absent candidate still admitted");
    for (int i = 20; i <= 30; ++i)
    {
        scan(filter, i * 0.1, {obstacle, background}, {obstacle});
        require(filter.admit(obstacle) == (i == 30), "free traversal did not restart admission evidence");
    }
}

void departedAndAdjacent()
{
    TemporalFilter filter;
    const Point beside{obstacle.x, 0.049, obstacle.z};  // Same voxel, outside the ray tube.
    for (int i = 0; i <= 10; ++i)
        scan(filter, i * 0.1, {obstacle, background}, {obstacle});
    require(filter.admit(obstacle), "one second of stable observations did not admit");
    for (int i = 11; i <= 14; ++i)
    {
        const auto removed = scan(filter, i * 0.1, {background}, {}, {beside, obstacle, obstacle});
        require(removed == (i == 14 ? std::vector<std::size_t>{1, 2} : std::vector<std::size_t>{}),
                "did not return only exact witnessed points after four distinct free frames");
    }
    require(filter.collectRemovals({obstacle}).empty(), "same frame consumed twice");
}

void thinPostsAndOcclusion()
{
    TemporalFilter filter;
    const Point thin_post{obstacle.x, 0.076, obstacle.z};  // Neighboring voxel and angular bin.
    const Point near_occluder{0.826, 0.048, obstacle.z};  // Ray crosses its voxel from a nonadjacent angle bin.
    const Point in_margin{0.753, obstacle.y, obstacle.z};
    for (int i = 0; i <= 3; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {obstacle}).empty(), "deleted too soon");
    // Neither the thin post nor its neighboring tree point survived thinning
    // into the candidate list, but the full raw hit resets the free sequence.
    require(scan(filter, 0.4, {thin_post, background}, {}, {obstacle, thin_post}).empty(),
            "raw-only thin post failed to protect a neighboring tree point");
    for (int i = 5; i <= 8; ++i)
    {
        const auto removed = scan(filter, i * 0.1, {background}, {}, {obstacle});
        require(removed.size() == (i == 8 ? 1u : 0u), "raw hit did not reset old free evidence");
    }
    for (int i = 9; i <= 16; ++i)
        require(scan(filter, i * 0.1, {background, near_occluder}, {}, {obstacle, in_margin}).empty(),
                "far ray cleared behind a nearer return or in its endpoint margin");
    for (int i = 17; i <= 22; ++i)
        require(scan(filter, i * 0.1, {{0.026, 3.026, 0.026}}, {}, {obstacle}).empty(),
                "unobserved native point was removed");
}

void signedAxesAndEndpointMargin()
{
    const std::vector<Point> directions{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                                        {0, 0, 1}, {0, 0, -1}, {-1, 1, -1}};
    for (const auto direction : directions)
    {
        TemporalFilter filter;
        const Point departed{origin.x + direction.x, origin.y + direction.y, origin.z + direction.z};
        const Point far{origin.x + 4 * direction.x, origin.y + 4 * direction.y, origin.z + 4 * direction.z};
        for (int i = 0; i <= 4; ++i)
            require(scan(filter, i * 0.1, {far}, {}, {departed}).size() == (i == 4 ? 1u : 0u),
                    "signed ray or simultaneous DDA crossings mishandled");
    }

    TemporalFilter filter;
    const Point end{1.180, obstacle.y, obstacle.z};
    const Point inside_margin{1.075, obstacle.y, obstacle.z};
    const Point before_margin{0.976, obstacle.y, obstacle.z};
    for (int i = 0; i <= 4; ++i)
        require(scan(filter, i * 0.1, {end}, {}, {inside_margin, before_margin}) ==
                    (i == 4 ? std::vector<std::size_t>{1} : std::vector<std::size_t>{}),
                "endpoint margin was too short or removed a fully traversed earlier voxel");
}

void protectionWithoutClearRays()
{
    TemporalFilter filter;
    const Point near_obstacle{0.576, obstacle.y, obstacle.z};
    for (int i = 0; i <= 3; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {near_obstacle}).empty(), "deleted too soon");
    const auto stats = filter.observe({near_obstacle}, {}, origin, epoch + 0.4, true);
    require(stats.raw_hits == 1 && !filter.needsRemovalCandidates(), "short hit should veto with zero rays");
    for (int i = 5; i <= 8; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {near_obstacle}).size() == (i == 8 ? 1u : 0u),
                "hit without clear rays did not invalidate old free evidence");
}

void discontinuitiesAndMotion()
{
    TemporalFilter filter;
    for (int i = 0; i <= 10; ++i)
        scan(filter, i * 0.1, {obstacle}, {obstacle}, {}, false);
    require(filter.admit(obstacle), "motion disabled positive hit evidence");
    for (int i = 11; i <= 13; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {obstacle}).empty(), "early removal before motion");
    require(scan(filter, 1.4, {background}, {}, {obstacle}, false).empty(), "motion cast a clear ray");
    for (int i = 15; i <= 17; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {obstacle}).empty(), "motion retained free history");
    require(filter.observe({}, {}, origin, epoch + 1.8, true).status == Status::invalid_cloud,
            "empty raw frame accepted");
    require(scan(filter, 1.9, {background}, {}, {obstacle}).empty(), "empty scan retained free history");
    require(filter.observe({background}, {}, origin, epoch + 1.8, true).status == Status::nonincreasing_stamp,
            "time rollback accepted");
    require(filter.collectRemovals({obstacle}).empty(), "time rollback left clear rays active");
    require(filter.observe({background}, {}, origin, std::numeric_limits<double>::quiet_NaN(), true).status ==
                Status::invalid_stamp,
            "nonfinite stamp accepted");
    require(filter.observe({background}, {}, origin, epoch + 1.9, true).status == Status::nonincreasing_stamp,
            "duplicate stamp accepted");
    require(scan(filter, 2.0, {background}, {}, {obstacle}).empty(), "invalid stamps retained old free history");
    require(scan(filter, 3.0, {background}, {}, {obstacle}).empty(), "long gap applied free evidence");
    for (int i = 31; i <= 34; ++i)
        require(scan(filter, i * 0.1, {background}, {}, {obstacle}).size() == (i == 34 ? 1u : 0u),
                "clear evidence crossed a long gap");
    filter.resetEvidence();
    require(scan(filter, 0, {background}, {}, {obstacle}).empty(), "explicit epoch reset failed");
}

void boundsAndCapacity()
{
    TemporalFilter filter;
    const Point too_near{0.516, obstacle.y, obstacle.z};
    const Point beyond{3.027, obstacle.y, obstacle.z};
    const Point behind{-1.026, obstacle.y, obstacle.z};
    const Point nan{std::numeric_limits<double>::quiet_NaN(), 0, 0};
    const Point huge{1e50, 0, 0};
    for (int i = 0; i <= 10; ++i)
    {
        require(scan(filter, i * 0.1, {background, nan, huge}, {beyond, nan, huge},
                     {too_near, beyond, behind, nan, huge}).empty(), "clear ray left configured valid range");
        require(!filter.admit(beyond) && !filter.admit(nan) && !filter.admit(huge), "invalid point admitted");
    }
    require(filter.observe({background}, {}, huge, epoch + 1.1, true).status == Status::invalid_origin,
            "invalid origin accepted");

    Options options;
    options.max_voxels = 1;
    options.max_frame_points = 3;
    TemporalFilter bounded(options);
    const Point other{obstacle.x + 0.1, obstacle.y, obstacle.z};
    auto stats = bounded.observe({obstacle, other}, {obstacle, other}, origin, epoch, true);
    require(stats.candidate_hits == 2 && stats.capacity_drops == 1, "capacity not enforced");
    for (int i = 1; i <= 10; ++i)
        scan(bounded, i * 0.1, {obstacle, other}, {obstacle, other});
    require(bounded.admit(obstacle) && !bounded.admit(other), "overflow created admission evidence");
    require(bounded.observe({obstacle, obstacle, obstacle, obstacle}, {}, origin, epoch + 1.1, true).status ==
                Status::frame_capacity,
            "oversize frame accepted");
    require(!bounded.admit(obstacle), "oversize frame left stale admission active");
}

void budgetRotation()
{
    Options options;
    options.max_clear_rays = 1;
    TemporalFilter filter(options);
    const std::vector<Point> returns{{4.026, -1.024, 0.026}, background, {4.026, 1.026, 0.026}};
    scan(filter, 0, returns, {});
    bool removed = false;
    for (int i = 1; i <= 12; ++i)
    {
        const auto stats = filter.observe(returns, {}, origin, epoch + i * 0.1, true);
        require(stats.available_clear_rays == 3 && stats.clear_rays == 1, "clear budget not enforced");
        removed = removed || !filter.collectRemovals({obstacle}).empty();
    }
    require(removed, "rotation never visited the single supporting ray often enough");

    TemporalFilter full_budget;
    std::vector<Point> dense_returns;
    for (int i = -30; i <= 30; ++i)
        for (int j = -30; j <= 30; ++j)
            dense_returns.push_back({4.026, 0.026 + i * 0.08, 0.026 + j * 0.08});
    scan(full_budget, 0, dense_returns, {});
    const auto stats = full_budget.observe(dense_returns, {}, origin, epoch + 0.1, true);
    require(stats.available_clear_rays > 2048 && stats.clear_rays == 2048, "default budget changed");
}
}  // namespace

int main()
{
    movingAndStopped();
    departedAndAdjacent();
    thinPostsAndOcclusion();
    signedAxesAndEndpointMargin();
    protectionWithoutClearRays();
    discontinuitiesAndMotion();
    boundsAndCapacity();
    budgetRotation();
    std::cout << "8 native temporal scenarios passed\n";
}
