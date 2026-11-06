#include "ikd_Tree.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Point = pcl::PointXYZINormal;
using Tree = KD_TREE<Point>;
using Points = Tree::PointVector;
using Expected = std::map<int, Point>;

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

Point make_point(int id, float x, float y, float z)
{
    Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = static_cast<float>(id);
    point.normal_x = 0.125f;
    point.normal_y = 0.25f;
    point.normal_z = 0.5f;
    point.curvature = 0.001f * static_cast<float>(id);
    return point;
}

int id_of(const Point& point)
{
    return static_cast<int>(point.intensity);
}

bool same_payload(const Point& a, const Point& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity
        && a.curvature == b.curvature && a.normal_x == b.normal_x
        && a.normal_y == b.normal_y && a.normal_z == b.normal_z;
}

Expected index_points(const Points& points)
{
    Expected result;
    for (const auto& point : points) {
        require(result.emplace(id_of(point), point).second, "duplicate point in fixture");
    }
    return result;
}

void require_points(const Points& actual, const Expected& expected, const std::string& label)
{
    if (actual.size() != expected.size()) {
        throw std::runtime_error(label + " count " + std::to_string(actual.size())
                                 + ", expected " + std::to_string(expected.size()));
    }
    auto remaining = expected;
    for (const auto& point : actual) {
        const auto it = remaining.find(id_of(point));
        require(it != remaining.end(), label + " unexpected or duplicate ID " + std::to_string(id_of(point)));
        require(same_payload(point, it->second), label + " changed payload");
        remaining.erase(it);
    }
}

void require_snapshot(Tree& tree, const Expected& expected, const std::string& label,
                      bool rebuild_idle = true)
{
    Points result{make_point(-1, 0.0f, 0.0f, 0.0f)};
    tree.Snapshot_Points(result);
    require_points(result, expected, label);
    if (rebuild_idle) {
        require(tree.validnum() == static_cast<int>(result.size()), label + " validnum mismatch");
    }
}

void require_radius(Tree& tree, const Expected& points, const Point& center, float radius,
                    const std::string& label)
{
    Points result{make_point(-1, 0.0f, 0.0f, 0.0f)};
    tree.Snapshot_Radius(center, radius, result);
    Expected expected;
    for (const auto& entry : points) {
        const auto& point = entry.second;
        const float dx = point.x - center.x;
        const float dy = point.y - center.y;
        const float dz = point.z - center.z;
        if (dx * dx + dy * dy + dz * dz <= radius * radius) expected.insert(entry);
    }
    require_points(result, expected, label);
}

void exact_delete_keeps_same_voxel_points()
{
    // KD_TREE includes a large embedded operation log. Allocate it on the heap.
    auto tree = std::make_unique<Tree>(0.5f, 0.7f, 0.10f);
    const auto stale = make_point(1, 1.025f, 0.025f, 0.025f);
    const auto rod_lower = make_point(2, 1.035f, 0.035f, 0.030f);
    const auto rod_upper = make_point(3, 1.035f, 0.035f, 0.085f);
    const auto boundary = make_point(4, 1.25f, 0.0f, 0.0f);
    const auto outside = make_point(5, 1.26f, 0.0f, 0.0f);
    const auto adjacent = make_point(6, std::nextafter(stale.x, 2.0f), stale.y, stale.z);
    const Points source{stale, rod_lower, rod_upper, boundary, outside, adjacent};
    auto expected = index_points(source);
    tree->Build(source);
    require_snapshot(*tree, expected, "initial tree");

    Points removal{stale};
    require(tree->Delete_Exact_Points(removal) == 1, "one exact point should be deleted");
    expected.erase(id_of(stale));
    require_snapshot(*tree, expected, "same voxel rods after point deletion");
    require_radius(*tree, expected, make_point(0, 1.0f, 0.0f, 0.0f), 0.25f,
                   "radius includes exact boundary");

    require(tree->Delete_Exact_Points(removal) == 0, "repeated deletion changed the tree");
    require_snapshot(*tree, expected, "repeated exact deletion");
}

const Tree::KD_TREE_NODE* find_equal_x(const Tree::KD_TREE_NODE* node, float x)
{
    if (node == nullptr || node->point.x == x) return node;
    const auto* left = find_equal_x(node->left_son_ptr, x);
    return left != nullptr ? left : find_equal_x(node->right_son_ptr, x);
}

void exact_delete_finds_equal_split_in_left_branch()
{
    auto tree = std::make_unique<Tree>();
    const Points source{
        make_point(1, -4.0f, 0.0f, 0.0f),
        make_point(2, 1.0f, 0.01f, 0.01f),
        make_point(3, 1.0f, 0.02f, 0.02f),
        make_point(4, 1.0f, 0.03f, 0.03f),
        make_point(5, 1.0f, 0.04f, 0.04f),
        make_point(6, 1.0f, 0.05f, 0.05f),
        make_point(7, 5.0f, 0.0f, 0.0f)
    };
    auto expected = index_points(source);
    tree->Build(source);
    const auto* root = tree->Root_Node;
    require(root->division_axis == 0 && root->point.x == 1.0f, "equal split fixture");
    const auto* target = find_equal_x(root->left_son_ptr, root->point.x);
    require(target != nullptr, "equal split point must exist in the left branch");
    Points removal{target->point};
    expected.erase(id_of(removal.front()));
    require(tree->Delete_Exact_Points(removal) == 1, "equal split deletion count");
    require_snapshot(*tree, expected, "deleting the exact point on the equal split plane");
}

void all_deleted_tree_can_accept_points()
{
    auto tree = std::make_unique<Tree>(1.0f, 1.0f, 0.1f);
    Points source;
    for (int i = 0; i < 19; ++i) source.push_back(make_point(i + 1, i * 0.20f, 0.0f, 0.0f));
    tree->Build(source);
    Points removal(source.begin() + 1, source.end());
    require(tree->Delete_Exact_Points(removal) == static_cast<int>(removal.size()), "bulk deletion count");
    require_snapshot(*tree, index_points(Points{source.front()}), "last live point");
    tree->Set_delete_criterion_param(0.5f);
    removal.assign(1, source.front());
    require(tree->Delete_Exact_Points(removal) == 1, "last point deletion count");
    require_snapshot(*tree, {}, "all points deleted");
    require_radius(*tree, {}, source.front(), 10.0f, "empty tree radius");
    require(tree->Delete_Exact_Points(removal) == 0, "empty tree deletion count");
    require_snapshot(*tree, {}, "repeat deletion on empty tree");

    Points additions{make_point(100, 0.15f, 0.015f, 0.015f)};
    tree->Add_Points(additions, false);
    require_snapshot(*tree, index_points(additions), "insertion after all points deleted");
    Points voxel_addition{make_point(101, 0.25f, 0.025f, 0.025f)};
    tree->Add_Points(voxel_addition, true);
    additions.push_back(voxel_addition.front());
    require_snapshot(*tree, index_points(additions), "voxel insertion after tree revival");
}

bool removal_reported(Tree& tree, int marker_id)
{
    Points removed;
    tree.acquire_removed_points(removed);
    return std::any_of(removed.begin(), removed.end(), [&](const Point& point) {
        return id_of(point) == marker_id;
    });
}

void background_rebuild_and_snapshots()
{
    auto tree = std::make_unique<Tree>(1.0f, 1.0f, 0.10f);
    const int initial_count = Multi_Thread_Rebuild_Point_Num * 6;
    Points seed;
    seed.reserve(initial_count);
    for (int i = 0; i < initial_count; ++i) {
        seed.push_back(make_point(i + 1, 0.006f * i, 0.017f * (i % 103),
                                  0.019f * (i % 67)));
    }
    tree->Build(seed);
    auto expected = index_points(seed);
    const Point old_root = tree->Root_Node->point;
    const int marker_id = id_of(old_root);
    Points removal{old_root};
    for (std::size_t i = 0; i < seed.size(); i += 3) {
        if (id_of(seed[i]) != marker_id) removal.push_back(seed[i]);
    }
    for (const auto& point : removal) expected.erase(id_of(point));
    require(tree->Delete_Exact_Points(removal) == static_cast<int>(removal.size()),
            "background workload deletion count");
    require_snapshot(*tree, expected, "bulk point deletions");
    require(!removal_reported(*tree, marker_id), "fixture root rebuilt before trigger");

    // A deleted root can only reach the removal cache when a root rebuild
    // compacts it. There are more than 1500 live points throughout this test.
    tree->Set_delete_criterion_param(0.10f);
    bool background_observed = false;
    for (int batch = 0; batch < 24; ++batch) {
        Points additions;
        for (int j = 0; j < 48; ++j) {
            const int i = batch * 48 + j;
            const auto point = make_point(initial_count + 1 + i, -0.006f * (i + 1),
                                          0.017f * (i % 103), 0.019f * (i % 67));
            additions.push_back(point);
            expected.emplace(id_of(point), point);
        }
        tree->Add_Points(additions, false);

        // This traversal must remain valid even when it arrives while the
        // worker owns the rebuild lock. The caller has one foreground thread.
        require_radius(*tree, expected, make_point(0, 12.0f, 0.5f, 0.5f), 2.321f,
                       "radius during rebuild workload");
        require_snapshot(*tree, expected, "snapshot during rebuild workload", false);
        if (!background_observed) background_observed = removal_reported(*tree, marker_id);
        if (!background_observed) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (int attempt = 0; !background_observed && attempt < 100; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        background_observed = removal_reported(*tree, marker_id);
    }
    require(background_observed, "background rebuild did not compact the deleted root");
    require_snapshot(*tree, expected, "snapshot after verified background root rebuild");
    require_radius(*tree, expected, make_point(0, -2.0f, 0.5f, 0.5f), 1.372f,
                   "radius after background root rebuild");
    std::cout << "Observed background root compaction with " << expected.size()
              << " expected live points" << std::endl;
}

}  // namespace

int main()
{
    try {
        std::cout << "Checking exact point deletion and radius" << std::endl;
        exact_delete_keeps_same_voxel_points();
        std::cout << "Checking a deletion on a tied split axis" << std::endl;
        exact_delete_finds_equal_split_in_left_branch();
        std::cout << "Checking insertion after complete deletion" << std::endl;
        all_deleted_tree_can_accept_points();
        std::cout << "Checking snapshots and exact content under background rebuild workload" << std::endl;
        background_rebuild_and_snapshots();
        std::cout << "All native ikd-tree regressions passed" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
