#include <vector>
#include <functional>
#include "jpsl/dir.hpp"

//
// In JPS, the successors are only generated in the direction of the parent. In expanded neighbrhood, the
// successors are the expanded nodes that are in the same octant as the parent.
//

bool IsInSameOctant(JPSL::Dir a, JPSL::Dir b)
{
    return ((a.dx() > 0 && b.dx() > 0) || (a.dx() < 0 && b.dx() < 0) || a.dx() == 0 || b.dx() == 0) &&
        ((a.dy() > 0 && b.dy() > 0) || (a.dy() < 0 && b.dy() < 0) || a.dy() == 0 || b.dy() == 0) &&
        ((a.dz() > 0 && b.dz() > 0) || (a.dz() < 0 && b.dz() < 0) || a.dz() == 0 || b.dz() == 0);
}

//checks if small is in the same dimention as great or if smaller dimention is contained insine great
bool IsContained(JPSL::Dir small, JPSL::Dir great)
{
    return
        ((great.dx() == 0 && small.dx() == 0) || great.dx() != 0) &&
        ((great.dy() == 0 && small.dy() == 0) || great.dy() != 0) &&
        ((great.dz() == 0 && small.dz() == 0) || great.dz() != 0);
}

bool HasVisibility(const Point& current_node_coord, const Point& neighbour_coord, const std::function<bool(const Point&)>& state_valid) {
    Point dir_vect(neighbour_coord.x() - current_node_coord.x(),
        neighbour_coord.y() - current_node_coord.y(),
        neighbour_coord.z() - current_node_coord.z());

    int steps = std::max({ std::abs(dir_vect.x()), std::abs(dir_vect.y()), std::abs(dir_vect.z()) });
    float x_increment = static_cast<float>(dir_vect.x()) / steps;
    float y_increment = static_cast<float>(dir_vect.y()) / steps;
    float z_increment = static_cast<float>(dir_vect.z()) / steps;

    float x = current_node_coord.x();
    float y = current_node_coord.y();
    float z = current_node_coord.z();

    for (int i = 0; i <= steps; ++i) {
        Point coord(static_cast<int>(round(x)), static_cast<int>(round(y)), static_cast<int>(round(z)));

        if (!state_valid(coord)) {
            return false;
        }

        x += x_increment;
        y += y_increment;
        z += z_increment;
    }

    return true;
}

std::vector<JPSL::Dir> GetExpandedSuccessorsOfSameDir(std::vector<JPSL::Dir> dirs, JPSL::Dir dir, const JPSL::Point& point, const std::function<bool(const Point&)>& state_valid)
{
    std::vector<JPSL::Dir> result;
    for (auto d : dirs)
    {
        if (IsInSameOctant(d, dir) && IsContained(d, dir) && state_valid(point + d) && HasVisibility(point, point + d, state_valid))
        {
            result.push_back(d);
        }
    }
    return result;
}

const std::vector<Dir> NEIGHBORS_3D_K3({
{-1,-1,-1}, \
{-1,-1,1}, \
{-1,1,-1}, \
{-1,1,1}, \
{1,-1,-1}, \
{1,-1,1}, \
{1,1,-1}, \
{1,1,1}, \
{-1,-1,0}, \
{-1,0,-1}, \
{-1,0,1}, \
{-1,1,0}, \
{0,-1,-1}, \
{0,-1,1}, \
{0,1,-1}, \
{0,1,1}, \
{1,-1,0}, \
{1,0,-1}, \
{1,0,1}, \
{1,1,0}, \
{-1,0,0}, \
{0,-1,0}, \
{0,0,-1}, \
{0,0,1}, \
{0,1,0}, \
{1,0,0}, \
    });


const std::vector<Dir> NEIGHBORS_3D_K4({
{-3,-2,-1}, \
{-3,-2,1}, \
{-3,-1,-2}, \
{-3,-1,2}, \
{-3,1,-2}, \
{-3,1,2}, \
{-3,2,-1}, \
{-3,2,1}, \
{-2,-3,-1}, \
{-2,-3,1}, \
{1,-2,-3}, \
{1,-2,3}, \
{-2,-1,-3}, \
{1,2,-3}, \
{2,1,-3}, \
{1,2,3}, \
{1,3,-2}, \
{1,3,2}, \
{-2,-1,3}, \
{2,-3,-1}, \
{-1,2,-3}, \
{-2,1,-3}, \
{2,-3,1}, \
{2,-1,-3}, \
{2,-1,3}, \
{2,1,3}, \
{2,3,-1}, \
{-2,1,3}, \
{-1,2,3}, \
{-1,3,-2}, \
{-2,3,-1}, \
{-2,3,1}, \
{-1,-3,-2}, \
{-1,-3,2}, \
{-1,-2,-3}, \
{-1,3,2}, \
{2,3,1}, \
{3,-2,-1}, \
{3,-2,1}, \
{3,-1,-2}, \
{-1,-2,3}, \
{3,-1,2}, \
{3,1,-2}, \
{3,1,2}, \
{3,2,-1}, \
{3,2,1}, \
{1,-3,-2}, \
{1,-3,2}, \
{-2,-2,-1}, \
{-2,-2,1}, \
{-2,-1,-2}, \
{2,1,2}, \
{1,2,-2}, \
{-1,2,2}, \
{-2,-1,2}, \
{2,1,-2}, \
{2,-1,2}, \
{-2,1,-2}, \
{2,-1,-2}, \
{2,-2,1}, \
{2,-2,-1}, \
{-2,1,2}, \
{-2,2,-1}, \
{-2,2,1}, \
{-1,-2,-2}, \
{1,-2,2}, \
{2,2,1}, \
{2,2,-1}, \
{-1,-2,2}, \
{1,-2,-2}, \
{-1,2,-2}, \
{1,2,2}, \
{-2,-1,-1}, \
{-1,2,1}, \
{-2,-1,1}, \
{1,2,1}, \
{1,1,-2}, \
{-2,1,-1}, \
{1,1,2}, \
{-2,1,1}, \
{-1,-2,-1}, \
{1,2,-1}, \
{-1,-2,1}, \
{-1,-1,-2}, \
{1,-2,-1}, \
{2,-1,-1}, \
{1,-2,1}, \
{-1,-1,2}, \
{2,-1,1}, \
{2,1,-1}, \
{2,1,1}, \
{1,-1,-2}, \
{1,-1,2}, \
{-1,1,-2}, \
{-1,1,2}, \
{-1,2,-1}, \
{-2,-1,0}, \
{-2,0,-1}, \
{-2,0,1}, \
{-2,1,0}, \
{-1,-2,0}, \
{1,0,-2}, \
{-1,0,2}, \
{2,-1,0}, \
{-1,0,-2}, \
{1,0,2}, \
{0,-2,1}, \
{1,-2,0}, \
{2,0,-1}, \
{2,0,1}, \
{0,-1,2}, \
{0,2,1}, \
{0,1,-2}, \
{0,-2,-1}, \
{0,1,2}, \
{2,1,0}, \
{-1,2,0}, \
{0,2,-1}, \
{1,2,0}, \
{0,-1,-2}, \
{-1,-1,-1}, \
{-1,-1,1}, \
{1,1,1}, \
{1,1,-1}, \
{-1,1,1}, \
{-1,1,-1}, \
{1,-1,-1}, \
{1,-1,1}, \
{-1,0,-1}, \
{-1,0,1}, \
{-1,-1,0}, \
{0,1,-1}, \
{0,-1,-1}, \
{1,1,0}, \
{-1,1,0}, \
{0,1,1}, \
{1,0,1}, \
{1,-1,0}, \
{0,-1,1}, \
{1,0,-1}, \
{-1,0,0}, \
{1,0,0}, \
{0,1,0}, \
{0,-1,0}, \
{0,0,-1}, \
{0,0,1}, \
    });




const std::vector<Dir> NEIGHBORS_3D_K5({
{-6,-5,-3}, \
{-5,-6,-3}, \
{-5,6,3}, \
{-6,-5,3}, \
{5,3,6}, \
{-5,-6,3}, \
{5,6,-3}, \
{5,6,3}, \
{-6,-3,-5}, \
{-5,-3,-6}, \
{3,-6,5}, \
{3,-5,-6}, \
{-3,6,-5}, \
{-3,5,6}, \
{-5,-3,6}, \
{-6,-3,5}, \
{-3,5,-6}, \
{6,5,-3}, \
{3,-6,-5}, \
{6,-5,-3}, \
{3,5,6}, \
{3,6,-5}, \
{6,-5,3}, \
{6,5,3}, \
{6,3,5}, \
{6,-3,5}, \
{3,6,5}, \
{3,5,-6}, \
{3,-5,6}, \
{5,-6,-3}, \
{-3,6,5}, \
{6,3,-5}, \
{-6,3,-5}, \
{-3,-5,6}, \
{-5,3,-6}, \
{5,-6,3}, \
{5,-3,-6}, \
{5,-3,6}, \
{6,-3,-5}, \
{-6,3,5}, \
{-5,3,6}, \
{-3,-5,-6}, \
{-3,-6,5}, \
{-5,6,-3}, \
{-6,5,-3}, \
{-3,-6,-5}, \
{5,3,-6}, \
{-6,5,3}, \
{-6,-5,-2}, \
{-6,-5,2}, \
{6,5,2}, \
{-5,-2,6}, \
{2,-6,-5}, \
{2,-5,-6}, \
{2,-5,6}, \
{-5,2,-6}, \
{-2,-6,-5}, \
{2,5,-6}, \
{2,5,6}, \
{2,6,-5}, \
{-6,-2,-5}, \
{-5,2,6}, \
{6,2,5}, \
{-6,-2,5}, \
{2,6,5}, \
{-2,6,5}, \
{-2,-6,5}, \
{6,2,-5}, \
{2,-6,5}, \
{-2,-5,-6}, \
{-2,6,-5}, \
{-5,6,-2}, \
{-6,2,-5}, \
{-5,6,2}, \
{-6,2,5}, \
{-2,5,6}, \
{-2,5,-6}, \
{6,-5,-2}, \
{6,-2,5}, \
{6,-2,-5}, \
{6,-5,2}, \
{-5,-2,-6}, \
{5,6,2}, \
{5,6,-2}, \
{5,2,6}, \
{-6,5,-2}, \
{-6,5,2}, \
{-5,-6,-2}, \
{-5,-6,2}, \
{5,2,-6}, \
{-2,-5,6}, \
{5,-2,6}, \
{6,5,-2}, \
{5,-2,-6}, \
{5,-6,2}, \
{5,-6,-2}, \
{-6,-4,-3}, \
{-3,-6,4}, \
{4,6,-3}, \
{-6,-4,3}, \
{-6,-3,-4}, \
{-4,-6,3}, \
{-3,4,6}, \
{-3,-6,-4}, \
{-6,-3,4}, \
{-3,4,-6}, \
{3,-6,-4}, \
{6,-4,3}, \
{-4,-3,-6}, \
{4,3,6}, \
{3,-4,-6}, \
{3,4,6}, \
{6,-3,-4}, \
{-3,-4,-6}, \
{-6,3,-4}, \
{-4,6,3}, \
{4,3,-6}, \
{4,-6,3}, \
{3,6,-4}, \
{-6,3,4}, \
{-6,4,-3}, \
{4,-6,-3}, \
{6,-3,4}, \
{-6,4,3}, \
{3,-6,4}, \
{3,4,-6}, \
{-4,-3,6}, \
{6,4,3}, \
{4,-3,6}, \
{-4,3,6}, \
{-4,6,-3}, \
{6,4,-3}, \
{6,3,4}, \
{6,-4,-3}, \
{-4,3,-6}, \
{6,3,-4}, \
{3,-4,6}, \
{3,6,4}, \
{4,-3,-6}, \
{4,6,3}, \
{-4,-6,-3}, \
{-3,-4,6}, \
{-3,6,-4}, \
{-3,6,4}, \
{-6,-1,-4}, \
{-6,-1,4}, \
{-6,1,-4}, \
{-6,1,4}, \
{1,-6,-4}, \
{-6,4,-1}, \
{-6,4,1}, \
{6,4,1}, \
{6,4,-1}, \
{6,1,4}, \
{6,1,-4}, \
{-4,-6,-1}, \
{-4,-1,6}, \
{4,-1,-6}, \
{-4,-1,-6}, \
{6,-4,-1}, \
{6,-4,1}, \
{4,-1,6}, \
{4,6,1}, \
{4,6,-1}, \
{1,-4,6}, \
{4,1,6}, \
{-1,6,4}, \
{-1,6,-4}, \
{-4,-6,1}, \
{-1,4,-6}, \
{1,4,-6}, \
{1,4,6}, \
{1,6,-4}, \
{1,6,4}, \
{-4,1,6}, \
{4,1,-6}, \
{4,-6,1}, \
{4,-6,-1}, \
{-1,4,6}, \
{1,-6,4}, \
{-4,6,-1}, \
{1,-4,-6}, \
{-4,6,1}, \
{-4,1,-6}, \
{-1,-4,6}, \
{-1,-4,-6}, \
{-1,-6,4}, \
{-1,-6,-4}, \
{6,-1,-4}, \
{6,-1,4}, \
{-6,-4,-1}, \
{-6,-4,1}, \
{6,3,2}, \
{-6,2,3}, \
{3,6,-2}, \
{-6,3,2}, \
{6,-2,-3}, \
{6,-2,3}, \
{3,2,-6}, \
{2,-3,-6}, \
{2,-3,6}, \
{2,3,-6}, \
{3,-2,-6}, \
{3,-6,2}, \
{2,3,6}, \
{6,-3,2}, \
{-2,-3,-6}, \
{6,-3,-2}, \
{-6,3,-2}, \
{-6,2,-3}, \
{-2,3,6}, \
{2,6,-3}, \
{2,6,3}, \
{2,-6,-3}, \
{2,-6,3}, \
{-3,2,6}, \
{-2,-6,3}, \
{-6,-2,3}, \
{-2,6,-3}, \
{-3,-2,-6}, \
{-6,-2,-3}, \
{6,2,-3}, \
{-6,-3,2}, \
{-2,6,3}, \
{-3,-6,-2}, \
{-2,-6,-3}, \
{3,6,2}, \
{3,2,6}, \
{-2,3,-6}, \
{-3,-6,2}, \
{3,-6,-2}, \
{6,2,3}, \
{-6,-3,-2}, \
{-3,-2,6}, \
{-3,6,-2}, \
{3,-2,6}, \
{-2,-3,6}, \
{-3,6,2}, \
{6,3,-2}, \
{-3,2,-6}, \
{-6,-3,-1}, \
{-6,-3,1}, \
{-6,-1,-3}, \
{-6,-1,3}, \
{-6,1,-3}, \
{-6,1,3}, \
{-6,3,-1}, \
{-6,3,1}, \
{6,-1,3}, \
{6,1,-3}, \
{6,1,3}, \
{6,3,-1}, \
{6,3,1}, \
{-3,-6,-1}, \
{-3,-6,1}, \
{1,-3,-6}, \
{-3,-1,-6}, \
{-3,-1,6}, \
{3,6,1}, \
{-3,1,-6}, \
{3,6,-1}, \
{-3,1,6}, \
{-3,6,-1}, \
{-3,6,1}, \
{-1,-6,3}, \
{1,-6,3}, \
{3,1,6}, \
{3,1,-6}, \
{3,-1,6}, \
{3,-1,-6}, \
{1,-6,-3}, \
{3,-6,1}, \
{3,-6,-1}, \
{-1,-6,-3}, \
{-1,-3,-6}, \
{-1,-3,6}, \
{1,6,3}, \
{1,6,-3}, \
{-1,3,-6}, \
{1,3,6}, \
{-1,3,6}, \
{1,3,-6}, \
{-1,6,-3}, \
{-1,6,3}, \
{1,-3,6}, \
{6,-3,-1}, \
{6,-3,1}, \
{6,-1,-3}, \
{-5,-4,-2}, \
{-5,-4,2}, \
{-5,-2,-4}, \
{-5,-2,4}, \
{2,-4,-5}, \
{-5,2,-4}, \
{-2,-4,5}, \
{2,-5,4}, \
{4,-2,-5}, \
{4,2,5}, \
{2,-5,-4}, \
{-5,2,4}, \
{-5,4,-2}, \
{-5,4,2}, \
{4,-5,-2}, \
{4,-5,2}, \
{-2,-4,-5}, \
{-2,-5,4}, \
{-4,-5,-2}, \
{-4,-5,2}, \
{-4,-2,-5}, \
{-4,-2,5}, \
{-4,2,-5}, \
{-2,-5,-4}, \
{-2,4,-5}, \
{2,5,-4}, \
{4,-2,5}, \
{2,4,5}, \
{-4,2,5}, \
{-2,4,5}, \
{2,5,4}, \
{-2,5,-4}, \
{5,4,2}, \
{2,4,-5}, \
{-2,5,4}, \
{5,4,-2}, \
{4,2,-5}, \
{-4,5,-2}, \
{5,2,4}, \
{5,2,-4}, \
{-4,5,2}, \
{5,-2,4}, \
{5,-2,-4}, \
{5,-4,2}, \
{5,-4,-2}, \
{4,5,2}, \
{4,5,-2}, \
{2,-4,5}, \
{-5,-3,-2}, \
{5,-2,3}, \
{5,2,-3}, \
{-2,-3,-5}, \
{3,-5,2}, \
{2,5,3}, \
{5,2,3}, \
{3,-5,-2}, \
{5,3,-2}, \
{-2,3,-5}, \
{3,2,-5}, \
{2,5,-3}, \
{5,3,2}, \
{-2,-5,3}, \
{3,-2,-5}, \
{-2,-5,-3}, \
{3,2,5}, \
{-2,3,5}, \
{-3,2,-5}, \
{2,3,5}, \
{-3,5,2}, \
{-5,3,2}, \
{-2,5,-3}, \
{-2,5,3}, \
{-5,3,-2}, \
{-3,5,-2}, \
{2,3,-5}, \
{-5,2,3}, \
{-5,2,-3}, \
{2,-3,5}, \
{-5,-2,3}, \
{3,5,2}, \
{-5,-2,-3}, \
{2,-3,-5}, \
{-5,-3,2}, \
{2,-5,-3}, \
{-3,-2,5}, \
{2,-5,3}, \
{3,-2,5}, \
{-3,-2,-5}, \
{-3,2,5}, \
{-2,-3,5}, \
{-3,-5,2}, \
{5,-3,-2}, \
{5,-3,2}, \
{-3,-5,-2}, \
{5,-2,-3}, \
{3,5,-2}, \
{-5,-3,-1}, \
{-5,-3,1}, \
{3,-5,1}, \
{3,-5,-1}, \
{1,-3,5}, \
{5,-3,-1}, \
{3,1,5}, \
{-5,-1,-3}, \
{5,-1,-3}, \
{5,-1,3}, \
{5,1,-3}, \
{5,1,3}, \
{5,-3,1}, \
{-3,5,-1}, \
{1,3,5}, \
{3,-1,-5}, \
{5,3,-1}, \
{5,3,1}, \
{3,1,-5}, \
{1,-5,-3}, \
{-1,-5,-3}, \
{-1,-5,3}, \
{-1,3,-5}, \
{-3,-1,-5}, \
{1,5,-3}, \
{1,5,3}, \
{1,-3,-5}, \
{-1,-3,-5}, \
{-1,5,-3}, \
{3,-1,5}, \
{1,-5,3}, \
{-3,-1,5}, \
{3,5,-1}, \
{-3,5,1}, \
{-1,3,5}, \
{-1,-3,5}, \
{-1,5,3}, \
{-5,3,1}, \
{-5,3,-1}, \
{-3,1,-5}, \
{1,3,-5}, \
{-3,1,5}, \
{-3,-5,-1}, \
{-5,1,3}, \
{-5,1,-3}, \
{3,5,1}, \
{-5,-1,3}, \
{-3,-5,1}, \
{-2,-4,3}, \
{-2,3,-4}, \
{-2,-4,-3}, \
{-2,-3,4}, \
{-2,3,4}, \
{-2,4,-3}, \
{4,-3,-2}, \
{-3,4,2}, \
{3,2,-4}, \
{4,-2,-3}, \
{-3,4,-2}, \
{3,4,-2}, \
{-3,-4,2}, \
{-2,4,3}, \
{3,-2,4}, \
{-3,-4,-2}, \
{-3,2,4}, \
{3,4,2}, \
{2,4,3}, \
{2,4,-3}, \
{2,3,4}, \
{2,3,-4}, \
{2,-3,4}, \
{2,-3,-4}, \
{4,3,-2}, \
{4,3,2}, \
{3,2,4}, \
{-4,-2,3}, \
{-4,-2,-3}, \
{-4,-3,2}, \
{-4,-3,-2}, \
{-2,-3,-4}, \
{-3,-2,-4}, \
{3,-4,2}, \
{3,-2,-4}, \
{-4,3,-2}, \
{-4,2,3}, \
{4,-3,2}, \
{4,-2,3}, \
{-3,2,-4}, \
{4,2,-3}, \
{3,-4,-2}, \
{-4,2,-3}, \
{-3,-2,4}, \
{-4,3,2}, \
{2,-4,-3}, \
{4,2,3}, \
{2,-4,3}, \
{-3,-4,-1}, \
{-1,3,4}, \
{4,-3,-1}, \
{-1,4,3}, \
{1,3,-4}, \
{-4,1,3}, \
{4,-1,-3}, \
{-4,1,-3}, \
{-4,-1,3}, \
{4,-1,3}, \
{-4,-1,-3}, \
{4,-3,1}, \
{1,3,4}, \
{3,1,4}, \
{3,1,-4}, \
{-3,4,1}, \
{-3,-1,-4}, \
{-3,4,-1}, \
{-4,-3,1}, \
{-4,-3,-1}, \
{-1,-4,-3}, \
{3,-1,4}, \
{4,1,-3}, \
{3,4,-1}, \
{3,4,1}, \
{-3,-1,4}, \
{-1,-4,3}, \
{3,-4,1}, \
{-1,-3,-4}, \
{4,1,3}, \
{-1,-3,4}, \
{1,4,-3}, \
{-3,1,4}, \
{1,-3,4}, \
{4,3,-1}, \
{4,3,1}, \
{3,-4,-1}, \
{1,-3,-4}, \
{-3,1,-4}, \
{-4,3,1}, \
{-1,3,-4}, \
{1,-4,-3}, \
{-4,3,-1}, \
{-3,-4,1}, \
{-1,4,-3}, \
{3,-1,-4}, \
{1,-4,3}, \
{1,4,3}, \
{-2,3,3}, \
{-3,3,-2}, \
{2,-3,-3}, \
{2,-3,3}, \
{2,3,-3}, \
{2,3,3}, \
{3,-3,-2}, \
{3,-2,-3}, \
{3,-2,3}, \
{3,2,-3}, \
{3,2,3}, \
{3,3,-2}, \
{3,3,2}, \
{-3,-3,-2}, \
{3,-3,2}, \
{-3,-3,2}, \
{-3,-2,-3}, \
{-3,-2,3}, \
{-3,2,-3}, \
{-3,2,3}, \
{-3,3,2}, \
{-2,-3,-3}, \
{-2,-3,3}, \
{-2,3,-3}, \
{-2,-4,-1}, \
{2,1,4}, \
{-4,-1,-2}, \
{-2,1,-4}, \
{1,-4,-2}, \
{4,2,1}, \
{-2,4,-1}, \
{4,2,-1}, \
{-2,4,1}, \
{-1,-4,-2}, \
{-1,-4,2}, \
{4,1,-2}, \
{2,-4,-1}, \
{1,4,-2}, \
{-4,-2,-1}, \
{1,4,2}, \
{-4,-2,1}, \
{-4,-1,2}, \
{-1,-2,-4}, \
{2,-1,-4}, \
{-2,-1,4}, \
{2,-4,1}, \
{4,-1,2}, \
{4,-1,-2}, \
{-1,2,4}, \
{4,1,2}, \
{-4,1,-2}, \
{2,-1,4}, \
{-4,2,-1}, \
{-4,1,2}, \
{-1,2,-4}, \
{-1,-2,4}, \
{2,1,-4}, \
{-4,2,1}, \
{4,-2,1}, \
{4,-2,-1}, \
{2,4,-1}, \
{-2,-1,-4}, \
{2,4,1}, \
{-1,4,2}, \
{-2,-4,1}, \
{-1,4,-2}, \
{1,2,4}, \
{-2,1,4}, \
{1,2,-4}, \
{1,-2,4}, \
{1,-2,-4}, \
{1,-4,2}, \
{-3,-3,-1}, \
{-3,-1,-3}, \
{-3,-1,3}, \
{-3,1,-3}, \
{3,1,3}, \
{3,3,1}, \
{-1,3,-3}, \
{-3,1,3}, \
{-1,-3,3}, \
{1,3,3}, \
{-1,-3,-3}, \
{1,3,-3}, \
{-1,3,3}, \
{3,-3,1}, \
{3,-3,-1}, \
{3,-1,-3}, \
{-3,3,-1}, \
{3,3,-1}, \
{3,-1,3}, \
{-3,-3,1}, \
{-3,3,1}, \
{1,-3,3}, \
{1,-3,-3}, \
{3,1,-3}, \
{-2,-2,-3}, \
{-3,-2,2}, \
{2,-3,2}, \
{3,2,-2}, \
{3,-2,-2}, \
{2,-3,-2}, \
{2,2,-3}, \
{-2,-3,2}, \
{2,3,2}, \
{-3,-2,-2}, \
{-2,3,-2}, \
{3,2,2}, \
{-2,3,2}, \
{-2,-3,-2}, \
{-2,2,-3}, \
{-2,-2,3}, \
{-3,2,2}, \
{2,-2,3}, \
{2,2,3}, \
{-3,2,-2}, \
{2,-2,-3}, \
{3,-2,2}, \
{2,3,-2}, \
{-2,2,3}, \
{-2,1,-3}, \
{3,-2,-1}, \
{3,-2,1}, \
{-2,-1,3}, \
{3,-1,-2}, \
{2,-3,-1}, \
{3,-1,2}, \
{2,-3,1}, \
{-2,-1,-3}, \
{3,1,-2}, \
{3,1,2}, \
{-2,-3,1}, \
{3,2,-1}, \
{3,2,1}, \
{2,-1,-3}, \
{-2,-3,-1}, \
{2,1,3}, \
{-3,2,1}, \
{-3,2,-1}, \
{-3,1,2}, \
{2,-1,3}, \
{-3,1,-2}, \
{-3,-1,2}, \
{-3,-1,-2}, \
{-3,-2,1}, \
{2,1,-3}, \
{-3,-2,-1}, \
{1,-3,-2}, \
{1,-3,2}, \
{1,-2,-3}, \
{1,-2,3}, \
{1,2,-3}, \
{1,2,3}, \
{1,3,-2}, \
{1,3,2}, \
{-1,3,2}, \
{-1,3,-2}, \
{2,3,-1}, \
{-1,2,3}, \
{2,3,1}, \
{-1,2,-3}, \
{-1,-2,3}, \
{-1,-2,-3}, \
{-1,-3,2}, \
{-1,-3,-2}, \
{-2,3,1}, \
{-2,3,-1}, \
{-2,1,3}, \
{-3,2,0}, \
{3,2,0}, \
{2,3,0}, \
{-3,-2,0}, \
{0,3,2}, \
{2,-3,0}, \
{3,-2,0}, \
{0,-2,-3}, \
{0,-3,2}, \
{-3,0,2}, \
{-2,0,3}, \
{0,-3,-2}, \
{0,-2,3}, \
{-3,0,-2}, \
{0,3,-2}, \
{0,2,-3}, \
{-2,0,-3}, \
{2,0,-3}, \
{0,2,3}, \
{3,0,-2}, \
{2,0,3}, \
{3,0,2}, \
{-2,-3,0}, \
{-2,3,0}, \
{1,1,-3}, \
{1,-1,-3}, \
{-1,-1,3}, \
{-1,1,-3}, \
{-1,1,3}, \
{-1,-3,1}, \
{1,-1,3}, \
{-1,3,-1}, \
{-1,3,1}, \
{-1,-3,-1}, \
{3,1,1}, \
{1,3,1}, \
{-3,-1,1}, \
{-3,-1,-1}, \
{1,3,-1}, \
{3,1,-1}, \
{1,-3,-1}, \
{-3,1,-1}, \
{3,-1,1}, \
{3,-1,-1}, \
{-1,-1,-3}, \
{-3,1,1}, \
{1,-3,1}, \
{1,1,3}, \
{0,-1,-3}, \
{-1,0,-3}, \
{3,1,0}, \
{3,-1,0}, \
{1,3,0}, \
{0,-3,1}, \
{-1,3,0}, \
{1,-3,0}, \
{0,-3,-1}, \
{-3,1,0}, \
{0,1,3}, \
{-3,0,-1}, \
{-1,0,3}, \
{0,3,-1}, \
{3,0,1}, \
{0,1,-3}, \
{3,0,-1}, \
{0,-1,3}, \
{-3,0,1}, \
{-3,-1,0}, \
{1,0,3}, \
{1,0,-3}, \
{-1,-3,0}, \
{0,3,1}, \
{-1,2,-2}, \
{-1,-2,2}, \
{2,-2,-1}, \
{2,-2,1}, \
{2,-1,-2}, \
{2,-1,2}, \
{2,1,2}, \
{2,2,-1}, \
{2,2,1}, \
{-2,-1,-2}, \
{-2,-1,2}, \
{-2,1,-2}, \
{-2,-2,-1}, \
{2,1,-2}, \
{-2,-2,1}, \
{1,-2,-2}, \
{1,-2,2}, \
{-2,1,2}, \
{1,2,-2}, \
{-2,2,-1}, \
{-2,2,1}, \
{-1,-2,-2}, \
{1,2,2}, \
{-1,2,2}, \
{1,-1,2}, \
{-2,-1,-1}, \
{1,2,1}, \
{-2,1,1}, \
{-1,-2,1}, \
{1,-1,-2}, \
{-1,-1,2}, \
{-1,1,-2}, \
{-1,1,2}, \
{-1,-2,-1}, \
{2,-1,-1}, \
{1,-2,1}, \
{-2,-1,1}, \
{-1,2,-1}, \
{-1,2,1}, \
{1,-2,-1}, \
{-1,-1,-2}, \
{1,2,-1}, \
{-2,1,-1}, \
{1,1,-2}, \
{2,-1,1}, \
{2,1,-1}, \
{2,1,1}, \
{1,1,2}, \
{-1,0,-2}, \
{0,-2,-1}, \
{1,0,-2}, \
{1,0,2}, \
{0,-1,2}, \
{0,1,2}, \
{0,-2,1}, \
{-2,0,-1}, \
{1,2,0}, \
{-2,-1,0}, \
{-2,1,0}, \
{0,-1,-2}, \
{0,2,-1}, \
{-2,0,1}, \
{0,2,1}, \
{2,0,1}, \
{2,0,-1}, \
{2,-1,0}, \
{1,-2,0}, \
{-1,2,0}, \
{-1,-2,0}, \
{2,1,0}, \
{0,1,-2}, \
{-1,0,2}, \
{1,1,-1}, \
{-1,-1,-1}, \
{-1,-1,1}, \
{1,-1,1}, \
{1,1,1}, \
{-1,1,-1}, \
{1,-1,-1}, \
{-1,1,1}, \
{0,-1,-1}, \
{-1,0,-1}, \
{1,0,1}, \
{0,1,-1}, \
{0,1,1}, \
{1,0,-1}, \
{0,-1,1}, \
{-1,-1,0}, \
{-1,1,0}, \
{1,-1,0}, \
{-1,0,1}, \
{1,1,0}, \
{0,1,0}, \
{1,0,0}, \
{0,0,-1}, \
{0,0,1}, \
{0,-1,0}, \
{-1,0,0}, \
    });