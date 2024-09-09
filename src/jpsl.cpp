#include <chrono>
#include <fstream>
#include <cmath>
#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"
#include "jpsl/3k_dir.h"

using namespace std;
using namespace JPSL;



PathDescriptor JPSL::plan(Point start,
    Point goal,
    int max_jump,
    const function<bool(const Point&)>& state_valid,
    float timeLimit,
    std::function<float(const Point&, const Point&)> HeuristicFunction,
    std::string expandion_policy,
    bool debug) {
    // solve a planning problem
    //  INPUTS
    //  ======
    //
    //   - start : JPSL::Point  
    //       start of trajectory
    //   - goal : JPSL::Point
    //       target of trajectory
    //   - max_jump : int    
    //       maximal jump length for bounded JPS, set to -1 for unlimited length (JPS) or to 1 for no jumps (A*)
    //   - state_valid : const JPSL::Point & -> bool
    //       function that returns true for states in free space
    //
    //  OUTPUTS
    //  =======
    //
    //   - ret : pair<vector<JPSL::Point>, float>  
    //       ret.first contains trajectory if found, ret.second its length
    //

    // map that remembers path
    unordered_map<Point, Point> parents;

    // priority queue with euclidean distance to target
    auto astar_cmp = [goal, HeuristicFunction](const ASNode& n1, const ASNode& n2) {
        float d_start_n1 = get<2>(n1);
        float d_start_n2 = get<2>(n2);
        Point n1_point = get<0>(n1);
        Point n2_point = get<0>(n2);
        float d_n1_goal = HeuristicFunction(n1_point, goal);
        float d_n2_goal = HeuristicFunction(n2_point, goal);
        return d_start_n1 + d_n1_goal > d_start_n2 + d_n2_goal;
    };
    priority_queue<ASNode, vector<ASNode>, decltype(astar_cmp)> astar(astar_cmp);

    // initialize with all valid neighbors around start
    astar.push(ASNode{ start, start, 0 });
    auto startTime = std::chrono::steady_clock::now();
    float elapsedTime = 0;
    float pathLenght = goal.distance(start);
    int visited = 0;


    bool found = false;
    while (!found && !astar.empty())
    {
        ASNode current = astar.top();
        auto currentTime = std::chrono::steady_clock::now();
        elapsedTime = std::chrono::duration<float>(currentTime - startTime).count();

        if (elapsedTime > timeLimit)
        {
            return PathDescriptor();
        }

        Point node = get<0>(current);
        Point parent = get<1>(current);
        float running_dist = get<2>(current);
        astar.pop();

        if (node == goal) {
            found = true;
            break;
        }

        for (Point successor : JPSLSucc(node, parent, goal, max_jump, state_valid, visited, expandion_policy)) {
            if (parents.find(successor) == parents.end())
            {
                astar.push(ASNode{ successor, node, running_dist + node.distance(successor) });
                parents[successor] = node;
            }
        }
    }



    if (found) {
        vector<Point> sol_path;
        float cost = 0;
        while (goal != start) {
            sol_path.push_back(goal);
            Point new_goal = parents[goal];
            cost += (new_goal - goal).norm();
            goal = new_goal;
        }
        sol_path.push_back(start);

        reverse(sol_path.begin(), sol_path.end());
        

        PathDescriptor descriptor(cost, visited, sol_path.size(), elapsedTime, pathLenght);
        if (debug)
        {
            descriptor.CopyPath(sol_path);
        }
        return descriptor;
    }
    return PathDescriptor();
}

PathDescriptor JPSL::plan_jps(Point start, Point goal, const function<bool(const Point&)>& state_valid, float timeLimit, std::function<float(const Point&, const Point&)> HeuristicFunction, std::string expandion_policy) {
    return plan(start, goal, -1, state_valid, timeLimit, HeuristicFunction, expandion_policy);
}

PathDescriptor JPSL::plan_astar(Point start, Point goal, const function<bool(const Point&)>& state_valid, float timeLimit, std::function<float(const Point&, const Point&)> HeuristicFunction, std::string expandion_policy) {
    return plan(start, goal, 1, state_valid, timeLimit, HeuristicFunction, expandion_policy);
}

pair<uint8_t, Point> jump_rec(const Point& p, const Dir& d, const Point& goal, const function<bool(const Point&)>& state_valid, const std::string& expancion_policy, std::unordered_map<Point, std::pair<uint8_t, Point>>& jump_results)
{
    if (!state_valid(p))
    {
        std::cout << "cant jump from here, something very wrong happened in jump_rec";
        return { 0, p };  // Can't jump from here
    }

    Point par_iter = p;
    Point nod_iter = p + d;

    while (state_valid(nod_iter))
    {
        // Check if the node has already been calculated
        auto it = jump_results.find(nod_iter);
        if (it != jump_results.end())
        {
            return it->second;
        }

        if (nod_iter == goal)
        {
            return { 2, nod_iter };  // Goal node reached
        }

        if (has_forced_neighbor(nod_iter, par_iter, state_valid))
        {
            return { 1, nod_iter };  // Forced neighbor found
        }

        //not a interesting node, add to jump_results
        jump_results[nod_iter] = { 0, nod_iter };

        // recursively expand higher dimentiosn first, until ends with only 1d dimentions or hit and obstacle
        if (d.order() > 1)
        {
            //expand nodes that are at the same octant and dimention of the parent -> node direction, ordered by its "norm"
            vector<Dir> jumpdirs = JPSL::natural_neighbors(nod_iter, par_iter, state_valid, expancion_policy);

            for (auto jumpdir : jumpdirs)
            {
                if (jumpdir == d)
                {
                    continue;
                }

                pair<uint8_t, Point> ret = jump_rec(nod_iter, jumpdir, goal, state_valid, expancion_policy, jump_results);
                if (ret.first)
                {
                    return { ret.first, nod_iter };
                }
            }
        }

        par_iter = nod_iter;
        nod_iter += d;
    }

    return { 0, p };  // No valid jump found
}


std::pair<uint8_t, Point> JPSL::jump(const Point& p, const Dir& d, const Point& goal, int max_jump, const std::function<bool(const Point&)>& state_valid, const std::string& expancion_policy)
{
    std::unordered_map<Point, std::pair<uint8_t, Point>> jump_results;
    return jump_rec(p, d, goal, state_valid, expancion_policy, jump_results);
}

vector<Dir> JPSL::natural_neighbors(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid, std::string expancion_policy)
{
    Dir par_dir = node.direction_to(parent);
    vector<Dir> ret;

    if (expancion_policy == "k3-neighbors")
    {
        ret = GetExpandedSuccessorsOfSameDir(NEIGHBORS_3D_K3, -par_dir, node, state_valid);
    }

    if (expancion_policy == "k4-neighbors")
    {
        ret = GetExpandedSuccessorsOfSameDir(NEIGHBORS_3D_K4, -par_dir, node, state_valid);
    }

    if (expancion_policy == "k5-neighbors")
    {
        ret = GetExpandedSuccessorsOfSameDir(NEIGHBORS_3D_K5, -par_dir, node, state_valid);
    }

    return ret;
}

vector<Dir> JPSL::all_neighbors(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid, std::string expancion_policy)
{
    // return all nodes that require expansion when moving 
    // into unoccupied center of 3x3 box from parent

    vector<Dir> ret, nat, forc;

    nat = JPSL::natural_neighbors(node, parent, state_valid, expancion_policy);
    forc = JPSL::forced_neighbors_fast(node, parent, state_valid);

    sort(nat.begin(), nat.end());
    sort(forc.begin(), forc.end());

    set_union(make_move_iterator(nat.begin()), make_move_iterator(nat.end()),
        make_move_iterator(forc.begin()), make_move_iterator(forc.end()),
        back_inserter(ret));

    return ret;
}

vector<Dir> JPSL::forced_neighbors_fast(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid) {

    Dir d = node.direction_to(parent);
    vector<Dir> ret;

    switch (d.order()) {
    case 1:
        return forced_neighbors_fast_1d(node, parent, state_valid);   // closed-form solution
        break;
    case 2:
        ret = decode_fn<2>(node, parent, lookup2d[encode_obs<2>(node, parent, state_valid)]);
        break;
    case 3:
        ret = decode_fn<3>(node, parent, lookup3d[encode_obs<3>(node, parent, state_valid)]);
        break;
    }

    vector<Dir> ret_trans;
    copy_if(ret.begin(), ret.end(), back_inserter(ret_trans),
        [state_valid, node](const Dir& d) {
            return state_valid(node + d);
        });

    return ret_trans;
}

vector<Dir> JPSL::forced_neighbors_fast_1d(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid) {
    Dir d_parent = node.direction_to(parent);

    vector<Dir> ret;

    if (d_parent.dx() != 0)
        for (Dir d_cand : NIEGHBORS1D_DX)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                ret.emplace_back(-d_parent.dx(), d_cand.dy(), d_cand.dz());

    if (d_parent.dy() != 0)
        for (Dir d_cand : NIEGHBORS1D_DY)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                ret.emplace_back(d_cand.dx(), -d_parent.dy(), d_cand.dz());

    if (d_parent.dz() != 0)
        for (Dir d_cand : NIEGHBORS1D_DZ)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                ret.emplace_back(d_cand.dx(), d_cand.dy(), -d_parent.dz());

    return ret;
}

bool JPSL::has_forced_neighbor(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid) {

    vector<Dir> ret;

    switch (node.direction_to(parent).order()) {
    case 1:
        return has_forced_neighbor_fast_1d(node, parent, state_valid);   // closed-form solution
        break;
    case 2:
        ret = decode_fn<2>(node, parent, lookup2d[encode_obs<2>(node, parent, state_valid)]);
        for (Dir d : ret)
            if (state_valid(node + d))
                return true;
        break;
    case 3:
        ret = decode_fn<3>(node, parent, lookup3d[encode_obs<3>(node, parent, state_valid)]);
        for (Dir d : ret)
            if (state_valid(node + d))
                return true;
        break;
    }
    return false;
}

bool JPSL::has_forced_neighbor_fast_1d(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid) {
    Dir d_parent = node.direction_to(parent);

    vector<Dir> ret;

    if (d_parent.dx() != 0)
        for (Dir d_cand : NIEGHBORS1D_DX)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                return true;

    if (d_parent.dy() != 0)
        for (Dir d_cand : NIEGHBORS1D_DY)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                return true;

    if (d_parent.dz() != 0)
        for (Dir d_cand : NIEGHBORS1D_DZ)
            if (!state_valid(node + d_cand) && state_valid(node + d_cand - d_parent))
                return true;

    return false;
}

vector<Dir> JPSL::forced_neighbors_slow(const Point& node, const Point& parent, const function<bool(const Point&)>& state_valid, std::string expancion_policy) {
    Dir d_parent = node.direction_to(parent);
    vector<Dir> ret;

    // define valid nodes in the box
    unordered_map<Dir, float> dist;
    vector<Dir> remaining;
    for (Dir d : JPSL::NEIGHBORS_3D) {
        if (state_valid(node + d)) {
            dist[d] = 10;
            remaining.push_back(d);
        }
    }
    dist[d_parent] = 0;

    // do Dijkstra
    while (!remaining.empty()) {
        sort(remaining.begin(), remaining.end(), [dist](const Dir& i1, const Dir& i2) { return dist.at(i1) > dist.at(i2); });
        Dir u(remaining.back());
        remaining.pop_back();
        for (int i = max(u.dx() - 1, -1); i < min(2, u.dx() + 2); ++i)
            for (int j = max(u.dy() - 1, -1); j < min(2, u.dy() + 2); ++j)
                for (int k = max(u.dz() - 1, -1); k < min(2, u.dz() + 2); ++k)
                    if (find(remaining.begin(), remaining.end(), Dir(i, j, k)) != remaining.end())
                        dist[Dir(i, j, k)] = min(dist[Dir(i, j, k)], dist[u] + Dir(i, j, k).distance_to(u));
    }

    // forced neighbors are those whose shortest distance from parent is through (0,0,0)
    for (const pair<Dir, float>& neigh_dist_pair : dist) {
        const Dir& neigh = neigh_dist_pair.first;
        float d = neigh_dist_pair.second;
        if (d_parent.norm() + neigh.norm() < d) {
            ret.push_back(Dir(neigh));
        }
    }

    // remove natural neighbors
    vector<Dir> nat = JPSL::natural_neighbors(node, parent, state_valid, expancion_policy);
    vector<Dir> ret_clean;
    copy_if(ret.begin(), ret.end(), back_inserter(ret_clean), [&nat](const Dir& d) { return find(nat.begin(), nat.end(), d) == nat.end(); });

    return ret_clean;
}


JPSL::JPSLSucc::JPSLSucc_iter::JPSLSucc_iter(const Point& node,
    const Point& goal,
    int max_jump,
    const function<bool(const Point&)>& state_valid,
    const vector<Dir>& jump_dirs,
    const std::string& expancion_policy)
    : node(node),
    goal(goal),
    max_jump(max_jump),
    state_valid(state_valid),
    jump_dirs(jump_dirs),
    res(0, Point(0, 0, 0)),
    it(this->jump_dirs.begin()),
    expancion_policy(expancion_policy) {

    // Early exit if jump_dirs is empty
    if (jump_dirs.empty()) {
        return;
    }

    // Initialize res with the first valid jump, if any
    res = jump(node, *it, goal, max_jump, state_valid, expancion_policy);

    // Advance the iterator until a valid jump result is found
    while (!res.first && ++it != jump_dirs.end()) {
        res = jump(node, *it, goal, max_jump, state_valid, expancion_policy);
    }

    // If the goal is in sight, move the iterator to the end
    if (res.first == 2) {
        it = jump_dirs.end() - 1;
    }
}

JPSL::JPSLSucc::JPSLSucc_iter::JPSLSucc_iter(const Point& node, const Point& goal,
    int max_jump,
    const function<bool(const Point&)>& state_valid,
    const vector<Dir>& jump_dirs,
    vector<Dir>::iterator it,
    const std::string& expancion_policy)
    : node(node),
    goal(goal),
    max_jump(max_jump),
    state_valid(state_valid),
    jump_dirs(jump_dirs),
    res(make_pair(0, Point(0, 0, 0))),
    it(it) {
    this->expancion_policy = expancion_policy;
}

JPSLSucc::JPSLSucc_iter& JPSL::JPSLSucc::JPSLSucc_iter::operator++() {
    ++it;
    res.first = false;

    // Advance the iterator and find the next valid jump direction
    while (it != jump_dirs.end()) {
        res = jump(node, *it, goal, max_jump, state_valid, this->expancion_policy);
        if (res.first) {
            // If goal is found, fast forward to the end
            if (res.first == 2) {
                it = jump_dirs.end() - 1;
            }
            break;
        }
        ++it;
    }
    return *this;
}

Point JPSL::JPSLSucc::JPSLSucc_iter::operator*() {
    return res.second;
}

bool JPSL::JPSLSucc::JPSLSucc_iter::operator!=(const JPSLSucc_iter& other) {
    return it != other.it;
}

JPSL::JPSLSucc::JPSLSucc(const Point& node, const Point& parent, const Point& goal, int max_jump, const function<bool(const Point&)>& state_valid, int& visited, const std::string& expancion_policy)
    : node(node),
    goal(goal),
    max_jump(max_jump),
    state_valid(state_valid) {
    this->expancion_policy = expancion_policy;

    // Determine the jump directions based on whether the node equals the parent
    if (node == parent)
    {
        jump_dirs = JPSL::NEIGHBORS_3D;
    }
    else
    {
        //the first expancion never uses the expanded neighborhood to avoid jumping the goal.
        jump_dirs = all_neighbors(node, parent, state_valid, expancion_policy);
    }

    visited += jump_dirs.size();

    // Efficiently sort directions towards the goal
    if (!jump_dirs.empty()) {
        sort(jump_dirs.begin(), jump_dirs.end(), [node, goal](const Dir& d1, const Dir& d2) {
            return ((node + d1) - goal).norm() < ((node + d2) - goal).norm();
            });
    }
}


JPSLSucc::JPSLSucc_iter JPSL::JPSLSucc::begin() {
    return JPSLSucc_iter(node, goal, max_jump, state_valid, jump_dirs, this->expancion_policy);
}

JPSLSucc::JPSLSucc_iter JPSL::JPSLSucc::end() {
    return JPSLSucc_iter(node, goal, max_jump, state_valid, jump_dirs, jump_dirs.end(), this->expancion_policy);
}