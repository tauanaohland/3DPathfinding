#ifndef JPSL_HEADER 
#define JPSL_HEADER

#include <math.h>

#include <vector>
#include <queue>
#include <functional>
#include <map>
#include <unordered_map>
#include <stdint.h>
#include <iostream>
#include <tuple>

#include "jpsl/dir.hpp"
#include "jpsl/point.hpp"
#include "jpsl/encodings.hpp"
#include "../Descriptors.h"

namespace JPSL {

    typedef std::tuple<JPSL::Point, JPSL::Point, float> ASNode;   // node, parent, running distance

    // core JPS algos
    PathDescriptor plan(Point, Point, int, const std::function<bool(const Point&)>&, float timeLimit, std::function<float(const Point&, const Point&)> HeuristicFunction, std::string expandion_policy, bool debug = false);
    PathDescriptor plan_jps(Point, Point, const std::function<bool(const Point&)>&, float timeLimit, std::function<float(const Point&, const Point&)> HeuristicFunction, std::string expandion_policy);
    PathDescriptor plan_astar(Point, Point, const std::function<bool(const Point&)>&, float timeLimit, std::function<float(const Point&, const Point&)> HeuristicFunction, std::string expandion_policy);
    std::pair<uint8_t, Point> jump(const Point&, const Dir&, const Point&, int, const std::function<bool(const Point&)>&, const std::string& expancion_policy);

    // neighbor algos
    std::vector<Dir> natural_neighbors(const Point&, const Point&, const std::function<bool(const Point&)>&, std::string expancion_policy);
    std::vector<Dir> forced_neighbors_fast(const Point&, const Point&, const std::function<bool(const Point&)>&);
    std::vector<Dir> forced_neighbors_slow(const Point&, const Point&, const std::function<bool(const Point&)>&, std::string expancion_policy);
    std::vector<Dir> all_neighbors(const Point&, const Point&, const std::function<bool(const Point&)>&, std::string expancion_policy);

    std::vector<Dir> forced_neighbors_fast_1d(const Point&, const Point&, const std::function<bool(const Point&)>&);

    bool has_forced_neighbor(const Point&, const Point&, const std::function<bool(const Point&)>&);
    bool has_forced_neighbor_fast_1d(const Point&, const Point&, const std::function<bool(const Point&)>&);

    // iterator class for JPS successors
    class JPSLSucc {
    public:
        std::string expancion_policy;

        class JPSLSucc_iter {
        public:
            JPSLSucc_iter(const Point&, const Point&, int, const std::function<bool(const Point&)>&, const std::vector<Dir>&, const std::string& expancion_policy);
            JPSLSucc_iter(const Point&, const Point&, int, const std::function<bool(const Point&)>&, const std::vector<Dir>&, std::vector<Dir>::iterator, const std::string& expancion_policy);
            typedef Dir value_type;
            JPSLSucc_iter& operator++();
            Point operator*();
            bool operator!=(const JPSLSucc_iter&);
            std::string expancion_policy;

            friend class JPSLSucc;
        private:
            const Point& node;
            const Point& goal;
            int max_jump;
            const std::function<bool(const Point&)>& state_valid;
            const std::vector<Dir>& jump_dirs;
            std::pair<uint8_t, Point> res;
            std::vector<Dir>::const_iterator it;
        };

        JPSLSucc(const Point&, const Point&, const Point&, int, const std::function<bool(const Point&)>&, int& a, const std::string& expancion_policy);
        JPSLSucc_iter begin();
        JPSLSucc_iter end();

    private:
        const Point& node;
        const Point& goal;
        int max_jump;
        const std::function<bool(const Point&)>& state_valid;
        std::vector<Dir> jump_dirs;
    };

}

#endif