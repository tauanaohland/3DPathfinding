#pragma once

#include <fstream>
#include <assert.h>
#include <string>
#include "Descriptors.h"
#include "ISearcher.h"
#include "jpsl/jpsl.hpp"
#include "3k/Node.h"

using namespace std;
using namespace JPSL;

class JPS : public ISearcher
{
    std::vector<bool> obstacles;
    int width, height, depth;
    std::function<float(const Point&, const Point&)> HeuristicFunction;

public:

    //conversion between Coord3D and Point in order to keep transparency with the rest of the code
    JPS(float (*heuristic)(const Coord3D& a, const Coord3D& b))
    {
        this->HeuristicFunction = [heuristic](const Point& a, const Point& b) -> float
        {
			return heuristic(Coord3D(a.x(), a.y(), a.z()), Coord3D(b.x(), b.y(), b.z()));
		};
    }

	std::string GetName() override
	{
		return "JPS";
	}

    void LoadMap(std::string filemap) override
    {
        ifstream infile;
        infile.open(filemap);

        string s;
        uint64_t x, y, z;

        infile >> s >> width >> height >> depth;

        obstacles.resize(width * height * depth, false);

        while (infile >> x >> y >> z)
            obstacles[x + y * width + z * width * height] = true;

        infile.close();
    }

    PathDescriptor Search(const Point& start, const Point& goal, const std::string& expansionPolicy, float TimeLimit, bool debug = false) override
    {
        auto state_valid = [this](const Point& p) -> bool
            {
                if (p.x() < 0 || p.y() < 0 || p.z() < 0 || uint64_t(p.x()) >= width || uint64_t(p.y()) >= height || uint64_t(p.z()) >= depth)
                    return false;
                return !obstacles[p.x() + p.y() * width + p.z() * height * width];
            };

        return plan(start, goal, -1, state_valid, TimeLimit, this->HeuristicFunction, expansionPolicy, debug);
    }
};