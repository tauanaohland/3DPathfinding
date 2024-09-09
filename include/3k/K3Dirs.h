#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include "Node.h"

std::vector<Coord3D> MapToAllOctants(const std::vector<Coord3D>& coords)
{
    std::vector<Coord3D> all_octants;

    for (int x : {-1, 1})
    {
        for (int y : {-1, 1})
        {
            for (int z : {-1, 1})
            {
                for (const auto& coord : coords)
                {
                    all_octants.emplace_back(x * coord.x, y * coord.y, z * coord.z);
                }
            }
        }
    }

    //remove duplicates
    std::sort(all_octants.begin(), all_octants.end());
    all_octants.erase(std::unique(all_octants.begin(), all_octants.end()), all_octants.end());
    return all_octants;
}

std::vector<Coord3D> ExpandNeighbors(Coord3D v1, Coord3D v2, Coord3D v3, int depth)
{
    std::vector<Coord3D> expanded_neighbors;

    if (depth == 0)
    {
        return { v1, v2, v3 };
    }

    Coord3D v4 = v1 + v2;
    Coord3D v5 = v2 + v3;
    Coord3D v6 = v1 + v3;
    Coord3D v7 = v1 + v2 + v3;

    Coord3D combinations[6][3] =
    {
        { v7, v1, v4 }, 
        { v7, v2, v5}, 
        { v7, v3, v6 }, 
        { v7, v1, v6 },
        { v7, v2, v4 }, 
        { v7, v3, v5 }
    };

    for (const auto& combination : combinations)
    {
        std::vector<Coord3D> new_neighbors = ExpandNeighbors(combination[0], combination[1], combination[2], depth - 1);
        expanded_neighbors.insert(expanded_neighbors.end(), new_neighbors.begin(), new_neighbors.end());
    }

    //remove duplicates
    std::sort(expanded_neighbors.begin(), expanded_neighbors.end());
    expanded_neighbors.erase(std::unique(expanded_neighbors.begin(), expanded_neighbors.end()), expanded_neighbors.end());

    return expanded_neighbors;
}


std::vector<Coord3D> find_3d_neighbors(const Coord3D& coord, int k)
{
    int depth = k - 2;
    std::vector<Coord3D> all_vectors;

    Coord3D initial_v1 = Coord3D(0, 0, 1);
    Coord3D initial_v2 = Coord3D(0, 1, 0);
    Coord3D initial_v3 = Coord3D(1, 0, 0);

    std::vector<Coord3D> new_vectors_from_initial = ExpandNeighbors(initial_v1, initial_v2, initial_v3, depth);
    all_vectors.insert(all_vectors.end(), new_vectors_from_initial.begin(), new_vectors_from_initial.end());

    std::vector<Coord3D> all_octants = MapToAllOctants(all_vectors);

    std::vector<Coord3D> neighbors;

    for (const auto& vec : all_octants)
    {
        neighbors.emplace_back(vec + coord);
    }

    return neighbors;
}

void Save3KDirs(std::string filename, int depth)
{
    std::vector<Coord3D> all_dirs;
    all_dirs = find_3d_neighbors(Coord3D(0, 0, 0), depth);

    //sort the coord based on its lenght
	std::sort(all_dirs.begin(), all_dirs.end(), [](const Coord3D& a, const Coord3D& b) {
		return a.length() > b.length();
		});

    // save in the style: const std::vector<Dir> NEIGHBORS_3D({{-1,-1,-1}, {1,1,1}, \...});
    std::ofstream file;
    file.open(filename);

    file << "#include <vector>\n";
    file << "const std::vector<Dir> NEIGHBORS_3D_K" << depth << "({";
    for (const auto& dir : all_dirs)
    {
		file << "{" << dir.x << "," << dir.y << "," << dir.z << "}, \\ \n";
	}
    file << "});";
    file.close();
}