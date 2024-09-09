#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <queue>
#include <cmath>
#include <string>
#include <chrono>
#include <ctime>
#include <random>
#include <stdexcept>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "ISearcher.h"
#include "3k/Node.h"
#include "Descriptors.h"
#include "3k/K3Dirs.h"

using namespace std;

class LazyTheta : public ISearcher
{

private:
    string map_name;
    string map_filename_;
    string map_color;
    vector<Coord3D> map_obj;
    set<Coord3D> blocked_set;
    int depth, width, height;

    vector<Coord3D> k3_vectors;
    vector<Coord3D> k4_vectors;
    vector<Coord3D> k5_vectors;
    vector<float> k3_costs;
    vector<float> k4_costs;
    vector<float> k5_costs;
    float cost;
    float path_size;
    float visited_size;
    int num_nos_abertos;
    bool debug;

    float (*HeuristicFunction)(const Coord3D& a, const Coord3D& b);
    

public:
    // Constructor
    LazyTheta(float (*HeuristicFunction)(const Coord3D& a, const Coord3D& b))
    {
        // Pre-calculate the neighbors and its costs
        k3_vectors = find_3d_neighbors(Coord3D(0, 0, 0), 3);
        for (const auto& vector : k3_vectors) {
            k3_costs.push_back(vector.length());
        }
        k4_vectors = find_3d_neighbors(Coord3D(0, 0, 0), 4);
        for (const auto& vector : k4_vectors) {
            k4_costs.push_back(vector.length());
        }
        k5_vectors = find_3d_neighbors(Coord3D(0, 0, 0), 5);
        for (const auto& vector : k5_vectors) {
            k5_costs.push_back(vector.length());
        }

        cost = numeric_limits<float>::infinity();
        path_size = 1.0;
        visited_size = 1.0;
        debug = true;
        this->HeuristicFunction = HeuristicFunction;

        num_nos_abertos = 0;
    }

    void LoadMap(std::string mapfile) override
    {
        map_obj = LoadMapFromVoxel(mapfile);
        blocked_set = ConverToBlockedSet(map_obj);
	}

	std::string GetName() override
	{
		return "Lazy Theta";
	}

    vector<Coord3D> LoadMapFromVoxel(const string& map_filename) 
    {
        vector<Coord3D> map;

        try {
            ifstream file(map_filename);
            if (!file.is_open()) {
                throw runtime_error("Failed to open file");
            }

            string line;
            getline(file, line);
            sscanf(line.c_str(), "%*s %d %d %d", &width, &height, &depth);

            while (getline(file, line)) {
                int x, y, z;
                sscanf(line.c_str(), "%d %d %d", &x, &y, &z);
                map.emplace_back(x, y, z);
            }

        }
        catch (const exception& e) {
            map.clear();
            cerr << e.what() << endl;
        }

        return map;
    }

    set<Coord3D> ConverToBlockedSet(const vector<Coord3D>& map_obj) {
        set<Coord3D> blocked_set;

        if (map_obj.empty()) {
            throw invalid_argument("The map object is empty.");
        }

        for (const auto& coord : map_obj) {
            blocked_set.emplace(coord);
        }

        return blocked_set;
    }

    bool HasVisibility(const Coord3D& current_node_coord, const Coord3D& neighbour_coord) {
        Coord3D dir_vect(neighbour_coord.x - current_node_coord.x,
            neighbour_coord.y - current_node_coord.y,
            neighbour_coord.z - current_node_coord.z);

        int steps = max({ abs(dir_vect.x), abs(dir_vect.y), abs(dir_vect.z) });
        float x_increment = static_cast<float>(dir_vect.x) / steps;
        float y_increment = static_cast<float>(dir_vect.y) / steps;
        float z_increment = static_cast<float>(dir_vect.z) / steps;

        float x = current_node_coord.x;
        float y = current_node_coord.y;
        float z = current_node_coord.z;

        for (int i = 0; i <= steps; ++i) {
            Coord3D coord(static_cast<int>(round(x)), static_cast<int>(round(y)), static_cast<int>(round(z)));

            if (blocked(coord)) {
                return false;
            }

            x += x_increment;
            y += y_increment;
            z += z_increment;
        }

        return true;
    }

    vector<pair<Coord3D, float>> Neighbours(const Coord3D& coord, const string& find_neighbors = "26-adjacency") 
    {
        vector<pair<Coord3D, float>> n;

        if (find_neighbors == "k3-neighbors" || find_neighbors == "26-adjacency") {
            for (size_t i = 0; i < k3_vectors.size(); ++i) {
                n.emplace_back(coord + k3_vectors[i], k3_costs[i]);
            }
            return n;
        }

        if (find_neighbors == "k4-neighbors") {
            for (size_t i = 0; i < k4_vectors.size(); ++i) {
                n.emplace_back(coord + k4_vectors[i], k4_costs[i]);
            }
            return n;
        }

        if (find_neighbors == "k5-neighbors") {
            for (size_t i = 0; i < k5_vectors.size(); ++i) {
                n.emplace_back(coord + k5_vectors[i], k5_costs[i]);
            }
            return n;
        }

        return n;
    }

    std::vector<Coord3D> ReconstructPath(const Coord3D& goal, std::unordered_map<Coord3D, std::shared_ptr<Node>, Coord3D::Hash> allNodes)
    {
        std::vector<Coord3D> path;
        for (auto it= allNodes.find(goal); it != allNodes.end(); it = allNodes.find(it->second->parent))
        {
            path.push_back(it->first);
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    void SetVertex(std::shared_ptr<Node> currentNode, std::unordered_map<Coord3D, std::shared_ptr<Node>, Coord3D::Hash>& allNodes)
    {
        auto newParent = allNodes[currentNode->regular_parent];
		if (HasVisibility(newParent->coord, currentNode->coord))
		{
			currentNode->parent = newParent->coord;
			currentNode->g = newParent->g + Euclidian(newParent->coord, currentNode->coord);
            return;
		}

		currentNode->g = std::numeric_limits<float>::infinity();
    }

    PathDescriptor Search(const JPSL::Point& startPoint, const JPSL::Point& goalPoint, const std::string& expansionPolicy, float TimeLimit, bool debug = false) override
    {
        auto compare = [](const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs)
        {
            return lhs->cost() > rhs->cost();
        };

        // Lists
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(compare)> openList(compare);
        std::unordered_map<Coord3D, std::shared_ptr<Node>, Coord3D::Hash> allNodes;
        std::unordered_set<Coord3D, Coord3D::Hash> closedList;

        auto startTime = std::chrono::steady_clock::now();

        // Compatibility with JPS
        Coord3D start(startPoint);
        Coord3D goal(goalPoint);

        auto startNode = std::make_shared<Node>(start, 0, HeuristicFunction(start, goal));
        openList.push(startNode);
        allNodes[start] = startNode;

        while (!openList.empty())
        {
            // Check time limit
            auto currentTime = std::chrono::steady_clock::now();
            float elapsedTime = std::chrono::duration<float>(currentTime - startTime).count();
            if (elapsedTime > TimeLimit)
            {
                float pathLength = std::sqrt(std::pow(goal.x - start.x, 2) + std::pow(goal.y - start.y, 2) + std::pow(goal.z - start.z, 2));

                return PathDescriptor(-1, -1, -1, elapsedTime, pathLength);
            }

            auto currentNode = openList.top();
            openList.pop();

			//check node has already been expanded
			if (blocked(currentNode->coord) || closedList.find(currentNode->coord) != closedList.end())
			{
				continue;
			}
            
            // Lazy Theta* main logic
            auto parentCoordNode = allNodes.find(currentNode->parent);
            if(parentCoordNode != allNodes.end() && !HasVisibility(parentCoordNode->first, currentNode->coord))
            {
                SetVertex(currentNode, allNodes);

                //not rechable
                if (currentNode->g == std::numeric_limits<float>::infinity())
                {
                    //remove from all nodes
					allNodes.erase(currentNode->coord);
                    continue;
                }

                //update allNodes
                allNodes[currentNode->coord] = currentNode;
                parentCoordNode = allNodes.find(currentNode->regular_parent);
            }

            if (currentNode->coord == goal)
            {
                std::vector<Coord3D> path = ReconstructPath(currentNode->coord, allNodes);

                float totalCost = currentNode->g;
                int visitedNodes = closedList.size();
                float totalTime = elapsedTime;
                float pathLength = std::sqrt(std::pow(goal.x - start.x, 2) + std::pow(goal.y - start.y, 2) + std::pow(goal.z - start.z, 2));
                
                allNodes.clear();
                closedList.clear();                
                PathDescriptor descriptor(totalCost, visitedNodes + closedList.size(), path.size(), totalTime, pathLength);

                if (debug)
                {
                    descriptor.CopyPath(path);
                }

                return descriptor;
            }

            // regular lazy expansion
            for (const auto& neighbor : Neighbours(currentNode->coord, expansionPolicy))
            {
                const Coord3D& neighborCoord = neighbor.first;
                auto it = allNodes.find(neighborCoord);
                float tentativeG;

                //first node, does not have a parent
                if (parentCoordNode != allNodes.end())
                {
                    tentativeG = parentCoordNode->second->g + Euclidian(parentCoordNode->first, neighborCoord);
                }
                else
                {
                    tentativeG = currentNode->g + neighbor.second;
                }

                if (it == allNodes.end() || tentativeG < it->second->g)
                {
                    std::shared_ptr<Node> neighborNode;

                    if (parentCoordNode != allNodes.end())
                    {
                        neighborNode = std::make_shared<Node>(neighborCoord, tentativeG, HeuristicFunction(neighborCoord, goal), parentCoordNode->first, currentNode->coord);
                    }
                    else
                    {
                        neighborNode = std::make_shared<Node>(neighborCoord, tentativeG, HeuristicFunction(neighborCoord, goal), currentNode->coord, currentNode->coord);
                    }

                    openList.push(neighborNode);
                    allNodes[neighborCoord] = neighborNode;
                }
                
            }

            closedList.insert(currentNode->coord);
        }

        return PathDescriptor(); // No path found
    }

    

private:
    bool blocked(const Coord3D& coord) 
    {
        return blocked_set.find(coord) != blocked_set.end();
    }

    float Euclidian(const Coord3D& a, const Coord3D& b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
    }

};
