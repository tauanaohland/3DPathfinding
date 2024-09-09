#pragma once
#include <vector>
#include <cstdlib> 
#include <fstream>
#include "jpsl/point.hpp"
#include "3k/Node.h"

using namespace JPSL;

struct PathDescriptor {
    double cost;
    int visited_nodes;
	std::vector<Coord3D> path;
    int path_nodes = 0;
    float total_time;
    float path_lenght;

    PathDescriptor()
    {
        cost = -1;
        visited_nodes = -1;
        total_time = -1;
        path_lenght = -1;
        path_nodes = -1;
        path = std::vector<Coord3D>();
    }

    PathDescriptor(double cost, int visited_nodes, int path_nodes, float total_time, float path_lenght, std::vector<Coord3D> = std::vector<Coord3D>())
    {
        this->cost = cost;
        this->visited_nodes = visited_nodes;
        this->total_time = total_time;
        this->path_lenght = path_lenght;
        this->path_nodes = path_nodes;
        this->path = path;
    }

	void CopyPath(std::vector<Point> path)
	{
		//copy
		for (auto p : path)
			this->path.push_back(Coord3D(p));
	}

    void CopyPath(std::vector<Coord3D> path)
    {
        //copy
        for (auto p : path)
            this->path.push_back(Coord3D(p));
    }
    
    void PlotPath(std::string map_filename)
    {
        //save path in x y z format
		std::ofstream file;
		file.open("path.csv");
		for (auto p : path)
			file << p.x << " " << p.y << " " << p.z << std::endl;
		file.close();
        
        //system call to run "python plot_path map_filename"
		system(("python plot_path.py " + map_filename).c_str());
    }
};