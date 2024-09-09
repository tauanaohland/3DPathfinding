#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <sstream>
#include "jpsl/point.hpp"

class Coord3D {
public:
    int x, y, z;

    Coord3D(int x, int y, int z) : x(x), y(y), z(z) {}
    Coord3D() : x(0), y(0), z(0) {}
	Coord3D(const JPSL::Point& point) : x(point.x()), y(point.y()), z(point.z()) {}

    float length() const 
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    std::string toString() const 
    {
        std::ostringstream oss;
        oss << "(" << x << "; " << y << "; " << z << ")";
        return oss.str();
    }

    bool operator==(const Coord3D& other) const 
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator==(const Coord3D* other) const
    {
        if (other == nullptr)
        {
            return false;
        }
        
        return x == other->x && y == other->y && z == other->z;
    }
    
    bool operator!=(const Coord3D& other) const
    {
        return x != other.x || y != other.y || z != other.z;
    }

    Coord3D operator+(const Coord3D& other) const 
    {
        return Coord3D(x + other.x, y + other.y, z + other.z);
    }

    Coord3D operator-(const Coord3D& other) const 
    {
        return Coord3D(x - other.x, y - other.y, z - other.z);
    }

    Coord3D operator*(float scalar) const 
    {
        return Coord3D(x * scalar, y * scalar, z * scalar);
    }

    bool operator<(const Coord3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }

    float distance(const Coord3D& other, const std::string& method = "euclidean") const {
        if (method == "euclidean") {
            return std::sqrt((x - other.x) * x - other.x + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
        }
        else if (method == "voxel3d") {
            float x_delta = std::abs(x - other.x);
            float y_delta = std::abs(y - other.y);
            float z_delta = std::abs(z - other.z);

            float d_min = std::min({ x_delta, y_delta, z_delta });
            float d_max = std::max({ x_delta, y_delta, z_delta });
            float d_mid = x_delta + y_delta + z_delta - d_min - d_max;

            return ((std::sqrt(3) - std::sqrt(2)) * d_min + (std::sqrt(2) - 1) * d_mid + d_max);
        }
        else if (method == "manhattan") {
            return std::abs(x - other.x) + std::abs(y - other.y) + std::abs(z - other.z);
        }
        else if (method == "chebyshev") {
            return std::max({ std::abs(x - other.x), std::abs(y - other.y), std::abs(z - other.z) });
        }
        else {
            throw std::invalid_argument("Unsupported distance method.");
        }
    }

    std::size_t hash() const {
        std::size_t seed = 0;
        seed ^= std::hash<int>{}(x)+0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(y)+0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(z)+0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }

    struct Hash
    {
		std::size_t operator()(const Coord3D& coord) const
		{
			return coord.hash();
		}
    };
};

// Custom hash function for Coord3D
namespace std {
    template <>
    struct hash<Coord3D> {
        std::size_t operator()(const Coord3D& coord) const {
            return coord.hash();
        }
    };
}

class Node {
public:

    Coord3D coord;
    float g; // Cost from start to node
    float h; // Heuristic cost to goal
    Coord3D parent; // Parent node
	Coord3D regular_parent; // auxiliar previus node for Lazy Theta*

	Node(const Coord3D& coord, float g, float h, const Coord3D& prev = Coord3D(), const Coord3D& reg_par = Coord3D())
        : coord(coord), g(g), h(h), parent(prev), regular_parent(reg_par) {}

    bool operator==(const Node& other) const {
        return coord == other.coord;
    }

    bool operator<(const Node& other) const {
        return cost() < other.cost();
    }

    bool operator<=(const Node& other) const {
        return cost() <= other.cost();
    }

    bool operator>(const Node& other) const {
        return cost() > other.cost();
    }

    bool operator>=(const Node& other) const {
        return cost() >= other.cost();
    }

    std::size_t hash() const {
        std::size_t seed = 0;
        seed ^= coord.hash() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<double>{}(g)+0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }

    double cost() const {
        return g + h;
    }
};
