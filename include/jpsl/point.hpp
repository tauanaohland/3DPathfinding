#ifndef JPSL_POINT_HEADER 
#define JPSL_POINT_HEADER

#include <stdint.h>
#include <iostream>
#include <math.h>
#include <tuple>

#include "jpsl/dir.hpp"

namespace JPSL {

  class Point {
  public:
    Point(int64_t, int64_t, int64_t);
    Point(const Point &) = default;
    Point& operator=(const Point &) = default;
	Point(Point&&) = default;
	Point& operator=(Point&&) = default;
    Point() : x_(0), y_(0), z_(0) {}
    

    inline int64_t x() const {return x_;}
    inline int64_t y() const {return y_;}
    inline int64_t z() const {return z_;}

    Point operator+(const Dir &) const;
    Point & operator+=(const Dir &);

    Point operator-(const Dir &) const;   
    Point operator-(const Point &) const;
    bool operator==(const Point &) const;
    bool operator!=(const Point &) const;
    bool operator<(const Point &) const;
    float norm() const;
    float manhattan_norm() const;
	float distance(const Point&);
  
    Dir incoming_dir(const Point &) const;
    Dir direction_to(const Point &) const;

  private:
    int64_t x_, y_, z_;
  };

  inline std::ostream& operator<<(std::ostream& os, const JPSL::Point& p) {
      os << "[" << (int) p.x() << "," << (int) p.y() << "," << (int) p.z() << "]";
      return os;
  }
}

namespace std {
  template <>
  struct hash<JPSL::Point> {
    size_t operator()(const JPSL::Point & p) const {
      return hash<int64_t>()(p.x()) ^ hash<int64_t>()(p.y()) ^ hash<int64_t>()(p.z());
    }
  };
}

#endif