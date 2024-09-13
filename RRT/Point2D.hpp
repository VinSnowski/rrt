#ifndef _POINT2D_HPP_
#define _POINT2D_HPP_

#include <fstream>

struct Point2D
{
  double x;
  double y;

  Point2D(double x, double y) : x(x), y(y) {}

  Point2D operator*(const double scalar) const
  {
    return Point2D({this->x * scalar, this->y * scalar});
  }

  Point2D operator+(const Point2D &other) const
  {
    return Point2D{this->x + other.x, this->y + other.y};
  }

  Point2D operator-(const Point2D &other) const
  {
    return *this + Point2D{-other.x, -other.y};
  }

  friend std::ostream &operator<<(std::ostream &out, Point2D const &curr);
};

std::ostream &operator<<(std::ostream &out, Point2D const &curr)
{
  out << curr.x << "," << curr.y;
  return out;
}

#endif // _POINT2D_HPP_