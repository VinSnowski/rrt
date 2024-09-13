#ifndef _OBSTACLE_HPP_
#define _OBSTACLE_HPP_

#include <vector>
#include "Point2D.hpp"

template <typename T>
struct Obstacle
{
  bool isColliding(T &a, T &b);
};

// This is a simple representation of a box in 2D defined by bottom left and top right points
template <>
struct Obstacle<Point2D>
{
  Point2D bottomLeftPoint;
  Point2D topRightPoint;

  Obstacle(Point2D blp, Point2D trp) : bottomLeftPoint(blp), topRightPoint(trp) {}

  // Assumes the step is small compared to the obstacle, so simply if point b is inside the obstacle, it is colliding
  // otherwise it is not
  bool isColliding(Point2D &a, Point2D &b)
  {
    return (b.x >= this->bottomLeftPoint.x &&
            b.x <= this->topRightPoint.x &&
            b.y >= this->bottomLeftPoint.y &&
            b.y <= this->topRightPoint.y);
  }
};

// detects collision between obstacles and two vertices that are connected
template <typename T>
bool isColliding(std::vector<Obstacle<T>> obstacles, T &a, T &b)
{
  for (auto obs : obstacles)
  {
    if (obs.isColliding(a, b))
    {
      return true;
    }
  }
  return false;
}

#endif // _OBSTACLE_HPP_