#include <memory>

#include "RRT.hpp"
#include "Point2D.hpp"
#include "Obstacle.hpp"

int main()
{

  RRT::RRT rrt = RRT::RRT<Point2D>(0.3, 20000);

  auto start_goal = new RRT::Vertex<Point2D>({5, 5});
  auto end_goal = new RRT::Vertex<Point2D>({9, 9.5});

  Obstacle<Point2D> obs1({Point2D({1, 1}), Point2D({3, 3})});
  Obstacle<Point2D> obs2({Point2D({8, 1}), Point2D({9, 9})});
  Obstacle<Point2D> obs3({Point2D({6, 6}), Point2D({7, 7})});

  std::vector<Obstacle<Point2D>> obstacles{obs1, obs2, obs3};
  auto result = rrt.run(start_goal, end_goal, obstacles, false);

  return 0;
}
