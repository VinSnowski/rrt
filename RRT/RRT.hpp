#ifndef _RRT_HPP_
#define _RRT_HPP_
#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <limits>
#include <cmath>
#include <fstream>
#include <algorithm>
#include "Point2D.hpp"
#include "Obstacle.hpp"

namespace RRT
{
  template <typename T>
  T generateRandomValue();

  template <typename T>
  double calculateDistanceFromTo(const T &a, const T &b);

  template <typename T>
  struct Vertex;

  template <typename T>
  struct Vertex
  {
    T state;
    Vertex<T> *parent;

    Vertex(T val) : state(val), parent(nullptr) {}
  };

  template <typename T>
  class RRT
  {
  private:
    std::vector<Vertex<T> *> graph;

    // Get the nearest vertex to a generated state
    Vertex<T> *getNearestNeighbor(const T &state)
    {
      double min_dist{std::numeric_limits<double>::max()};
      Vertex<T> *nearest = nullptr;
      for (Vertex<T> *v : this->graph)
      {
        double distance = calculateDistanceFromTo<T>(state, v->state);

        if (distance < min_dist)
        {
          min_dist = distance;
          nearest = v;
        }
      }
      return nearest;
    }

    // Get a new vertex originating from nearest incrementing it in the direction of 'state'
    Vertex<T> *getNewVertex(const T &state, Vertex<T> *nearest)
    {
      double distance = calculateDistanceFromTo<T>(state, nearest->state);

      double step = std::min(1.0, stepLength / distance);

      return new Vertex(nearest->state + (state - nearest->state) * step);
    }

  public:
    double stepLength;
    unsigned K;

    RRT(double stepLength, unsigned K) : stepLength(stepLength), K(K) {}

    // Run the algorithm and try to obtain a path from start to goal
    // Results are stored in "graph.txt" and "path.txt"
    std::vector<Vertex<T> *> run(Vertex<T> *start, Vertex<T> *goal, const std::vector<Obstacle<T>> obstacles, bool stopEarly)
    {
      this->graph.push_back(start);

      Vertex<T> *final_vertex = nullptr;

      bool foundSolution = false;

      std::cout << std::fixed << std::setprecision(3);
      std::ofstream graph("results/graph.txt");

      for (size_t i = 0; i < K; i++)
      {
        Vertex<T> *new_vertex = nullptr;
        Vertex<T> *nearest_vertex = nullptr;

        while (true)
        {
          auto random_state = generateRandomValue<T>();
          nearest_vertex = getNearestNeighbor(random_state);
          new_vertex = getNewVertex(random_state, nearest_vertex);
          if (!isColliding(obstacles, nearest_vertex->state, new_vertex->state))
            break;
        }

        this->graph.push_back(new_vertex);
        new_vertex->parent = nearest_vertex;

        graph << new_vertex->state << "-" << nearest_vertex->state << "\n";

        if (!foundSolution && calculateDistanceFromTo<T>(new_vertex->state, goal->state) < stepLength)
        {
          std::cout << "Found a path with " << i << " iterations" << std::endl;
          final_vertex = new_vertex;
          foundSolution = true;
          if (stopEarly)
          {
            break;
          }
        }
      }
      graph.close();

      std::ofstream path("results/path.txt");

      std::vector<Vertex<T> *> solution;

      if (foundSolution)
      {
        while (final_vertex != nullptr)
        {
          solution.push_back(final_vertex);
          path << final_vertex->state << "-";
          final_vertex = final_vertex->parent;
        }
      }

      path.close();
      return solution;
    }
  };

}

template <>
Point2D RRT::generateRandomValue()
{
  double MIN_X{0.0};
  double MIN_Y{0.0};
  double MAX_X{10.0};
  double MAX_Y{10.0};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis_x(MIN_X, MAX_X);
  std::uniform_real_distribution<double> dis_y(MIN_Y, MAX_Y);
  return Point2D({dis_x(gen), dis_y(gen)});
}

template <>
double RRT::calculateDistanceFromTo(const Point2D &a, const Point2D &b)
{
  return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

#endif // _RRT_HPP_