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
#include <memory>
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

    Vertex() = default;
    Vertex(T val) : state(val), parent(nullptr) {}
  };

  template <typename T>
  class RRT
  {
  private:
    std::vector<std::unique_ptr<Vertex<T>>> graph;

    // Get the nearest vertex to a generated state
    Vertex<T> *getNearestNeighbor(const T &state)
    {
      double min_dist{std::numeric_limits<double>::max()};
      Vertex<T> *nearest = nullptr;
      for (const auto &v : this->graph)
      {
        double distance = calculateDistanceFromTo<T>(state, v->state);

        if (distance < min_dist)
        {
          min_dist = distance;
          nearest = v.get();
        }
      }
      return nearest;
    }

    // Generates a new vertex originating from nearest incrementing it in the direction of 'state'
    Vertex<T> generateNewVertex(const T &state, const Vertex<T> &nearest)
    {
      double distance = calculateDistanceFromTo<T>(state, nearest.state);
      double step = std::min(1.0, stepLength / distance);
      return Vertex(nearest.state + (state - nearest.state) * step);
    }

  public:
    double stepLength;
    unsigned K;

    RRT(double stepLength, unsigned K) : stepLength(stepLength), K(K) {}

    // Run the algorithm and try to obtain a path from start to goal
    // Results are stored in "graph.txt" and "path.txt"
    std::vector<Vertex<T> *> run(Vertex<T> startVertex, Vertex<T> goalVertex, const std::vector<Obstacle<T>> obstacles, bool stopEarly)
    {
      this->graph.push_back(std::make_unique<Vertex<T>>(startVertex));
      Vertex<T> *finalVertexPtr = nullptr;
      bool foundSolution = false;

      std::ofstream graph("graph.txt");
      graph << std::fixed << std::setprecision(3);

      for (size_t i = 0; i < K; i++)
      {
        Vertex<T> newVertex;
        Vertex<T> *nearestVertexPtr = nullptr;

        while (true)
        {
          auto randomState = generateRandomValue<T>();
          nearestVertexPtr = getNearestNeighbor(randomState);
          newVertex = generateNewVertex(randomState, *nearestVertexPtr);
          if (!isColliding(obstacles, nearestVertexPtr->state, newVertex.state))
            break;
        }

        this->graph.push_back(std::make_unique<Vertex<T>>(newVertex));
        this->graph.back()->parent = nearestVertexPtr;
        graph << this->graph.back()->state << "-" << nearestVertexPtr->state << "\n";

        if (!foundSolution && calculateDistanceFromTo<T>(this->graph.back()->state, goalVertex.state) < stepLength)
        {
          std::cout << "Found a path with " << i << " iterations" << std::endl;
          finalVertexPtr = this->graph.back().get();
          foundSolution = true;
          if (stopEarly)
          {
            break;
          }
        }
      }
      graph.close();

      std::ofstream path("path.txt");
      path << std::fixed << std::setprecision(3);
      std::vector<Vertex<T> *> solution;
      if (foundSolution)
      {
        while (finalVertexPtr != nullptr)
        {
          // std::cout << finalVertexPtr->state << std::endl;
          solution.push_back(finalVertexPtr);
          path << finalVertexPtr->state << "-";
          finalVertexPtr = finalVertexPtr->parent;
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