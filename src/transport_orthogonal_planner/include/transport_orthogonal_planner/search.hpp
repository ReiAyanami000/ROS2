// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace transport_orthogonal_planner
{
struct Point {double x, y;};
struct Grid
{
  int width{0}, height{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  std::vector<uint8_t> costs;  // Nav2 raw costs: 253 inscribed, 254 lethal, 255 unknown.
};
struct Options
{
  double radius{0.0};             // Conservative circular robot envelope in meters.
  double turn_penalty{0.10};      // Equivalent additional meters per 90-degree turn.
  double cost_weight{2.0};
  double timeout{3.0};
  size_t max_expansions{1000000};
  size_t max_states{2000000};
};
enum class ErrorCode
{
  Invalid, StartOutside, GoalOutside, StartOccupied, GoalOccupied,
  NoPath, Timeout, Cancelled
};
class SearchError : public std::runtime_error
{
public:
  SearchError(ErrorCode value, const std::string & message)
  : std::runtime_error(message), code(value) {}
  ErrorCode code;
};
struct Result {std::vector<Point> points; size_t expansions{0}; double cost{0};};

class Search
{
public:
  Search(const Grid & grid, const Options & options, std::function<bool()> cancel = {})
  : grid_(grid), options_(options), cancel_(cancel), begin_(Clock::now())
  {
    if (grid.width <= 0 || grid.height <= 0 || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0 || !std::isfinite(grid.origin_x) ||
      !std::isfinite(grid.origin_y) ||
      grid.costs.size() != static_cast<size_t>(grid.width) * grid.height ||
      !std::isfinite(options.radius) || options.radius < 0 ||
      !std::isfinite(options.turn_penalty) || options.turn_penalty < 0 ||
      !std::isfinite(options.cost_weight) || options.cost_weight < 0 ||
      !std::isfinite(options.timeout) || options.timeout <= 0 ||
      options.max_states < 5 || options.max_expansions == 0)
    {
      throw SearchError(ErrorCode::Invalid, "Invalid grid or planner parameters");
    }
    if (grid.costs.size() > options.max_states / 5) {
      throw SearchError(ErrorCode::Invalid, "Map exceeds max_states budget");
    }
    buildMask();
  }

  Result plan(Point start, Point goal)
  {
    checkBudget();
    if (!inside(start)) {throw SearchError(ErrorCode::StartOutside, "Start outside map");}
    if (!inside(goal)) {throw SearchError(ErrorCode::GoalOutside, "Goal outside map");}
    if (!segmentFree(start, start)) {
      throw SearchError(ErrorCode::StartOccupied, "Start lacks circular footprint clearance");
    }
    if (!segmentFree(goal, goal)) {
      throw SearchError(ErrorCode::GoalOccupied, "Goal lacks circular footprint clearance");
    }
    if (start.x == goal.x && start.y == goal.y) {return {{goal}, 0, 0};}

    // Include exact endpoint axes, so no diagonal snapping connector is ever needed.
    auto xs = axis(grid_.width, grid_.origin_x, start.x, goal.x);
    auto ys = axis(grid_.height, grid_.origin_y, start.y, goal.y);
    const size_t nx = xs.size(), ny = ys.size();
    if (nx > options_.max_states / 5 / ny) {
      throw SearchError(ErrorCode::Invalid, "Augmented grid exceeds max_states budget");
    }
    const size_t count = nx * ny * 5;
    auto locate = [](const std::vector<double> & a, double v) {
        return static_cast<size_t>(std::lower_bound(a.begin(), a.end(), v) - a.begin());
      };
    const size_t start_node = locate(ys, start.y) * nx + locate(xs, start.x);
    const size_t goal_node = locate(ys, goal.y) * nx + locate(xs, goal.x);
    auto point = [&](size_t node) {return Point{xs[node % nx], ys[node / nx]};};
    auto heuristic = [&](Point p) {return std::abs(goal.x-p.x) + std::abs(goal.y-p.y);};

    const size_t none = std::numeric_limits<size_t>::max();
    std::vector<double> distance(count, std::numeric_limits<double>::infinity());
    std::vector<size_t> parent(count, none);
    struct Item {double f, g; size_t state, order;};
    struct Compare
    {
      bool operator()(const Item & a, const Item & b) const
      {
        if (a.f != b.f) {return a.f > b.f;}
        if (a.g != b.g) {return a.g < b.g;}
        return a.order > b.order;
      }
    };
    std::priority_queue<Item, std::vector<Item>, Compare> open;
    const size_t first = start_node * 5 + 4;  // 4: no previous movement direction.
    distance[first] = 0;
    size_t order = 0, expanded = 0, last = none;
    open.push({heuristic(start), 0, first, order++});
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    while (!open.empty()) {
      if ((expanded & 255U) == 0) {checkBudget();}
      const Item current = open.top(); open.pop();
      if (current.g != distance[current.state]) {continue;}
      const size_t node = current.state / 5;
      const int previous = static_cast<int>(current.state % 5);
      if (node == goal_node) {last = current.state; break;}
      if (++expanded > options_.max_expansions) {
        throw SearchError(ErrorCode::Timeout, "Exceeded max_expansions");
      }
      const int x = static_cast<int>(node % nx), y = static_cast<int>(node / nx);
      const Point a = point(node);
      for (int direction = 0; direction < 4; ++direction) {
        const int xx = x + dx[direction], yy = y + dy[direction];
        if (xx < 0 || yy < 0 || xx >= static_cast<int>(nx) || yy >= static_cast<int>(ny)) {
          continue;
        }
        const size_t next_node = static_cast<size_t>(yy) * nx + xx;
        const Point b = point(next_node);
        uint8_t cost = 0;
        if (!segmentFree(a, b, &cost)) {continue;}
        const double length = std::abs(b.x-a.x) + std::abs(b.y-a.y);
        const int turns = previous == 4 || previous == direction ? 0 :
          (previous / 2 == direction / 2 ? 2 : 1);
        const double next_g = current.g + length * (1 + options_.cost_weight * cost / 252.0) +
          turns * options_.turn_penalty;
        const size_t state = next_node * 5 + direction;
        if (next_g < distance[state]) {
          distance[state] = next_g;
          parent[state] = current.state;
          open.push({next_g + heuristic(b), next_g, state, order++});
        }
      }
    }
    if (last == none) {throw SearchError(ErrorCode::NoPath, "No orthogonal path exists");}
    std::vector<Point> reverse;
    for (size_t s = last; s != none; s = parent[s]) {reverse.push_back(point(s/5));}
    std::reverse(reverse.begin(), reverse.end());
    std::vector<Point> corners;
    for (Point p : reverse) {
      if (corners.size() >= 2) {
        const Point a = corners[corners.size()-2], b = corners.back();
        const bool same_x = a.x == b.x && b.x == p.x && (b.y-a.y)*(p.y-b.y) > 0;
        const bool same_y = a.y == b.y && b.y == p.y && (b.x-a.x)*(p.x-b.x) > 0;
        if (same_x || same_y) {corners.pop_back();}
      }
      corners.push_back(p);
    }
    for (size_t i=1; i<corners.size(); ++i) {
      checkBudget();
      if (!segmentFree(corners[i-1], corners[i])) {
        throw SearchError(ErrorCode::NoPath, "Final segment failed collision validation");
      }
    }
    return {corners, expanded, distance[last]};
  }

  bool segmentFree(Point a, Point b, uint8_t * max_cost = nullptr) const
  {
    if (!inside(a) || !inside(b) || (a.x != b.x && a.y != b.y)) {return false;}
    const double r = grid_.resolution;
    // Include both cells when a segment lies exactly on a grid boundary.
    const double eps = 1e-8;
    int x0 = static_cast<int>(std::floor((std::min(a.x,b.x)-grid_.origin_x)/r-eps));
    int x1 = static_cast<int>(std::floor((std::max(a.x,b.x)-grid_.origin_x)/r+eps));
    int y0 = static_cast<int>(std::floor((std::min(a.y,b.y)-grid_.origin_y)/r-eps));
    int y1 = static_cast<int>(std::floor((std::max(a.y,b.y)-grid_.origin_y)/r+eps));
    if (x0 < 0 || y0 < 0 || x1 >= grid_.width || y1 >= grid_.height) {return false;}
    uint8_t cost = 0;
    for (int y=y0; y<=y1; ++y) {
      for (int x=x0; x<=x1; ++x) {
        const size_t i = static_cast<size_t>(y)*grid_.width+x;
        if (blocked_[i]) {return false;}
        cost = std::max(cost, grid_.costs[i]);
      }
    }
    if (max_cost) {*max_cost = cost;}
    return true;
  }

private:
  using Clock = std::chrono::steady_clock;
  const Grid & grid_;
  Options options_;
  std::function<bool()> cancel_;
  Clock::time_point begin_;
  std::vector<uint8_t> blocked_;

  void checkBudget() const
  {
    if (cancel_ && cancel_()) {throw SearchError(ErrorCode::Cancelled, "Planning cancelled");}
    if (std::chrono::duration<double>(Clock::now()-begin_).count() > options_.timeout) {
      throw SearchError(ErrorCode::Timeout, "Planning time limit exceeded");
    }
  }
  bool inside(Point p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
      p.x >= grid_.origin_x && p.y >= grid_.origin_y &&
      p.x < grid_.origin_x + grid_.width*grid_.resolution &&
      p.y < grid_.origin_y + grid_.height*grid_.resolution;
  }
  std::vector<double> axis(int size, double origin, double start, double goal) const
  {
    std::vector<double> values{start, goal};
    for (int i=0; i<size; ++i) {values.push_back(origin+(i+0.5)*grid_.resolution);}
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
  }
  void buildMask()
  {
    // Conservative cell-area clearance, based only on lethal/unknown cells.
    // Do NOT dilate the already-inflated soft costs by the robot radius a second time.
    checkBudget();
    blocked_.resize(grid_.costs.size(), 0);
    const double r = grid_.resolution, radius = options_.radius;
    for (int y=0; y<grid_.height; ++y) {
      for (int x=0; x<grid_.width; ++x) {
        const size_t i = static_cast<size_t>(y)*grid_.width+x;
        if (grid_.costs[i]>=253 || x*r < radius || y*r < radius ||
          (x+1)*r > grid_.width*r-radius || (y+1)*r > grid_.height*r-radius)
        {blocked_[i]=1;}
      }
    }
    if (radius == 0) {return;}
    const int reach = static_cast<int>(std::min(
      std::ceil(radius/r)+1, static_cast<double>(std::max(grid_.width,grid_.height))));
    for (int y=0; y<grid_.height; ++y) {
      checkBudget();
      for (int x=0; x<grid_.width; ++x) {
        if (grid_.costs[static_cast<size_t>(y)*grid_.width+x]<254) {continue;}
        for (int yy=std::max(0,y-reach); yy<=std::min(grid_.height-1,y+reach); ++yy) {
          for (int xx=std::max(0,x-reach); xx<=std::min(grid_.width-1,x+reach); ++xx) {
            const double dx=std::max(0,std::abs(xx-x)-1)*r;
            const double dy=std::max(0,std::abs(yy-y)-1)*r;
            if (dx*dx+dy*dy <= radius*radius+1e-14) {
              blocked_[static_cast<size_t>(yy)*grid_.width+xx]=1;
            }
          }
        }
      }
    }
  }
};
}  // namespace transport_orthogonal_planner
