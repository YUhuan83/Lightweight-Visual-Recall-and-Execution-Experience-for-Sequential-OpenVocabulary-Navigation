// Diagnostic only: installed Nav2 checker, no subscriptions or vehicle control.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

using Map = nav2_costmap_2d::Costmap2D;
using Footprint = nav2_costmap_2d::Footprint;
constexpr unsigned BINS = 72;

Footprint body()
{
  Footprint f;
  for (auto xy : std::vector<std::pair<double, double>>{
    {3.41, 1.06}, {3.41, -1.06}, {-1.06, -1.06}, {-1.06, 1.06}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first; p.y = xy.second; f.push_back(p);
  }
  return f;
}

struct Pose { double x, y, yaw; };

unsigned angleBin(double yaw)
{
  // SmacPlannerHybrid 1.1.20 start-heading rounding (not a path quaternion).
  double value = std::round(yaw / (2.0 * M_PI / BINS));
  while (value < 0) { value += BINS; }
  while (value >= BINS) { value -= BINS; }
  return static_cast<unsigned>(value);
}

struct Costs { double continuous; bool grid_collision; float grid_cost; };

Costs inspect(Map & map, const Pose & p, bool allow_unknown, double shortcut_cost = 0)
{
  unsigned mx, my;
  if (!map.worldToMap(p.x, p.y, mx, my)) {
    throw std::runtime_error("Pose outside map");
  }
  nav2_costmap_2d::FootprintCollisionChecker<Map *> continuous(&map);
  nav2_smac_planner::GridCollisionChecker grid(&map, BINS, nullptr);
  // Force real rectangular footprint checks, no inflation-field shortcut.
  grid.setFootprint(body(), false, shortcut_cost);
  bool collision = grid.inCollision(
    static_cast<float>(mx), static_cast<float>(my),
    static_cast<float>(angleBin(p.yaw)), allow_unknown);
  return {continuous.footprintCostAtPose(p.x, p.y, p.yaw, body()),
    collision, grid.getCost()};
}

class InflationFormula : public nav2_costmap_2d::InflationLayer
{
public:
  double circumscribedThreshold(double resolution, double radius, double scaling)
  {
    resolution_ = resolution; inflation_radius_ = radius; cost_scaling_factor_ = scaling;
    double circumscribed;
    nav2_costmap_2d::calculateMinAndMaxDistances(body(), inscribed_radius_, circumscribed);
    return computeCost(circumscribed / resolution_);
  }
};

void require(bool condition, const char * message)
{
  if (!condition) { throw std::runtime_error(message); }
}

void controls()
{
  Map map(80, 80, 0.25, -10, -10, 0);
  const Pose p{0.125, 0.125, 0};
  auto free = inspect(map, p, false);
  require(free.continuous == 0 && !free.grid_collision, "free rectangle rejected");
  unsigned mx, my;
  map.worldToMap(3.5, 0, mx, my);
  map.setCost(mx, my, 254);
  auto blocked = inspect(map, p, true);
  require(blocked.continuous == 254 && blocked.grid_collision,
    "lethal body edge was ignored even with allow_unknown");
  InflationFormula formula;
  const auto shortcut = formula.circumscribedThreshold(0.25, 2.0, 2.0);
  require(shortcut == 1, "frozen global inflation threshold changed");
  require(!inspect(map, p, true, shortcut).grid_collision,
    "expected narrow-inflation shortcut counterexample not reproduced");
  map.setCost(mx, my, 0);
  map.worldToMap(p.x, p.y, mx, my);
  map.setCost(mx, my, 255);
  require(inspect(map, p, false).grid_collision, "unknown center must be rejected");
  require(!inspect(map, p, true).grid_collision, "allowed unknown center rejected");
  map.setCost(mx, my, 253);
  require(inspect(map, p, true).grid_collision, "inscribed center ignored");
  map.setCost(mx, my, 254);
  require(inspect(map, p, true).grid_collision, "lethal center ignored");
  require(angleBin(-M_PI / 3) == 60 && angleBin(2 * M_PI) == 0,
    "heading wrap or rounding changed");
  std::cout << "9 checker controls passed\n";
}

void snapshot(const std::string & filename)
{
  std::ifstream in(filename);
  unsigned w, h;
  double res, ox, oy;
  Pose actual{}, returned{}, goal{};
  if (!(in >> w >> h >> res >> ox >> oy >> actual.x >> actual.y >> actual.yaw
    >> returned.x >> returned.y >> returned.yaw >> goal.x >> goal.y >> goal.yaw))
  {
    throw std::runtime_error("Invalid snapshot header");
  }
  require(w > 0 && h > 0 && w <= 10000 && h <= 10000 && res > 0,
    "Invalid snapshot dimensions");
  Map map(w, h, res, ox, oy, 0);
  unsigned index; int occupancy;
  while (in >> index >> occupancy) {
    require(index < w * h && occupancy >= -1 && occupancy <= 100,
      "Invalid snapshot cell");
    // OccupancyGrid is lossy: exact lethal/inscribed/unknown classes are retained;
    // other classes are only an approximate inverse of Nav2's publication table.
    unsigned char value = occupancy < 0 ? 255 : occupancy == 100 ? 254 :
      occupancy == 99 ? 253 : occupancy == 0 ? 0 :
      static_cast<unsigned char>(1 + (occupancy - 1) * 251 / 97);
    map.setCost(index % w, index / w, value);
  }
  require(in.eof(), "Malformed snapshot cell");
  InflationFormula formula;
  const auto shortcut = formula.circumscribedThreshold(res, 2.0, 2.0);
  std::cout << std::setprecision(17) << "{\n"
    << "  \"nav2_version\": \"1.1.20\",\n"
    << "  \"published_map_not_private_planner_map\": true,\n"
    << "  \"soft_cost_inverse_is_approximate\": true,\n"
    << "  \"global_inflation_radius\": 2.0,\n"
    << "  \"actual_inflation_shortcut_threshold\": " << shortcut << ",\n"
    << "  \"footprint\": [3.41, 1.06, -1.06, -1.06],\n";
  const std::vector<std::pair<std::string, Pose>> poses{
    {"actual", actual}, {"returned_first", returned},
    {"position_only", {returned.x, returned.y, actual.yaw}},
    {"heading_only", {actual.x, actual.y, returned.yaw}}, {"goal", goal}};
  for (unsigned i = 0; i < poses.size(); ++i) {
    const auto & name = poses[i].first;
    const auto & p = poses[i].second;
    unsigned mx, my; map.worldToMap(p.x, p.y, mx, my);
    auto c = inspect(map, p, true);
    auto optimized = inspect(map, p, true, shortcut);
    std::cout << "  \"" << name << "\": {\"pose\": [" << p.x << ", " << p.y
      << ", " << p.yaw << "], \"continuous_boundary_cost\": " << c.continuous
      << ", \"grid_collision\": " << (c.grid_collision ? "true" : "false")
      << ", \"grid_cost\": " << c.grid_cost << ", \"heading_bin\": "
      << angleBin(p.yaw) << ", \"center_cost\": "
      << static_cast<unsigned>(map.getCost(mx, my))
      << ", \"configured_shortcut_collision\": "
      << (optimized.grid_collision ? "true" : "false") << "}"
      << (i + 1 == poses.size() ? "\n" : ",\n");
  }
  std::cout << "}\n";
}

int main(int argc, char ** argv)
{
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") { controls(); }
    else if (argc == 2) { snapshot(argv[1]); }
    else { throw std::runtime_error("Use --self-test or a snapshot filename"); }
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
