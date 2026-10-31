// Installed Smac + InflationLayer. Isolated DDS, no simulation clients/actions.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

struct Pose { double x, y, yaw; };
struct Snapshot {
  unsigned w = 320, h = 320;
  double resolution = .25, ox = -40, oy = -40;
  Pose start{.125, .125, 0}, first{.125, .125, 0}, goal{10.125, .125, 0};
  std::vector<std::pair<unsigned, int>> cells;
};

Snapshot read(const char * filename)
{
  Snapshot s;
  std::ifstream in(filename);
  if (!(in >> s.w >> s.h >> s.resolution >> s.ox >> s.oy
    >> s.start.x >> s.start.y >> s.start.yaw
    >> s.first.x >> s.first.y >> s.first.yaw
    >> s.goal.x >> s.goal.y >> s.goal.yaw))
  { throw std::runtime_error("Invalid snapshot header"); }
  unsigned i; int cost;
  while (in >> i >> cost) {
    if (i >= s.w * s.h || cost < -1 || cost > 100) {
      throw std::runtime_error("Invalid snapshot cell");
    }
    s.cells.emplace_back(i, cost);
  }
  if (!in.eof()) { throw std::runtime_error("Malformed snapshot cell"); }
  return s;
}

nav2_costmap_2d::Footprint body()
{
  nav2_costmap_2d::Footprint f;
  for (auto xy : std::vector<std::pair<double, double>>{
    {3.41,1.06}, {3.41,-1.06}, {-1.06,-1.06}, {-1.06,1.06}})
  {
    geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second; f.push_back(p);
  }
  return f;
}

geometry_msgs::msg::PoseStamped pose(Pose p)
{
  geometry_msgs::msg::PoseStamped out; out.header.frame_id = "world";
  out.pose.position.x = p.x; out.pose.position.y = p.y;
  tf2::Quaternion q; q.setRPY(0,0,p.yaw); out.pose.orientation = tf2::toMsg(q);
  return out;
}

struct Result {
  unsigned path_size = 0, center_cost = 0;
  double shortcut_cost = 0, actual_body_cost = 0, returned_body_cost = 0;
  bool quantized_start_collision = false;
  std::string exception;
  nav_msgs::msg::Path path;
};

Result plan(const Snapshot & s, double radius)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("smac_snapshot_probe");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("snapshot_map");
  map->set_parameters({
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"global_frame", "world"}, {"robot_base_frame", "base"},
    {"resolution", s.resolution}, {"width", static_cast<int>(s.w*s.resolution)},
    {"height", static_cast<int>(s.h*s.resolution)}, {"origin_x", s.ox}, {"origin_y", s.oy},
    {"track_unknown_space", false}, {"footprint_padding", 0.0},
    {"footprint", "[[3.41,1.06],[3.41,-1.06],[-1.06,-1.06],[-1.06,1.06]]"}});
  // Layer parameters are not declared until configure().
  map->declare_parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
  map->declare_parameter("inflation_layer.inflation_radius", radius);
  map->declare_parameter("inflation_layer.cost_scaling_factor", 2.0);
  map->configure();
  if(map->getUseRadius()) { throw std::runtime_error("Rectangle was configured as radius"); }
  auto * grid = map->getCostmap();
  grid->resizeMap(s.w, s.h, s.resolution, s.ox, s.oy);
  map->setRobotFootprint(body());
  std::fill(grid->getCharMap(), grid->getCharMap()+s.w*s.h, 0);
  // Rebuild both cases from the SAME lethal/unknown observations. Old published
  // soft costs are deliberately discarded, not presented as private-map replay.
  for (auto cell : s.cells) {
    if (cell.second == 100 || cell.second < 0) {
      grid->setCost(cell.first%s.w, cell.first/s.w, cell.second == 100 ? 254 : 255);
    }
  }
  auto inflation = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(
    map->getLayeredCostmap()->getPlugins()->at(0));
  inflation->matchSize();
  inflation->updateCosts(*grid, 0, 0, s.w, s.h);
  for (const auto & p : std::vector<rclcpp::Parameter>{
    {"CarLike.motion_model_for_search", "REEDS_SHEPP"},
    {"CarLike.minimum_turning_radius", 8.0}, {"CarLike.angle_quantization_bins", 72},
    {"CarLike.allow_unknown", true}, {"CarLike.tolerance", 1.0},
    {"CarLike.max_iterations", 300000}, {"CarLike.max_on_approach_iterations", 1000},
    {"CarLike.max_planning_time", 1.5}, {"CarLike.analytic_expansion_ratio", 3.5},
    {"CarLike.analytic_expansion_max_length", 50.0},
    {"CarLike.non_straight_penalty", 1.2}, {"CarLike.cost_penalty", 2.0},
    {"CarLike.lookup_table_size", 20.0}})
  { node->declare_parameter(p.get_name(), p.get_parameter_value()); }
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  nav2_smac_planner::SmacPlannerHybrid planner;
  planner.configure(node, "CarLike", tf, map); planner.activate();
  Result out;
  out.shortcut_cost = nav2_smac_planner::findCircumscribedCost(map);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> check(grid);
  out.actual_body_cost = check.footprintCostAtPose(s.start.x,s.start.y,s.start.yaw,body());
  out.returned_body_cost = check.footprintCostAtPose(s.first.x,s.first.y,s.first.yaw,body());
  unsigned mx, my; grid->worldToMap(s.start.x,s.start.y,mx,my);
  out.center_cost = grid->getCost(mx,my);
  nav2_smac_planner::GridCollisionChecker collision(grid,72,node);
  collision.setFootprint(map->getRobotFootprint(),false,out.shortcut_cost);
  double bin = std::round(s.start.yaw/(2*M_PI/72));
  while(bin<0) { bin+=72; } while(bin>=72) { bin-=72; }
  out.quantized_start_collision = collision.inCollision(
    static_cast<float>(mx),static_cast<float>(my),static_cast<float>(bin),true);
  try {
    out.path = planner.createPlan(pose(s.start),pose(s.goal));
    out.path_size = out.path.poses.size();
  }
  catch (const std::exception & e) { out.exception = e.what(); }
  planner.deactivate(); planner.cleanup(); map->cleanup();
  return out;
}

void require(bool ok, const char * text) { if(!ok) { throw std::runtime_error(text); } }

void controls()
{
  Snapshot s;
  unsigned mx = static_cast<unsigned>((3.5-s.ox)/s.resolution);
  unsigned my = static_cast<unsigned>((0.0-s.oy)/s.resolution);
  s.cells.emplace_back(my*s.w+mx,100);
  auto narrow = plan(s,2.0), covered = plan(s,3.75);
  for (const auto & item : std::vector<std::pair<const char *, Result>>{
    {"narrow",narrow},{"covered",covered}})
  {
    const auto & r = item.second;
    std::cout << item.first << " center=" << r.center_cost << " threshold=" << r.shortcut_cost
      << " body=" << r.actual_body_cost << " collision=" << r.quantized_start_collision
      << " size=" << r.path_size << " exception=" << r.exception << '\n';
  }
  require(narrow.actual_body_cost==254 && narrow.center_cost==0,
    "synthetic lethal body-edge setup changed");
  require(narrow.shortcut_cost==1 && !narrow.quantized_start_collision && narrow.path_size>0,
    "actual narrow-field Smac false acceptance not reproduced");
  require(covered.center_cost>=covered.shortcut_cost && covered.quantized_start_collision
    && covered.path_size==0, "covered inflation did not reject occupied start");
  s.cells.clear();
  auto recovery = plan(s,3.75);
  require(!recovery.quantized_start_collision && recovery.path_size>0,
    "free-map/recovery planning regressed");
  std::cout << "4 actual inflation/planner controls passed\n";
}

int main(int argc, char ** argv)
{
  rclcpp::init(0,nullptr);
  try {
    if(argc==2 && std::string(argv[1])=="--self-test") { controls(); }
    else if(argc==4 || argc==5) {
      auto r = plan(read(argv[1]),std::stod(argv[2]));
      std::ofstream out(argv[3]);
      require(out.good(), "JSON output unavailable");
      out << "{\"reconstructed_lethal_map\":true,\"inflation_radius\":" << argv[2]
        << ",\"shortcut_cost\":" << r.shortcut_cost << ",\"center_cost\":" << r.center_cost
        << ",\"actual_body_cost\":" << r.actual_body_cost
        << ",\"returned_first_body_cost\":" << r.returned_body_cost
        << ",\"quantized_start_collision\":" << (r.quantized_start_collision?"true":"false")
        << ",\"path_size\":" << r.path_size << "}\n";
      if(argc==5) {
        std::ofstream csv(argv[4]); require(csv.good(), "Path output unavailable");
        csv << std::setprecision(17) << "x,y,yaw\n";
        for(const auto & p:r.path.poses) {
          csv << p.pose.position.x << ',' << p.pose.position.y << ','
            << tf2::getYaw(p.pose.orientation) << '\n';
        }
      }
      if(!r.exception.empty()) { std::cerr << "Planner rejected: " << r.exception << '\n'; }
    } else { throw std::runtime_error("Use --self-test or SNAPSHOT RADIUS OUTPUT.json [PATH.csv]"); }
    rclcpp::shutdown(); return 0;
  } catch(const std::exception & e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
}
