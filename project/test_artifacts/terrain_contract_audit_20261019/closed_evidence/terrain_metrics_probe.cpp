#include <graphnav_mapper/terrain_metrics.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
int main() {
  std::vector<Eigen::Vector3d> points;
  double x,y,z;
  while (std::cin >> x >> y >> z) points.emplace_back(x,y,z);
  const auto m=graphnav_mapper::terrain_metrics(points);
  std::cout << std::setprecision(17) << "{\"observed\":" << (m.observed?"true":"false")
    << ",\"slope\":" << m.slope << ",\"step\":" << m.step << ",\"roughness\":" << m.roughness << "}\n";
}
