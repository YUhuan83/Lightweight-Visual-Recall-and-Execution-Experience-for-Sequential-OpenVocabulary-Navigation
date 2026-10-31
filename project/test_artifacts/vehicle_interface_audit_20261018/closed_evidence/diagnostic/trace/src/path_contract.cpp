// SPDX-License-Identifier: Apache-2.0
#include "wildos_ackermann_smac_trace/path_contract.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace wildos_ackermann_smac_trace {
double maneuverCost(const std::vector<Pose> & path) {
  if (path.size() < 2) { return std::numeric_limits<double>::infinity(); }
  double cost = 0., previous_dx = 0., previous_dy = 0.;
  bool previous = false;
  for (size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y) ||
      !std::isfinite(path[i].yaw)) { return std::numeric_limits<double>::infinity(); }
    if (!i) { continue; }
    const double dx = path[i].x - path[i-1].x, dy = path[i].y - path[i-1].y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) { continue; }
    cost += length;
    if (previous && dx * previous_dx + dy * previous_dy < 0.) { cost += 3.; }
    previous_dx = dx; previous_dy = dy; previous = true;
  }
  return previous ? cost : std::numeric_limits<double>::infinity();
}
namespace {
double distance(const Pose & a, const Pose & b) { return std::hypot(a.x-b.x,a.y-b.y); }
double delta(double a,double b) { return std::remainder(a-b,2.*std::acos(-1.)); }
bool sectionFeasible(const std::vector<Pose> & all,size_t first,size_t last,
  double resolution,double radius,unsigned bins)
{
  std::vector<Pose> p(all.begin()+first,all.begin()+last+1);
  const size_t size=p.size();
  const double bin=bins?2.*std::acos(-1.)/bins:0.;
  const double micro=std::min(.05,std::max(.01,resolution*.1));
  std::vector<double> traveled(size,0.), headings;
  std::vector<bool> quantized;
  bool any_quantized=false;
  for(size_t i=0;i<size;i++) {
    headings.push_back(p[i].yaw);
    bool q=bin>0 && std::abs(p[i].yaw/bin-std::round(p[i].yaw/bin))<1e-4;
    quantized.push_back(q);any_quantized|=q;
    if(i) {
      const double length=distance(p[i-1],p[i]),turn=std::abs(delta(p[i].yaw,p[i-1].yaw));
      traveled[i]=traveled[i-1]+(turn>1e-6?length*turn/(2.*std::sin(turn/2.)):length);
    }
  }
  if(any_quantized) {
    for(size_t n=1;n<size;n++) {
      const bool boundary=n==1 || n==size-1 || (n+1<size &&
        (p[n].x-p[n-1].x)*(p[n+1].x-p[n].x)+(p[n].y-p[n-1].y)*(p[n+1].y-p[n].y)<0);
      if(boundary && quantized[n-1] && quantized[n] && distance(p[n-1],p[n])<micro &&
        std::abs(delta(p[n].yaw,p[n-1].yaw))<=bin+1e-6) { headings[n]=headings[n-1]; }
    }
  }
  for(size_t i=0;i+1<size;i++) {
    size_t j=i+1;
    while(j+1<size && traveled[j]-traveled[i]<2.) { ++j; }
    const double arc=traveled[j]-traveled[i];
    if(arc<.5) { continue; }
    const double uncertainty=bin*.5*(int(quantized[i])+int(quantized[j]));
    const double turn=std::max(0.,std::abs(delta(headings[j],headings[i]))-uncertainty);
    if(turn/arc>1./radius+1e-6) { return false; }
  }
  if(any_quantized) {
    std::vector<int> directions;
    for(size_t n=0;n+1<size;n++) {
      directions.push_back((p[n+1].x-p[n].x)*std::cos(p[n].yaw)+
        (p[n+1].y-p[n].y)*std::sin(p[n].yaw)>=0?1:-1);
    }
    for(size_t i=0;i+2<size;i++) {
      size_t j=i+1;
      while(j+1<size && distance(p[i],p[j])<1.) { ++j; }
      size_t k=j+1;
      while(k+1<size && distance(p[j],p[k])<1.) { ++k; }
      if(k>=size || distance(p[j],p[k])<1.) { continue; }
      bool mixed=false,micro_fit=false;
      for(size_t n=i;n<k;n++) { mixed|=directions[n]!=directions[i]; }
      if(mixed) { continue; }
      for(size_t n:{i,j,k}) {
        micro_fit|=(n>0 && distance(p[n-1],p[n])<micro) ||
          (n+1<size && distance(p[n],p[n+1])<micro);
      }
      if(micro_fit) { continue; }
      const double product=distance(p[i],p[j])*distance(p[j],p[k])*distance(p[k],p[i]);
      if(product<1e-9) { continue; }
      const double curvature=2.*std::abs((p[j].x-p[i].x)*(p[k].y-p[i].y)-
        (p[j].y-p[i].y)*(p[k].x-p[i].x))/product;
      if(curvature>1./radius+2e-5) { return false; }
    }
  }
  return true;
}
}
bool curvatureFeasible(const std::vector<Pose> & p,double resolution,double radius,unsigned bins)
{
  if(p.size()<2 || !std::isfinite(radius) || radius<=0 ||
    !std::isfinite(resolution) || resolution<=0) { return false; }
  for(const auto & pose:p) {
    if(!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) { return false; }
  }
  size_t first=0;double prev_x=0.,prev_y=0.;bool have_previous=false;
  for(size_t n=0;n+1<p.size();n++) {
    const double dx=p[n+1].x-p[n].x,dy=p[n+1].y-p[n].y;
    if(std::hypot(dx,dy)<=1e-9) { continue; }
    if(have_previous && prev_x*dx+prev_y*dy<0) {
      if(!sectionFeasible(p,first,n,resolution,radius,bins)) { return false; }
      first=n;
    }
    prev_x=dx;prev_y=dy;have_previous=true;
  }
  return sectionFeasible(p,first,p.size()-1,resolution,radius,bins);
}
}
