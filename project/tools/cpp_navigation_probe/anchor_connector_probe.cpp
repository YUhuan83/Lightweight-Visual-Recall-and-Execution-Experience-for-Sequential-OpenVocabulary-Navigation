// Feasibility diagnostic only. Real OMPL curves and Nav2 map/angle checks.
// No ROS node, action clients, simulator clients or vehicle movement.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

using Map = nav2_costmap_2d::Costmap2D;
struct Pose { double x,y,yaw; };
constexpr double RADIUS = 8.0;
constexpr unsigned BINS = 72;
constexpr double MIN_SECTION = .4;  // Twice the existing 0.2 m cusp disk.

struct Snapshot {
  std::unique_ptr<Map> map;
  Pose actual, first, goal;
};

Snapshot read(const char * filename)
{
  std::ifstream in(filename);
  unsigned w,h; double res,ox,oy; Snapshot s;
  if(!(in>>w>>h>>res>>ox>>oy>>s.actual.x>>s.actual.y>>s.actual.yaw
    >>s.first.x>>s.first.y>>s.first.yaw>>s.goal.x>>s.goal.y>>s.goal.yaw))
  { throw std::runtime_error("Invalid snapshot"); }
  if(w==0 || h==0 || w>10000 || h>10000 || !std::isfinite(res) || res<=0)
  { throw std::runtime_error("Invalid map dimensions"); }
  s.map=std::make_unique<Map>(w,h,res,ox,oy,0);
  unsigned index; int cost;
  while(in>>index>>cost) {
    if(index>=w*h || cost < -1 || cost>100) { throw std::runtime_error("Invalid cell"); }
    unsigned char value=cost<0?255:cost==100?254:cost==99?253:cost==0?0:
      static_cast<unsigned char>(1+(cost-1)*251/97);
    s.map->setCost(index%w,index/w,value);
  }
  if(!in.eof()) { throw std::runtime_error("Malformed cell"); }
  return s;
}

nav2_costmap_2d::Footprint body()
{
  nav2_costmap_2d::Footprint f;
  for(auto xy:std::vector<std::pair<double,double>>{
    {3.41,1.06},{3.41,-1.06},{-1.06,-1.06},{-1.06,1.06}})
  { geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; f.push_back(p); }
  return f;
}

bool safe(Map & map, Pose p)
{
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) { return false; }
  unsigned mx,my;
  if(!map.worldToMap(p.x,p.y,mx,my)) { return false; }
  auto center=map.getCost(mx,my);
  if(center==253||center==254) { return false; }
  const auto f=body();
  std::vector<std::pair<unsigned,unsigned>> corners;
  double c=std::cos(p.yaw),s=std::sin(p.yaw);
  unsigned minx=map.getSizeInCellsX(),miny=map.getSizeInCellsY(),maxx=0,maxy=0;
  for(const auto & point:f) {
    if(!map.worldToMap(p.x+c*point.x-s*point.y,p.y+s*point.x+c*point.y,mx,my))
    { return false; }
    corners.emplace_back(mx,my);
    minx=std::min(minx,mx);miny=std::min(miny,my);
    maxx=std::max(maxx,mx);maxy=std::max(maxy,my);
  }
  // Nav2's exact integer boundary cells, plus lethal interior cell centers.
  // Unknown cells remain traversable; they never mask a known lethal cell.
  for(unsigned i=0;i<4;++i) {
    auto a=corners[i],b=corners[(i+1)%4];
    for(nav2_util::LineIterator line(a.first,a.second,b.first,b.second);
      line.isValid();line.advance())
    { if(map.getCost(line.getX(),line.getY())==254) { return false; } }
  }
  for(unsigned y=miny;y<=maxy;++y) {
    for(unsigned x=minx;x<=maxx;++x) {
      if(map.getCost(x,y)!=254) { continue; }
      double wx,wy;map.mapToWorld(x,y,wx,wy);
      double dx=wx-p.x,dy=wy-p.y,forward=c*dx+s*dy,left=-s*dx+c*dy;
      if(forward>=-1.06 && forward<=3.41 && std::abs(left)<=1.06) { return false; }
    }
  }
  return true;
}

struct Curve { std::vector<Pose> poses; double length=0; unsigned cusps=0; };

Curve curve(Pose from,Pose to)
{
  ompl::base::ReedsSheppStateSpace space(RADIUS);
  auto a=space.allocState(),b=space.allocState();
  auto sa=a->as<ompl::base::SE2StateSpace::StateType>();
  auto sb=b->as<ompl::base::SE2StateSpace::StateType>();
  sa->setXY(from.x,from.y);sa->setYaw(from.yaw);
  sb->setXY(to.x,to.y);sb->setYaw(to.yaw);
  const auto path=space.reedsShepp(a,b);
  space.freeState(a);space.freeState(b);
  Curve out;out.length=path.length()*RADIUS;
  double section=0;int previous=0;
  for(unsigned i=0;i<5;++i) {
    const double length=path.length_[i]*RADIUS;
    if(std::abs(length)<1e-9) { continue; }
    int direction=length>0?1:-1;
    if(previous && previous!=direction) {
      if(section<=MIN_SECTION) { return {}; }
      ++out.cusps;section=0;
    }
    section+=std::abs(length);previous=direction;
  }
  if(section<=MIN_SECTION) { return {}; }
  out.poses.push_back(from);
  Pose current=from;
  for(unsigned i=0;i<5;++i) {
    double length=path.length_[i]*RADIUS;
    if(std::abs(length)<1e-9) { continue; }
    double k=path.type_[i]==ompl::base::ReedsSheppStateSpace::RS_LEFT?1/RADIUS:
      path.type_[i]==ompl::base::ReedsSheppStateSpace::RS_RIGHT?-1/RADIUS:0;
    unsigned steps=static_cast<unsigned>(std::ceil(std::abs(length)/.05));
    for(unsigned j=1;j<=steps;++j) {
      double travel=length*j/steps,angle=k*travel;
      double x=k==0?travel:std::sin(angle)/k;
      double y=k==0?0:(1-std::cos(angle))/k;
      out.poses.push_back({current.x+x*std::cos(current.yaw)-y*std::sin(current.yaw),
        current.y+x*std::sin(current.yaw)+y*std::cos(current.yaw),current.yaw+angle});
    }
    current=out.poses.back();
  }
  double yaw_error=std::atan2(std::sin(current.yaw-to.yaw),std::cos(current.yaw-to.yaw));
  if(std::hypot(current.x-to.x,current.y-to.y)>1e-8||std::abs(yaw_error)>1e-8)
  { throw std::runtime_error("Curve integration did not reach exact anchor"); }
  return out;
}

void require(bool ok,const char * why) { if(!ok) { throw std::runtime_error(why); } }

void controls()
{
  Map map(320,320,.25,-40,-40,0);
  for(double direction:{-1.,1.}) {
    auto c=curve({0,0,0},{direction*2,0,0});
    require(c.poses.size()>2 && std::abs(c.length-2)<1e-8 && c.cusps==0,
      "forward/reverse exact straight connection failed");
    for(const auto & p:c.poses) { require(safe(map,p),"free-map curve rejected"); }
  }
  auto curved=curve({0,0,0},{RADIUS*std::sin(.25),RADIUS*(1-std::cos(.25)),.25});
  require(curved.poses.size()>2 && std::abs(curved.length-2)<1e-8,
    "radius-8 exact arc failed");
  require(curve({0,0,0},{.2,0,0}).poses.empty(),"short section inside cusp disk accepted");
  unsigned x,y;map.worldToMap(2.,0.,x,y);map.setCost(x,y,254);
  require(!safe(map,{0,0,0}),"lethal body interior ignored");
  map.setCost(x,y,0);map.worldToMap(3.5,0,x,y);map.setCost(x,y,254);
  require(!safe(map,{.125,.125,0}),"lethal Nav2 body boundary ignored");
  map.setCost(x,y,0);
  require(safe(map,{.125,.125,0}),"removing obstacle did not restore geometry");
  require(!safe(map,{100,0,0}),"outside-map accepted");
  std::cout<<"8 connector controls passed\n";
}

int main(int argc,char ** argv)
{
  try {
    if(argc==2 && std::string(argv[1])=="--self-test") { controls(); return 0; }
    if(argc!=5) { throw std::runtime_error("Use GLOBAL LOCAL CSV SUMMARY.json"); }
    auto global=read(argv[1]),local=read(argv[2]);
    require(std::hypot(global.actual.x-local.actual.x,global.actual.y-local.actual.y)<1e-9,
      "snapshot actual poses differ");
    require(safe(*global.map,global.actual)&&safe(*local.map,global.actual),
      "actual start is not collision-free in both maps");
    nav2_smac_planner::GridCollisionChecker checker(global.map.get(),BINS,nullptr);
    checker.setFootprint(body(),false,0);
    struct Candidate { Pose anchor; Curve path; double score; };
    std::vector<Candidate> candidates;
    unsigned actualx,actualy;
    global.map->worldToMap(global.actual.x,global.actual.y,actualx,actualy);
    int span=static_cast<int>(std::ceil((RADIUS/2)/global.map->getResolution()));
    unsigned evaluated=0,collision_rejected=0,geometry_rejected=0;
    for(int y=static_cast<int>(actualy)-span;y<=static_cast<int>(actualy)+span;++y) {
      for(int x=static_cast<int>(actualx)-span;x<=static_cast<int>(actualx)+span;++x) {
        if(x<0||y<0||x>=static_cast<int>(global.map->getSizeInCellsX())
          ||y>=static_cast<int>(global.map->getSizeInCellsY())) { continue; }
        double wx,wy;global.map->mapToWorld(x,y,wx,wy);
        if(std::hypot(wx-global.actual.x,wy-global.actual.y)>RADIUS/2) { continue; }
        for(unsigned bin=0;bin<BINS;++bin) {
          ++evaluated;
          if(checker.inCollision(static_cast<float>(x),static_cast<float>(y),
            static_cast<float>(bin),true)) { ++collision_rejected;continue; }
          // Native NodeHybrid return-heading precision, not only cached checker yaw.
          float angle=bin*(2.f*static_cast<float>(M_PI)/static_cast<float>(BINS));
          Pose anchor{wx,wy,angle};
          auto path=curve(global.actual,anchor);
          if(path.poses.empty()||path.length>RADIUS) { ++geometry_rejected;continue; }
          bool collision=false;
          for(const auto & p:path.poses) {
            if(!safe(*global.map,p)||!safe(*local.map,p)) { collision=true;break; }
          }
          if(collision) { ++collision_rejected;continue; }
          candidates.push_back({anchor,path,path.length+3.*path.cusps});
        }
      }
    }
    std::sort(candidates.begin(),candidates.end(),[](const Candidate & a,const Candidate & b){
      return a.score<b.score;
    });
    std::ofstream csv(argv[3]),summary(argv[4]);
    require(csv.good()&&summary.good(),"output file unavailable");
    csv<<std::setprecision(17)<<"candidate,x,y,yaw\n";
    summary<<std::setprecision(17)<<"{\"scope\":\"geometry_only_no_execution\","
      <<"\"evaluated\":"<<evaluated<<",\"collision_rejected\":"<<collision_rejected
      <<",\"geometry_rejected\":"<<geometry_rejected<<",\"feasible_count\":"<<candidates.size()
      <<",\"candidates\":[";
    unsigned count=std::min(static_cast<unsigned>(candidates.size()),20u);
    for(unsigned i=0;i<count;++i) {
      const auto & c=candidates[i];
      summary<<(i?",":"")<<"{\"anchor\":["<<c.anchor.x<<","<<c.anchor.y<<","<<c.anchor.yaw
        <<"],\"length\":"<<c.path.length<<",\"cusps\":"<<c.path.cusps<<"}";
      for(const auto & p:c.path.poses) { csv<<i<<","<<p.x<<","<<p.y<<","<<p.yaw<<"\n"; }
    }
    summary<<"]}\n";
    std::cout<<"feasible="<<candidates.size()<<" evaluated="<<evaluated<<"\n";
    return candidates.empty()?2:0;
  } catch(const std::exception & e) { std::cerr<<e.what()<<'\n';return 1; }
}
