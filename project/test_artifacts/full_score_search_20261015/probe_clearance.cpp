#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "wildos_ackermann_smac/connector.hpp"
using namespace wildos_ackermann_smac;
int main(int argc,char**argv){
 if(argc!=2)return 2;unsigned w,h;double res,ox,oy;Pose p,g;
 std::ifstream in(argv[1]);in>>w>>h>>res>>ox>>oy>>p.x>>p.y>>p.yaw>>g.x>>g.y>>g.yaw;
 nav2_costmap_2d::Costmap2D map(w,h,res,ox,oy,0);int v;
 for(unsigned i=0;i<w*h;i++){if(!(in>>v))return 3;unsigned char c=v<0?255:v==100?254:v==99?253:v==0?0:static_cast<unsigned char>(1+std::round((v-1)*251./97.));map.setCost(i%w,i/w,c);}
 nav2_costmap_2d::Footprint f;for(auto a:std::vector<std::pair<double,double>>{{3.41,1.06},{3.41,-1.06},{-1.06,-1.06},{-1.06,1.06}}){geometry_msgs::msg::Point x;x.x=a.first;x.y=a.second;f.push_back(x);}
 std::cout.precision(12);std::cout<<"start="<<p.x<<","<<p.y<<","<<p.yaw<<" safe="<<safe(map,f,p,true)<<" resolution="<<res<<"\n";
 for(double travel:{-.5,-.2,-.1,-.05,0.,.05,.1,.2,.5}){Pose q{p.x+travel*cos(p.yaw),p.y+travel*sin(p.yaw),p.yaw};std::cout<<"straight="<<travel<<" safe="<<safe(map,f,q,true)<<"\n";}
 nav2_costmap_2d::Costmap2D free_map(map);std::fill(free_map.getCharMap(),free_map.getCharMap()+w*h,0);
 Limits l;l.radius=12;l.max_evaluations=100000;auto a=search(free_map,f,p,l,Clock::now()+std::chrono::seconds(3));
 for(auto&c:a.curves){
  for(size_t i=0;i<c.poses.size();i++)if(!safe(map,f,c.poses[i],true)){
   auto q=c.poses[i];unsigned x,y;bool inside=map.worldToMap(q.x,q.y,x,y);
   std::cout<<"free_curve_first_failure index="<<i<<" length="<<c.length<<" cusps="<<c.cusps<<" pose="<<q.x<<","<<q.y<<","<<q.yaw<<" travel_from_start="<<hypot(q.x-p.x,q.y-p.y)<<" center_cost="<<(inside?int(map.getCost(x,y)):-1)<<"\n";
   break;
  }
 }
 for(unsigned y=0;y<h;y++)for(unsigned x=0;x<w;x++){
  auto cost=map.getCost(x,y);if(cost!=254 && cost!=253)continue;
  double wx,wy;map.mapToWorld(x,y,wx,wy);double dx=wx-p.x,dy=wy-p.y;
  double bx=dx*cos(p.yaw)+dy*sin(p.yaw),by=-dx*sin(p.yaw)+dy*cos(p.yaw);
  if(bx>-1.8 && bx<4.2 && fabs(by)<1.8)std::cout<<"near_lethal cost="<<int(cost)<<" world="<<wx<<","<<wy<<" body="<<bx<<","<<by<<"\n";
 }
}
