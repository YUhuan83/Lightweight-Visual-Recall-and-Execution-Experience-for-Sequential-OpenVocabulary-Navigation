#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include "wildos_ackermann_smac/connector.hpp"
using namespace wildos_ackermann_smac;
int main(int argc,char**argv){
 if(argc!=2)return 2;unsigned w,h;double res,ox,oy;Pose p,g;
 std::ifstream in(argv[1]);in>>w>>h>>res>>ox>>oy>>p.x>>p.y>>p.yaw>>g.x>>g.y>>g.yaw;
 nav2_costmap_2d::Costmap2D map(w,h,res,ox,oy,0);int v;
 for(unsigned i=0;i<w*h;i++){if(!(in>>v))return 3;unsigned char c=v<0?255:v==100?254:v==99?253:v==0?0:static_cast<unsigned char>(1+std::round((v-1)*251./97.));map.setCost(i%w,i/w,c);}
 nav2_costmap_2d::Footprint f;for(auto a:std::vector<std::pair<double,double>>{{3.41,1.06},{3.41,-1.06},{-1.06,-1.06},{-1.06,1.06}}){geometry_msgs::msg::Point x;x.x=a.first;x.y=a.second;f.push_back(x);}
 for(bool free:{false,true}){
 if(free)std::fill(map.getCharMap(),map.getCharMap()+w*h,0);
 for(unsigned cap:{6000u,100000u}){
 Limits l;l.radius=12;l.max_evaluations=cap;
 auto began=Clock::now();auto a=search(map,f,p,l,began+std::chrono::seconds(3));
 std::cout<<"free="<<free<<" cap="<<cap<<" safe="<<safe(map,f,p,true)<<" evaluated="<<a.evaluated<<" candidates="<<a.curves.size()<<" timeout="<<a.timed_out<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<"\n";
 for(auto&c:a.curves)std::cout<<"length="<<c.length<<" cusps="<<c.cusps<<" endpoint="<<c.poses.back().x<<","<<c.poses.back().y<<","<<c.poses.back().yaw<<"\n";
 }
 }
}
