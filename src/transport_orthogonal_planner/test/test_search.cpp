// SPDX-License-Identifier: Apache-2.0
#include "transport_orthogonal_planner/search.hpp"
#include <iostream>
#include <random>
#include <thread>
using namespace transport_orthogonal_planner;

void require(bool ok, const char * text) {if (!ok) {throw std::runtime_error(text);}}
Grid empty(int n=20)
{return {n,n,0.1,-0.1,-0.1,std::vector<uint8_t>(static_cast<size_t>(n)*n,0)};}
void block(Grid & g,int x,int y,uint8_t value=254)
{g.costs[static_cast<size_t>(y)*g.width+x]=value;}
void validatePhysicalClearance(const Grid & g,const Result & path,double radius)
{
  // Independent swept-circle check against original lethal cell rectangles.
  for(size_t k=1;k<path.points.size();++k) {
    const auto a=path.points[k-1],b=path.points[k];
    const double lx=std::min(a.x,b.x),hx=std::max(a.x,b.x);
    const double ly=std::min(a.y,b.y),hy=std::max(a.y,b.y);
    require(lx-radius>=g.origin_x && ly-radius>=g.origin_y &&
            hx+radius<=g.origin_x+g.width*g.resolution &&
            hy+radius<=g.origin_y+g.height*g.resolution,"footprint outside map");
    for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x) {
      if(g.costs[static_cast<size_t>(y)*g.width+x]<254)continue;
      const double x0=g.origin_x+x*g.resolution,y0=g.origin_y+y*g.resolution;
      const double dx=std::max({x0-hx,lx-(x0+g.resolution),0.0});
      const double dy=std::max({y0-hy,ly-(y0+g.resolution),0.0});
      require(dx*dx+dy*dy>radius*radius,"physical swept-circle collision");
    }
  }
}
void validate(Search & s,const Result & r,Point a,Point b)
{
  require(!r.points.empty(),"empty result");
  require(r.points.front().x==a.x && r.points.front().y==a.y,"start changed");
  require(r.points.back().x==b.x && r.points.back().y==b.y,"goal changed");
  for (size_t i=1;i<r.points.size();++i) {
    Point p=r.points[i-1],q=r.points[i];
    require((p.x==q.x)!=(p.y==q.y),"diagonal or duplicate segment");
    require(s.segmentFree(p,q),"collision");
    if(i>=2) {
      Point z=r.points[i-2];
      require(!((z.x==p.x&&p.x==q.x)||(z.y==p.y&&p.y==q.y)),"uncompressed path");
    }
  }
}
template<typename F> void expect(ErrorCode code,F fn)
{
  try {fn();} catch(const SearchError & e) {require(e.code==code,"wrong failure code"); return;}
  throw std::runtime_error("expected failure did not occur");
}
int main()
{
  try {
    Options o; o.timeout=10;
    {
      auto g=empty(); Search s(g,o);
      Point a{0.123,0.217},b{1.334,1.527};
      auto r=s.plan(a,b); validate(s,r,a,b);
      require(r.points.size()==3,"empty scene should need one turn");
      require(std::abs(r.cost-(std::abs(b.x-a.x)+std::abs(b.y-a.y)+o.turn_penalty))<1e-9,
              "unexpected cost");
    }
    {
      auto g=empty(); Search s(g,o); Point a{0.13,0.21},b{1.33,0.21};
      auto r=s.plan(a,b);validate(s,r,a,b);require(r.points.size()==2,"straight route split");
      auto same=s.plan(a,a);require(same.points.size()==1,"same point failed");
    }
    {
      auto g=empty();for(int y=0;y<14;++y)block(g,9,y);
      Search s(g,o);Point a{0.25,0.25},b{1.45,0.25};
      auto r=s.plan(a,b);validate(s,r,a,b);
      require(r.points.size()>=4,"wall did not cause detour");
    }
    {
      auto g=empty();for(int y=0;y<g.height;++y)block(g,9,y);
      Search s(g,o);expect(ErrorCode::NoPath,[&]{s.plan({0.25,0.25},{1.45,0.25});});
    }
    {
      auto g=empty();block(g,10,10);Search s(g,o);
      expect(ErrorCode::StartOccupied,[&]{s.plan({0.95,0.95},{1.45,0.25});});
      expect(ErrorCode::GoalOccupied,[&]{s.plan({0.25,0.25},{0.95,0.95});});
      expect(ErrorCode::StartOutside,[&]{s.plan({-1,0.25},{1.45,0.25});});
      expect(ErrorCode::GoalOutside,[&]{s.plan({0.25,0.25},{2.1,0.25});});
      // A line exactly touching the blocked cell boundary must not be accepted.
      require(!s.segmentFree({0.90,0.80},{0.90,1.10}),"boundary supercover failed");
    }
    {
      auto g=empty();block(g,10,10,255);Search s(g,o);
      expect(ErrorCode::GoalOccupied,[&]{s.plan({0.25,0.25},{0.95,0.95});});
    }
    {
      auto g=empty();block(g,10,10);Options footprint=o;footprint.radius=0.15;
      Search s(g,footprint);
      expect(ErrorCode::StartOccupied,[&]{s.plan({-0.05,0.25},{1.45,1.45});});
      expect(ErrorCode::GoalOccupied,[&]{s.plan({0.25,0.25},{0.75,0.95});});
      Point a{0.25,0.25},b{1.45,1.45};auto path=s.plan(a,b);validate(s,path,a,b);
      validatePhysicalClearance(g,path,footprint.radius);
    }
    {
      auto g=empty();bool cancelled=false;Search s(g,o,[&]{return cancelled;});
      cancelled=true;expect(ErrorCode::Cancelled,[&]{s.plan({0.2,0.2},{1.2,1.2});});
      Options timed=o;timed.timeout=0.001;
      expect(ErrorCode::Timeout,[&]{Search s2(g,timed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        s2.plan({0.2,0.2},{1.2,1.2});});
      Options limited=o;limited.max_expansions=1;Search s3(g,limited);
      expect(ErrorCode::Timeout,[&]{s3.plan({0.2,0.2},{1.2,1.2});});
    }
    {
      std::mt19937 generator(42);
      for(int trial=0;trial<50;++trial) {
        auto g=empty();for(int i=0;i<30;++i)block(g,3+generator()%14,3+generator()%14);
        Search s(g,o);Point a{0.123,0.127},b{1.723,1.727};
        try {validate(s,s.plan(a,b),a,b);} catch(const SearchError & e) {
          require(e.code==ErrorCode::NoPath,"random test unexpected error");
        }
      }
    }
    {
      auto g=empty(260);g.resolution=0.01;
      for(int y=80;y<180;++y)for(int x=80;x<180;++x)block(g,x,y);
      Options real=o;real.radius=0.15;real.timeout=3;
      auto begin=std::chrono::steady_clock::now();Search s(g,real);
      Point a{0.30,0.30},b{2.1,2.1};auto r=s.plan(a,b);validate(s,r,a,b);
      validatePhysicalClearance(g,r,real.radius);
      std::cout<<"260x260 map: "<<r.expansions<<" expansions, "<<r.points.size()<<" points, "
               <<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<" s\n";
    }
    std::cout<<"PASS: exact endpoints, orthogonality, turns, obstacle/unknown clearance, "
             <<"bounds, no-path, cancellation, time/iteration limits, 50 random maps\n";
    return 0;
  } catch(const std::exception & e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
