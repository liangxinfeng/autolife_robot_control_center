#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
namespace control_center_navigation {
// Bounded terminal smoothing, not another search or a mandatory staging goal.
// Only replace a short forward-accessible tail when every pose is safe. Keep
// the original Smac route unchanged if geometry/clearance cannot support it.
template<class Safe>
bool align_terminal_approach(nav_msgs::msg::Path &path, Safe safe){
  if(path.poses.size()<4)return false;
  const auto goal=path.poses.back();const auto &g=goal.pose.position;
  const double yaw=tf2::getYaw(goal.pose.orientation),ux=std::cos(yaw),uy=std::sin(yaw);
  double old_length=0.;size_t index=path.poses.size()-1;
  while(index>0 && old_length<.65){
    auto &a=path.poses[index].pose.position;auto &b=path.poses[index-1].pose.position;
    old_length+=std::hypot(a.x-b.x,a.y-b.y);--index;
  }
  if(index==0 || old_length<.3 || old_length>.85)return false;
  const auto &s=path.poses[index].pose.position,&prev=path.poses[index-1].pose.position;
  double dx=g.x-s.x,dy=g.y-s.y,chord=std::hypot(dx,dy);
  // No loop, reverse entry, or hairpin to force a requested final heading.
  if(chord<.25 || dx*ux+dy*uy<.20)return false;
  double tx=s.x-prev.x,ty=s.y-prev.y,tlen=std::hypot(tx,ty);
  if(tlen<1e-5)return false;tx/=tlen;ty/=tlen;
  if(tx*dx+ty*dy<.2*chord)return false;
  const double handle=std::min(.22,chord*.35);
  double c1x=s.x+tx*handle,c1y=s.y+ty*handle,c2x=g.x-ux*handle,c2y=g.y-uy*handle;
  double lastx=s.x,lasty=s.y,lasta=std::atan2(ty,tx),length=0.;
  std::vector<geometry_msgs::msg::PoseStamped> tail;
  // <=5mm chord sampling + bounded angular increment, including footprint rotation.
  const int steps=std::max(80,int(std::ceil(old_length/.003)));
  for(int k=0;k<=steps;k++){
    const double t=double(k)/steps,v=1.-t;
    double x=v*v*v*s.x+3*v*v*t*c1x+3*v*t*t*c2x+t*t*t*g.x;
    double y=v*v*v*s.y+3*v*v*t*c1y+3*v*t*t*c2y+t*t*t*g.y;
    double vx=3*v*v*(c1x-s.x)+6*v*t*(c2x-c1x)+3*t*t*(g.x-c2x);
    double vy=3*v*v*(c1y-s.y)+6*v*t*(c2y-c1y)+3*t*t*(g.y-c2y);
    if(std::hypot(vx,vy)<.01 || vx*dx+vy*dy<=0)return false;
    double a=std::atan2(vy,vx),da=std::remainder(a-lasta,2*M_PI);
    double ds=std::hypot(x-lastx,y-lasty);length+=ds;
    if(length>old_length*1.15+.02 || (ds>1e-6 && std::abs(da)/ds>5.))return false;
    int subdivisions=std::max(1,int(std::ceil(std::max(ds/.005,std::abs(da)/.01))));
    for(int j=0;j<=subdivisions;j++){
      double f=double(j)/subdivisions;
      if(!safe(lastx+f*(x-lastx),lasty+f*(y-lasty),lasta+f*da))return false;
    }
    // Match the original grid sampling for index-based MPPI path critics.
    if(k==0 || k==steps || std::hypot(x-tail.back().pose.position.x,y-tail.back().pose.position.y)>=.02){
      auto q=goal;q.pose.position.x=x;q.pose.position.y=y;
      q.pose.orientation.z=std::sin(a/2);q.pose.orientation.w=std::cos(a/2);tail.push_back(q);
    }
    lastx=x;lasty=y;lasta=a;
  }
  tail.back()=goal;path.poses.resize(index);path.poses.insert(path.poses.end(),tail.begin(),tail.end());return true;
}
}
