#undef NDEBUG
#include <cassert>
#include "exact_near_goal.hpp"
using control_center_navigation::exact_near_goal;
int main(){
  geometry_msgs::msg::PoseStamped start,goal;
  start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=goal.pose.orientation.w=1.;
  goal.pose.position.x=.015;goal.pose.orientation.z=std::sin(.05);goal.pose.orientation.w=std::cos(.05);
  auto free=[](double,double,double){return true;};auto active=[](){return false;};
  auto path=exact_near_goal(start,goal,.02,free,active);
  assert(path && path->poses.size()>=21 && path->poses.back().pose==goal.pose);
  assert(!exact_near_goal(start,goal,.02,[](double x,double,double){return x>.001;},active));
  assert(!exact_near_goal(start,goal,.02,[](double x,double,double){return x<.014;},active));
  assert(!exact_near_goal(start,goal,.02,[](double,double,double a){return a<.04 || a>.06;},active));
  assert(!exact_near_goal(start,goal,.02,free,[]{return true;}));
  goal.pose.position.x=.041;assert(!exact_near_goal(start,goal,.02,free,active));
  goal.pose.position.x=.06;assert(!exact_near_goal(start,goal,.1,free,active));
  goal.pose.position.x=.01;goal.pose.orientation.z=std::sin(.2);goal.pose.orientation.w=std::cos(.2);
  assert(!exact_near_goal(start,goal,.02,free,active));
  goal.pose.orientation=start.pose.orientation;goal.header.frame_id="odom";
  assert(!exact_near_goal(start,goal,.02,free,active));
  goal.header.frame_id="map";assert(!exact_near_goal(start,goal,0.,free,active));
  goal.pose.position.x=NAN;assert(!exact_near_goal(start,goal,.02,free,active));
}
