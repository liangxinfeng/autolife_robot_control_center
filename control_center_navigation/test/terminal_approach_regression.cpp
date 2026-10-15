#include "terminal_approach.hpp"
#include <stdexcept>
using namespace control_center_navigation;
void require(bool ok){if(!ok)throw std::runtime_error("terminal approach regression");}
nav_msgs::msg::Path line(double goal_yaw=0){
 nav_msgs::msg::Path p;p.header.frame_id="map";
 for(int i=0;i<=50;i++){
  geometry_msgs::msg::PoseStamped q;q.header=p.header;q.pose.position.x=i*.02;q.pose.position.y=.10*(1-i/50.);
  q.pose.orientation.w=1.;p.poses.push_back(q);
 }
 p.poses.back().pose.orientation.z=sin(goal_yaw/2);p.poses.back().pose.orientation.w=cos(goal_yaw/2);return p;
}
int main(){
 auto p=line(),original=p;size_t checked=0;
 require(align_terminal_approach(p,[&](double x,double y,double yaw){++checked;return std::isfinite(x+y+yaw);}));
 require(checked>200);require(p.header==original.header);require(p.poses.front()==original.poses.front());require(p.poses.back()==original.poses.back());
 auto &a=p.poses[p.poses.size()-2].pose.position,&b=p.poses.back().pose.position;
 require(std::abs(atan2(b.y-a.y,b.x-a.x))<.025);
 auto blocked=line(),saved=blocked;
 require(!align_terminal_approach(blocked,[](double x,double,double){return !(x>.78 && x<.8);}));require(blocked==saved);
 auto backwards=line(M_PI),copy=backwards;
 require(!align_terminal_approach(backwards,[](double,double,double){return true;}));require(backwards==copy);
 nav_msgs::msg::Path empty;require(!align_terminal_approach(empty,[](double,double,double){return true;}));
 auto short_path=line();short_path.poses.erase(short_path.poses.begin(),short_path.poses.begin()+40);auto short_copy=short_path;
 require(!align_terminal_approach(short_path,[](double,double,double){return true;}));require(short_path==short_copy);
}
