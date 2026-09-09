#include <cmath>
#include <iostream>
#include <stdexcept>
#include "dock_projection.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"

void require(bool ok, const char * message) {
  if (!ok) {throw std::runtime_error(message);}
}

int main() {
  // Four room headings and both drive directions: projection never flips sides.
  for (double yaw : {0., M_PI_2, M_PI, -M_PI_2}) {
    for (bool backwards : {false, true}) {
      geometry_msgs::msg::Pose dock;
      dock.position.x = 2.; dock.position.y = 3.;
      dock.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      const auto p = projectDockControlTarget(dock, backwards);
      require(std::abs(p.position.x - (2.+.25*std::cos(yaw))) < 1e-9, "projection X");
      require(std::abs(p.position.y - (3.+.25*std::sin(yaw))) < 1e-9, "projection Y");
    }
  }
  geometry_msgs::msg::Pose dock;
  dock.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
  const auto corrected = projectDockControlTarget(dock, true);
  auto old = corrected;
  old.position.x = .25;  // actual 1.3.10 flip-before-projection result
  for (bool fixed : {false, true}) {
    int successful = 0;
    for (double lateral : {-.03, 0., .03}) {
      for (double heading : {-.04, 0., .04}) {
        nav2_graceful_controller::SmoothControlLaw law(3.,2.,.5,2.,.32,.025,.07,.12);
        const auto target = fixed ? corrected : old;
        double x=.55, y=lateral, yaw=heading;
        double max_w=0.;
        for (int k=0; k<750 && x>0.001; ++k) {
          // Deterministic small feedback disturbance, no robot/ROS publisher.
          const double noise=.001*std::sin(k*.21);
          geometry_msgs::msg::Pose measured;
          measured.position.x=x; measured.position.y=y+noise;
          measured.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw);
          const auto cmd=law.calculateRegularVelocity(target,measured,true);
          require(std::isfinite(cmd.linear.x)&&std::isfinite(cmd.angular.z),"nonfinite command");
          require(cmd.linear.x<=1e-9,"unexpected forward command");
          max_w=std::max(max_w,std::abs(cmd.angular.z));
          x+=cmd.linear.x*std::cos(yaw)/30.;
          y+=cmd.linear.x*std::sin(yaw)/30.;
          yaw+=cmd.angular.z/30.;
          if(fixed) {
            require(std::abs(y)<.06,"lateral guard exceeded");
            require(std::abs(yaw)<.14,"heading guard exceeded");
          }
        }
        if(x<=.001) ++successful;
        if(fixed) require(x<=.001,"did not reach contact within 25 s");
        std::cout<<(fixed?"fixed":"old")<<" lateral="<<lateral<<" yaw="<<heading
          <<" remaining="<<x<<" y="<<y<<" max_w="<<max_w<<"\n";
      }
    }
    std::cout<<(fixed?"fixed":"old")<<" reached="<<successful<<"/9\n";
    if(!fixed) require(successful==0,"old bug not reproduced");
  }
}
