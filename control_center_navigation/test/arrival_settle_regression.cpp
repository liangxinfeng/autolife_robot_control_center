#include "arrival_settle_policy.hpp"
#include <stdexcept>
#include <limits>
using control_center_navigation::ArrivalSettlePolicy;
void require(bool ok){if(!ok)throw std::runtime_error("settling regression");}
int main(){
 ArrivalSettlePolicy p;
 require(!p.hold(.041,.02));require(!p.hold(.02,.041));
 require(p.hold(.04,.04));require(p.hold(.045,.045));
 require(!p.hold(.051,.03));require(!p.hold(.041,.02));
 require(p.hold(.02,.02));require(!p.hold(.02,.051));
 require(p.hold(.02,.02));p.reset();require(!p.holding);
 require(!p.hold(std::numeric_limits<double>::quiet_NaN(),0));
 require(!p.hold(0,std::numeric_limits<double>::infinity()));
 for(double d=0;d<1;d+=.001){
  require(ArrivalSettlePolicy::percentage(d,.45,20)<=20);
  require(ArrivalSettlePolicy::percentage(d,.45,100)<=100);
  require(ArrivalSettlePolicy::percentage(d,.45,100)<=ArrivalSettlePolicy::percentage(d+.001,.45,100));
 }
}
