#include "wall_tail_filter_core.hpp"
#include <cassert>
#include <iostream>
using control_center_navigation::weakWallTailIndices;
struct Scan {
  const double a=-3.14159265,inc=2*3.14159265/719;
  std::vector<float> r=std::vector<float>(720,INFINITY),s=std::vector<float>(720,0);
  Scan(double distance=.65, double bend=.06, bool toward=true) {
    for(int i=305;i<=376;++i) {
      const double t=std::max(0.,(i-358.)/18.);
      r[i]=(distance+(toward?-1:1)*bend*t*t)/std::cos(a+i*inc);
      s[i]=i>358?3:25;
    }
  }
  auto run() const {return weakWallTailIndices(r,s,a,inc,.04,16.);}
};
int main() {
  int count=0;
  { Scan s; auto ids=s.run(); assert(ids.size()>=5); for(int i:ids) assert(i>358); ++count; }
  { Scan s(.65,0); assert(s.run().empty()); ++count; } // straight weak wall retained
  { Scan s; std::fill(s.s.begin(),s.s.end(),25); assert(s.run().empty()); ++count; }
  { Scan s; std::fill(s.s.begin(),s.s.end(),3); assert(s.run().empty()); ++count; }
  { Scan s; s.s.clear(); assert(s.run().empty()); ++count; }
  { Scan s(.30); assert(s.run().empty()); ++count; } // close obstacle retained
  { Scan s(2.); assert(s.run().empty()); ++count; }
  { Scan s(.65,.15); assert(s.run().empty()); ++count; } // large intrusion retained
  { Scan s(.65,.06,false); assert(s.run().empty()); ++count; }
  { Scan s; for(int i=305;i<=358;++i)s.r[i]=INFINITY; assert(s.run().empty()); ++count; }
  { Scan s; for(int i=377;i<400;++i){s.r[i]=.65/std::cos(s.a+i*s.inc);s.s[i]=25;}
    assert(s.run().empty()); ++count; } // separate low-intensity bump inside wall
  { Scan s; for(int i=359;i<=376;++i)s.r[i]-=.14; assert(s.run().empty()); ++count; }
  { Scan s; for(int i=305;i<=358;++i)s.r[i]+=(i%2?.02:-.02); assert(s.run().empty()); ++count; }
  { Scan s; for(int i=305;i<=376;++i)s.s[i]=NAN; assert(s.run().empty()); ++count; }
  { Scan s; for(int i=374;i<=376;++i)s.r[i]=.65/std::cos(s.a+i*s.inc);
    assert(s.run().empty()); ++count; } // round small object returning to wall
  { Scan s; std::reverse(s.r.begin(),s.r.end());std::reverse(s.s.begin(),s.s.end());
    assert(s.run().size()>=5); ++count; }
  { Scan s; s.r[364]=INFINITY;s.r[365]=INFINITY; auto ids=s.run();
    assert(ids.size()>=5); for(int i:ids)assert(i!=364 && i!=365); ++count; }
  { Scan s; for(int i=360;i<366;++i)s.r[i]=INFINITY; assert(s.run().empty()); ++count; }
  // Strong points at the weak-tail boundary are already slightly curved.
  // The fixed 32-point anchor fails its original fit; a bounded anchor search
  // must recover the weak end without deleting any of the strong transition.
  auto transition = [] {
    Scan s;
    for(int i=350;i<=358;++i){double t=(i-349.)/9.;
      s.r[i]=(.65-.025*t*t)/std::cos(s.a+i*s.inc);}
    for(int i=359;i<=376;++i){double t=(i-358.)/18.;
      s.r[i]=(.65-.025-.035*t)/std::cos(s.a+i*s.inc);}
    return s;
  };
  { auto s=transition();auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(i>=359);} ++count; }
  { auto s=transition();std::reverse(s.r.begin(),s.r.end());
    std::reverse(s.s.begin(),s.s.end());assert(s.run().size()>=5);++count; }
  { auto s=transition();for(int i=359;i<=376;++i)s.s[i]=25;
    assert(s.run().empty());++count; }
  { auto s=transition();s.r[356]=INFINITY;s.r[355]=INFINITY;s.r[354]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=transition();s.s[356]=NAN;assert(s.run().empty());++count; }
  { auto s=transition();for(int i=350;i<=376;++i)s.r[i]-=.12;
    assert(s.run().empty());++count; } // separate large inward object
  { auto s=transition();for(int i=350;i<=358;++i){
      s.r[i]=(.65-.06)/std::cos(s.a+i*s.inc);}
    assert(s.run().empty());++count; } // do not bridge a strong ledge
  { auto s=transition();s.r[355]-=.08;assert(s.run().empty());++count; }
  { auto s=transition();for(int i=359;i<=376;++i)s.s[i]=0;
    // More than 16 returns must not use the extended zero-tail certificate.
    for(int i=359;i<=376;++i){double t=(i-358.)/18.;
      s.r[i]=(.65-.025-.075*t)/std::cos(s.a+i*s.inc);}
    assert(s.run().empty());++count; }
  { auto s=transition();for(int i=359;i<=376;++i){s.s[i]=0;s.r[i]-=.13;}
    assert(s.run().empty());++count; }
  { auto s=transition();std::fill(s.s.begin(),s.s.end(),0);
    assert(s.run().empty());++count; }
  { auto s=transition();s.r[356]=INFINITY;auto ids=s.run();
    assert(ids.size()>=5);assert(std::find(ids.begin(),ids.end(),356)==ids.end());++count; }
  { auto s=transition();s.r[340]=INFINITY;s.r[330]=INFINITY;
    assert(s.run().size()>=5);++count; }
  { auto s=transition();s.r[340]=INFINITY;s.r[339]=INFINITY;s.r[338]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=transition();for(int i:{350,347,344,341,338,335,332,329,326})s.r[i]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=transition();s.r[340]=INFINITY;s.r[335]=INFINITY;s.r[330]=INFINITY;
    assert(s.run().size()>=5);++count; }
  { auto s=transition();s.r[359]=INFINITY;
    auto ids=s.run();assert(ids.size()>=5);
    assert(std::find(ids.begin(),ids.end(),359)==ids.end());++count; }
  { auto s=transition();s.r[359]=INFINITY;s.r[360]=INFINITY;s.r[361]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=transition();s.r[356]=INFINITY;s.r[355]=INFINITY;
    assert(s.run().size()>=5);++count; }
  { auto s=transition();s.r[340]=INFINITY;s.r[339]=INFINITY;
    assert(s.run().size()>=5);++count; }
  { auto s=transition();s.r[359]=INFINITY;s.r[360]=INFINITY;
    assert(s.run().size()>=5);++count; }
  { auto s=transition();for(int i=359;i<=376;++i)s.s[i]=0;
    for(int i:{360,363,366,369,372,375})s.r[i]=INFINITY;
    auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(std::isfinite(s.r[i]));} ++count; }
  { auto s=transition();for(int i:{360,363,366,369,372,375})s.r[i]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=transition();for(int i=359;i<=376;++i)s.s[i]=0;
    for(int i=363;i<=366;++i)s.r[i]=INFINITY;
    auto ids=s.run();for(int i:ids){assert(i<363);} ++count; }
  // Nearby straight wall with an oblique, more distant weak inward tail.
  // The strong anchor partly lies inside 40 cm; only the tail is eligible
  // for deletion. Rotations, intensities and close returns are independent
  // checks against accidentally broadening the protected-distance rule.
  auto nearby = [] {
    Scan s;
    std::fill(s.r.begin(),s.r.end(),INFINITY);
    std::fill(s.s.begin(),s.s.end(),0);
    for(int i=382;i<=449;++i) {
      double t=std::max(0.,(i-434.)/15.);
      s.r[i]=(.35-.05*t*t)/std::cos(s.a+i*s.inc);
      s.s[i]=i<=434?25:0;
    }
    return s;
  };
  { auto s=nearby();auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(i>434);assert(s.r[i]>=.4);}++count; }
  { auto s=nearby();std::reverse(s.r.begin(),s.r.end());
    std::reverse(s.s.begin(),s.s.end());auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(s.r[i]>=.4);}++count; }
  { auto s=nearby();for(int i=435;i<=449;++i)s.s[i]=25;
    assert(s.run().empty());++count; }
  { auto s=nearby();std::fill(s.s.begin(),s.s.end(),0);
    assert(s.run().empty());++count; }
  { auto s=nearby();for(int i=435;i<=449;++i)s.r[i]=.35/std::cos(s.a+i*s.inc);
    assert(s.run().empty());++count; }
  { auto s=nearby();s.r[449]=.39;auto ids=s.run();
    for(int i:ids){assert(s.r[i]>=.4);}++count; }
  { auto s=nearby();for(int i=435;i<=449;++i)s.r[i]-=.20;
    assert(s.run().empty());++count; }
  { auto s=nearby();for(int i=435;i<=449;++i)s.s[i]=3;
    auto ids=s.run();assert(ids.size()>=5);for(int i:ids)assert(s.r[i]>=.4);++count; }
  { Scan s;for(int i=359;i<=376;++i)s.s[i]=0;
    for(int i=374;i<=376;++i)s.r[i]=.65/std::cos(s.a+i*s.inc);
    assert(s.run().empty());++count; } // zero-strength rounded object retained
  { auto s=nearby();for(int i=450;i<=465;++i){
      s.r[i]=.35/std::cos(s.a+i*s.inc);s.s[i]=25;}
    assert(s.run().empty());++count; } // weak object inside continuing wall
  // A longer continuous bend with mixed-strength transition returns retained
  // in place. A stable strong wall is available behind that transition.
  auto mixed_transition = [] {
    Scan s(.50);
    std::fill(s.r.begin(),s.r.end(),INFINITY);
    std::fill(s.s.begin(),s.s.end(),0);
    for(int i=300;i<=370;++i) {
      double departure=0.;
      if(i>=340 && i<=358){double t=(i-339.)/19.;departure=.04*t*t;}
      if(i>358)departure=.04+.025*(i-358.)/12.;
      s.r[i]=(.50-departure)/std::cos(s.a+i*s.inc);
      s.s[i]=i>358?(i%3?0:3):25;
    }
    s.s[358]=11;s.s[348]=7;
    return s;
  };
  { auto s=mixed_transition();auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(i>358);assert(s.r[i]>=.4);}++count; }
  { auto s=mixed_transition();std::reverse(s.r.begin(),s.r.end());
    std::reverse(s.s.begin(),s.s.end());auto ids=s.run();assert(ids.size()>=5);
    for(int i:ids){assert(i<361);assert(s.r[i]>=.4);}++count; }
  { auto s=mixed_transition();for(int i=359;i<=370;++i)s.s[i]=25;
    assert(s.run().empty());++count; }
  { auto s=mixed_transition();std::fill(s.s.begin(),s.s.end(),0);
    assert(s.run().empty());++count; }
  { auto s=mixed_transition();for(int i=300;i<=339;++i)s.r[i]+=(i%2?.015:-.015);
    assert(s.run().empty());++count; }
  { auto s=mixed_transition();for(int i=340;i<=358;++i)
      s.r[i]=.46/std::cos(s.a+i*s.inc);
    assert(s.run().empty());++count; } // abrupt retained ledge, not wall distortion
  { auto s=mixed_transition();for(int i=368;i<=370;++i)
      s.r[i]=.50/std::cos(s.a+i*s.inc);
    assert(s.run().empty());++count; } // rounded object returns toward wall
  { auto s=mixed_transition();for(int i=350;i<=352;++i)s.r[i]=INFINITY;
    assert(s.run().empty());++count; }
  { auto s=mixed_transition();s.r[370]=.38;auto ids=s.run();
    for(int i:ids){assert(s.r[i]>=.4);}++count; }
  { auto s=mixed_transition();for(int i=371;i<=390;++i){
      s.r[i]=.50/std::cos(s.a+i*s.inc);s.s[i]=25;}
    assert(s.run().empty());++count; } // actual weak object inside continuing wall
  std::cout<<count<<" wall-tail geometry regression cases passed\n";
}
