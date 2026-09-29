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
  std::cout<<count<<" wall-tail geometry regression cases passed\n";
}
